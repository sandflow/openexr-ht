#include <string>
#include <map>
#include <chrono>
#include <numeric>
#include <cmath>
#include <iterator>
#include <mutex>

#include "ImfArray.h"
#include "ImfCompression.h"
#include "ImfHeader.h"
#include "ImfRgbaFile.h"
#include "ImfMultiPartInputFile.h"
#include "ImfMultiPartOutputFile.h"
#include "ImfInputPart.h"
#include <ImfOutputFile.h>
#include <ImfOutputPart.h>
#include "ImfChannelList.h"
#include "ImfMisc.h"
#include "IlmThread.h"
#include "IlmThreadPool.h"
#include "ImfFrameBuffer.h"
#include <ImfNamespace.h>
#include <OpenEXRConfig.h>

#include "cxxopts.hpp"

namespace IMF = OPENEXR_IMF_NAMESPACE;
using namespace OPENEXR_IMF_NAMESPACE;
using namespace IMATH_NAMESPACE;
using namespace ILMTHREAD_NAMESPACE;

static std::map<std::string, Compression> comp_table = {
    {{"NO_COMPRESSION", NO_COMPRESSION},
     {"RLE_COMPRESSION", RLE_COMPRESSION},
     {"ZIPS_COMPRESSION", ZIPS_COMPRESSION},
     {"ZIP_COMPRESSION", ZIP_COMPRESSION},
     {"PIZ_COMPRESSION", PIZ_COMPRESSION},
     {"PXR24_COMPRESSION", PXR24_COMPRESSION},
     {"B44_COMPRESSION", B44_COMPRESSION},
     {"B44A_COMPRESSION", B44A_COMPRESSION},
     {"DWAA_COMPRESSION", DWAA_COMPRESSION},
     {"DWAB_COMPRESSION", DWAA_COMPRESSION},
     {"HTJ2K_COMPRESSION", HTJ2K_COMPRESSION}}};

template <class T>
double
mean (const T& v)
{
    double r = 0;

    for (auto i = v.begin (); i != v.end (); i++)
    {
        r += *i;
    }

    return r / static_cast<double> (v.size ());
}

template <class T>
double
stddev (const T& v, double mean)
{
    double r = 0;

    for (auto i = v.begin (); i != v.end (); i++)
    {
        r += pow (*i - mean, 2);
    }

    return sqrt (r / static_cast<double> (v.size ()));
}

class OMemStream : public OStream
{
public:
    OMemStream (size_t initial_sz = 1024) : OStream ("<omemfile>")
    {
        this->sz        = initial_sz;
        this->buf       = (uint8_t*) malloc (this->sz);
        this->cur_ptr   = this->buf;
        this->used_size = 0;
    }

    virtual ~OMemStream () { free (this->buf); }

    virtual void write (const char c[/*n*/], int n)
    {
        size_t cur_sz = this->cur_ptr - this->buf;
        size_t req_sz = cur_sz + n;
        if (cur_sz + n > this->sz)
        {
            size_t new_sz = req_sz + req_sz / 4;
            this->buf     = (uint8_t*) realloc (this->buf, new_sz);
            this->sz      = new_sz;
            this->cur_ptr = this->buf + cur_sz;
            std::cout << "growing" << std::endl;
        }
        memcpy (this->cur_ptr, c, n);
        this->cur_ptr += n;
        this->used_size =
            std::max (this->used_size, (size_t) (this->cur_ptr - this->buf));
    }

    virtual uint64_t tellp () { return this->cur_ptr - this->buf; }

    virtual void seekp (uint64_t pos) { this->cur_ptr = this->buf + pos; }

    uint8_t* buffer () const { return this->buf; }

    size_t buffer_size () const { return used_size; }

private:
    uint8_t* buf;
    size_t   sz;
    size_t   used_size;
    uint8_t* cur_ptr;
};

class IMemStream : public IStream
{
public:
    IMemStream (uint8_t* buf, size_t sz) : IStream ("<imemfile>")
    {
        this->buf     = buf;
        this->end_ptr = this->buf + sz;
        this->cur_ptr = this->buf;
    }

    virtual ~IMemStream () {}

    virtual bool read (char c[/*n*/], int n)
    {
        if (this->cur_ptr + n > this->end_ptr)
            throw IEX_NAMESPACE::InputExc (
                "Attempt to read past the end of the file.");

        memcpy (c, this->cur_ptr, n);
        this->cur_ptr += n;

        return this->cur_ptr != this->end_ptr;
    }

    virtual uint64_t tellg () { return this->cur_ptr - this->buf; }

    virtual void seekg (uint64_t pos) { this->cur_ptr = this->buf + pos; }

    virtual void clear () {}

private:
    uint8_t* buf;
    uint8_t* cur_ptr;
    uint8_t* end_ptr;
};

struct PartBuffer
{
    std::vector<std::vector<char>> pixels;
    FrameBuffer                    fb;
};

class WritingTask : public Task
{
public:
    WritingTask (
        TaskGroup*           group,
        MultiPartOutputFile* file,
        Header*              header,
        int                  part_num,
        PartBuffer*          buffer)
        : Task (group)
        , part (*file, part_num)
        , buffer (buffer)
        , header (header)
        , part_num (part_num)
    {}

    void execute ()
    {
        part.setFrameBuffer (buffer->fb);
        Box2i dw = header->dataWindow ();
        part.writePixels (dw.max.y - dw.min.y + 1);
    }

private:
    OutputPart  part;
    PartBuffer* buffer;
    Header*     header;
    int         part_num;
};

class ReadingTask : public Task
{
public:
    ReadingTask (
        TaskGroup*          group,
        MultiPartInputFile* file,
        int                 part_num,
        PartBuffer*         buffer)
        : Task (group)
        , part (*file, part_num)
        , part_num (part_num)
        , buffer (buffer)
    {}

    void execute ()
    {
        Box2i dw = part.header ().dataWindow ();
        part.readPixels (buffer->fb, dw.min.y, dw.max.y);
    }

private:
    InputPart   part;
    PartBuffer* buffer;
    int         part_num;
};

int
main (int argc, char* argv[])
{
    cxxopts::Options options (
        "exrperf", "OpenEXR compress/uncompress benchmarks");

    options.add_options () (
        "r,repetitions",
        "Repetition count",
        cxxopts::value<int> ()->default_value ("5")) (
        "t,threads",
        "Total number of threads",
        cxxopts::value<int> ()->default_value ("0")) (
        "p,parts_threads",
        "Number of threads to handle parts",
        cxxopts::value<int> ()->default_value ("0")) (
        "h,file_threads",
        "Number of threads to per file",
        cxxopts::value<int> ()->default_value ("0")) (
        "v,verbose",
        "Output more information",
        cxxopts::value<bool> ()->default_value ("false")) (
        "l",
        "Line by line read",
        cxxopts::value<bool> ()->default_value ("false")) (
        "file", "Input image", cxxopts::value<std::string> ()) (
        "compression", "Compression", cxxopts::value<std::string> ());

    options.parse_positional ({"file", "compression"});

    options.show_positional_help ();

    auto args = options.parse (argc, argv);

    if (!supportsThreads ())
    {
        std::cout << "Threading not supported!" << std::endl;
        exit (-1);
    }

    if (args.count ("compression") != 1 || args.count ("file") != 1)
    {
        std::cout << options.help () << std::endl;
        exit (-1);
    }

    Compression c = comp_table[args["compression"].as<std::string> ()];

    auto& src_fn = args["file"].as<std::string> ();

    /* verbose mode */

    bool is_verbose = args["verbose"].as<bool> ();

    /* thread count */

    int global_thread_count = args["threads"].as<int> ();
    int parts_thread_count  = args["parts_threads"].as<int> ();
    int file_thread_count   = args["file_threads"].as<int> ();

    if (file_thread_count == 0) file_thread_count = global_thread_count;

    setGlobalThreadCount (global_thread_count);

    if (is_verbose)
    {
        std::cout << "Global thread count: " << global_thread_count
                  << std::endl;
        std::cout << "Simultaneous parts processing: "
                  << std::max (1, parts_thread_count) << std::endl;
        std::cout << "Thread count per part: " << file_thread_count
                  << std::endl;
    }

    /* load src image */

    MultiPartInputFile src_file (src_fn.c_str ());

    size_t decoded_size = 0;

    std::vector<Header>     headers (src_file.parts ());
    std::vector<PartBuffer> src_buffers (src_file.parts ());
    std::vector<PartBuffer> decoded_buffers (src_file.parts ());

    for (int part_index = 0; part_index < src_file.parts (); part_index++)
    {
        InputPart src_part (src_file, part_index);

        headers[part_index] = src_part.header ();
        /* set the target compressor */
        headers[part_index].compression () = c;

        Box2i dw     = src_part.header ().dataWindow ();
        int   width  = dw.max.x - dw.min.x + 1;
        int   height = dw.max.y - dw.min.y + 1;

        /* calculate the number of channels in the part */
        int num_chans = 0;
        for (ChannelList::ConstIterator j =
                 src_part.header ().channels ().begin ();
             j != src_part.header ().channels ().end ();
             ++j, num_chans++)
        {}

        src_buffers[part_index].pixels.resize (num_chans);
        decoded_buffers[part_index].pixels.resize (num_chans);

        int ch_index = 0;
        for (ChannelList::ConstIterator ch_iter =
                 src_part.header ().channels ().begin ();
             ch_iter != src_part.header ().channels ().end ();
             ++ch_iter, ++ch_index)
        {
            int sample_size = pixelTypeSize (ch_iter.channel ().type);
            decoded_size += (width / ch_iter.channel ().xSampling) *
                            (height / ch_iter.channel ().ySampling) *
                            sample_size;

            /* initialize the source pixel buffers */
            src_buffers[part_index].pixels[ch_index].resize (
                height * width * sample_size);
            src_buffers[part_index].fb.insert (
                ch_iter.name (),
                Slice (
                    ch_iter.channel ().type,
                    src_buffers[part_index].pixels[ch_index].data () -
                        (dw.min.y * width + dw.min.x) * sample_size,
                    sample_size,
                    sample_size * width,
                    ch_iter.channel ().xSampling,
                    ch_iter.channel ().ySampling));

            /* read the source pixels */
            src_part.readPixels (
                src_buffers[part_index].fb, dw.min.y, dw.max.y);

            /* initialize the decoded pixel buffers */
            decoded_buffers[part_index].pixels[ch_index].resize (
                height * width * sample_size);
            decoded_buffers[part_index].fb.insert (
                ch_iter.name (),
                Slice (
                    ch_iter.channel ().type,
                    decoded_buffers[part_index].pixels[ch_index].data () -
                        (dw.min.y * width + dw.min.x) * sample_size,
                    sample_size,
                    sample_size * width,
                    ch_iter.channel ().xSampling,
                    ch_iter.channel ().ySampling));
        }
    }

    /* encode performance */

    OMemStream o_memfile (decoded_size);

    std::vector<double> encode_times;

    int encoded_size;

    for (int i = 0; i < args["repetitions"].as<int> (); i++)
    {
        o_memfile.seekp (0);

        MultiPartOutputFile o_file (
            o_memfile,
            headers.data (),
            headers.size (),
            false,
            file_thread_count);

        /*MultiPartOutputFile o_file (
            "/var/tmp/out.exr", headers.data (), headers.size ());*/

        auto start = std::chrono::high_resolution_clock::now ();

        if (parts_thread_count == 0)
        {
            for (int part_index = 0; part_index < src_file.parts ();
                 part_index++)
            {
                OutputPart part (o_file, part_index);
                part.setFrameBuffer (src_buffers[part_index].fb);
                Box2i dw = headers[part_index].dataWindow ();
                part.writePixels (dw.max.y - dw.min.y + 1);
            }
        }
        else
        {
            ThreadPool* thread_pool = new ThreadPool (parts_thread_count);
            TaskGroup   task_group;
            for (int part_index = 0; part_index < src_file.parts ();
                 part_index++)
            {
                thread_pool->addTask (new WritingTask (
                    &task_group,
                    &o_file,
                    &(headers[part_index]),
                    part_index,
                    &(src_buffers[part_index])));
            }
            delete thread_pool;
        }

        auto dur = std::chrono::high_resolution_clock::now () - start;
        encode_times.push_back (std::chrono::duration<double> (dur).count ());

        if (is_verbose)
            std::cout << "Encode time :"
                      << std::chrono::duration<double> (dur).count ()
                      << std::endl;

        if (i == 0) { encoded_size = o_memfile.tellp (); }
    }

    /* decode performance */

    IMemStream i_memfile (o_memfile.buffer (), o_memfile.buffer_size ());

    std::vector<double> decode_times;

    for (int i = 0; i < args["repetitions"].as<int> (); i++)
    {
        i_memfile.seekg (0);

        MultiPartInputFile i_file (i_memfile, file_thread_count);

        ThreadPool* thread_pool = new ThreadPool (parts_thread_count);
        TaskGroup   task_group;

        auto start = std::chrono::high_resolution_clock::now ();
        if (parts_thread_count == 0)
        {
            for (int part_index = 0; part_index < i_file.parts (); part_index++)
            {
                InputPart part (i_file, part_index);
                Box2i     dw = headers[part_index].dataWindow ();
                part.readPixels (
                    decoded_buffers[part_index].fb, dw.min.y, dw.max.y);
            }
        }
        else
        {
            for (int part_index = 0; part_index < i_file.parts (); part_index++)
            {
                thread_pool->addTask (new ReadingTask (
                    &task_group,
                    &i_file,
                    part_index,
                    &(decoded_buffers[part_index])));
            }
            delete thread_pool;
        }
        auto dur = std::chrono::high_resolution_clock::now () - start;

        if (is_verbose)
            std::cout << "Decode time :"
                      << std::chrono::duration<double> (dur).count ()
                      << std::endl;

        decode_times.push_back (std::chrono::duration<double> (dur).count ());

        /* compare pixels */

        for (int part_index = 0; part_index < i_file.parts (); part_index++)
        {
            if (!std::equal (
                    src_buffers[part_index].pixels.begin (),
                    src_buffers[part_index].pixels.end (),
                    decoded_buffers[part_index].pixels.begin ()))
            {
                std::cerr << "Not lossless at part " << part_index << std::endl;
                exit (-1);
            }
        }
    }

    double encode_time_mean = mean (encode_times);
    double encode_time_dev  = stddev (encode_times, encode_time_mean);

    double decode_time_mean = mean (decode_times);
    double decode_time_dev  = stddev (decode_times, decode_time_mean);

    if (is_verbose)
        std::cout
            << "fn, c, n, decoded size, encoded size, encode time mean, encode time stddev, decode time mean, decode time stddev, global threads, per-part threads, parts worker threads"
            << std::endl;

    std::string fn = src_fn.substr (src_fn.find_last_of ("/\\") + 1);

    std::cout << fn << ", " << args["compression"].as<std::string> () << ", "
              << args["repetitions"].as<int> () << ", " << decoded_size << ", "
              << encoded_size << ", " << encode_time_mean << ", "
              << encode_time_dev << ", " << decode_time_mean << ", "
              << decode_time_dev << ", " << global_thread_count << ", "
              << file_thread_count << ", " << parts_thread_count << std::endl;

    return 0;
}