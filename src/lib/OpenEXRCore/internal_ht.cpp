/*
** SPDX-License-Identifier: BSD-3-Clause
** Copyright Contributors to the OpenEXR Project.
*/

#include <limits>
#include <string>
#include <fstream>

#include "openexr_decode.h"
#include "openexr_encode.h"
#include "internal_ht_common.h"

#ifdef KDU_AVAILABLE

#include <iostream>

#include "openexr_compression.h"
#include "kdu_elementary.h"
#include "kdu_params.h"
#include "kdu_stripe_compressor.h"
#include "kdu_compressed.h"
#include "kdu_file_io.h"
#include "kdu_messaging.h"
#include "kdu_sample_processing.h"
#include "kdu_stripe_decompressor.h"

using namespace kdu_supp;

class mem_compressed_target : public kdu_compressed_target
{
public:
    mem_compressed_target () {}

    bool close ()
    {
        this->buf.clear ();
        return true;
    }

    bool write (const kdu_byte* buf, int num_bytes)
    {
        std::copy (buf, buf + num_bytes, std::back_inserter (this->buf));
        return true;
    }

    void set_target_size (kdu_long num_bytes) { this->buf.reserve (num_bytes); }

    bool prefer_large_writes () const { return false; }

    std::vector<uint8_t>& get_buffer () { return this->buf; }

private:
    std::vector<uint8_t> buf;
};

class error_message_handler : public kdu_core::kdu_message
{
public:
    void put_text (const char* msg) { std::cout << msg; }

    virtual void flush (bool end_of_message = false)
    {
        if (end_of_message) { std::cout << std::endl; }
    }
};

static error_message_handler error_handler;

extern "C" exr_result_t
internal_exr_undo_ht (
    exr_decode_pipeline_t* decode,
    const void*            compressed_data,
    uint64_t               comp_buf_size,
    void*                  uncompressed_data,
    uint64_t               uncompressed_size)
{
    exr_result_t rv = EXR_ERR_SUCCESS;

    std::vector<CodestreamChannelInfo> cs_to_file_ch (decode->channel_count);

    /* read the channel map */

    size_t header_sz;
    read_header (
        (uint8_t*) compressed_data, comp_buf_size, header_sz, cs_to_file_ch);
    if (decode->channel_count != cs_to_file_ch.size ())
        throw std::runtime_error ("Unexpected number of channels");

    std::vector<int> heights (decode->channel_count);
    std::vector<int> sample_offsets (decode->channel_count);

    int32_t width  = decode->chunk.width;
    int32_t height = decode->chunk.height;

    for (int i = 0; i < sample_offsets.size (); i++)
    {
        sample_offsets[i] = cs_to_file_ch[i].file_index * width;
    }

    std::vector<int> row_gaps (decode->channel_count);
    std::fill (
        row_gaps.begin (), row_gaps.end (), width * decode->channel_count);

    kdu_core::kdu_customize_errors (&error_handler);

    kdu_compressed_source_buffered infile (
       ((kdu_byte*) (compressed_data)) + header_sz, comp_buf_size - header_sz);

    kdu_codestream cs;
    cs.create (&infile);

    kdu_dims dims;
    cs.get_dims (0, dims, false);

    assert (width == dims.size.x);
    assert (height == dims.size.y);
    assert (decode->channel_count == cs.get_num_components ());
    assert (sizeof (int16_t) == 2);

    kdu_stripe_decompressor d;

    d.start (cs);

    std::fill (heights.begin (), heights.end (), height);
    d.pull_stripe (
        (kdu_int16*) uncompressed_data,
        heights.data (),
        sample_offsets.data (),
        NULL,
        row_gaps.data ());

    d.finish ();

    cs.destroy ();

    return rv;
}

extern "C" exr_result_t
internal_exr_apply_ht (exr_encode_pipeline_t* encode)
{
    exr_result_t rv = EXR_ERR_SUCCESS;

    std::vector<CodestreamChannelInfo> cs_to_file_ch (encode->channel_count);
    bool             isRGB = make_channel_map (
        encode->channel_count, encode->channels, cs_to_file_ch);

    int height = encode->chunk.height;
    int width  = encode->chunk.width;

    std::vector<int> heights (encode->channel_count);
    std::vector<int> sample_offsets (encode->channel_count);
    std::vector<int> row_gaps (encode->channel_count);
    std::fill (
        row_gaps.begin (), row_gaps.end (), width * encode->channel_count);

    siz_params siz;
    siz.set (Scomponents, 0, 0, encode->channel_count);
    siz.set (Sdims, 0, 0, height);
    siz.set (Sdims, 0, 1, width);
    siz.set (Nprecision, 0, 0, 16);
    siz.set (Nsigned, 0, 0, true);
    static_cast<kdu_params&> (siz).finalize ();

    kdu_codestream        codestream;
    // kdu_simple_file_target output("/tmp/out.j2c");
    // codestream.create (&siz, &output);

    mem_compressed_target output;
    codestream.create (&siz, &output);

    codestream.set_disabled_auto_comments (0xFFFFFFFF);

    kdu_params* cod = codestream.access_siz ()->access_cluster (COD_params);

    cod->set (Creversible, 0, 0, true);
    cod->set (Corder, 0, 0, Corder_RPCL);
    cod->set (Cmodes, 0, 0, Cmodes_HT);
    cod->set (Cblk, 0, 0, 32);
    cod->set (Cblk, 0, 1, 128);
    cod->set (Clevels, 0, 0, 5);
    cod->set (Cycc, 0, 0, isRGB);

    kdu_params* nlt = codestream.access_siz ()->access_cluster (NLT_params);

    nlt->set (NLType, 0, 0, NLType_SMAG);

    codestream.access_siz ()->finalize_all ();

    kdu_stripe_compressor compressor;
    compressor.start (codestream);

    std::fill (heights.begin (), heights.end (), height);
    compressor.push_stripe (
        (kdu_int16*) encode->packed_buffer,
        heights.data (),
        sample_offsets.data (),
        NULL,
        row_gaps.data ());

    compressor.finish ();

    codestream.destroy ();

    size_t header_sz = write_header (
        (uint8_t*) encode->compressed_buffer,
        encode->packed_bytes,
        cs_to_file_ch);

    int compressed_sz = static_cast<size_t> (output.get_buffer ().size ());

    if (compressed_sz + header_sz < encode->packed_bytes)
    {
        memcpy (
            ((uint8_t*) encode->compressed_buffer) + header_sz,
            output.get_buffer ().data (),
            compressed_sz);
        encode->compressed_bytes = compressed_sz + header_sz;
    }
    else
    {
        encode->compressed_bytes = encode->packed_bytes;
    }

    return rv;
}

#else

#include <ojph_arch.h>
#include <ojph_file.h>
#include <ojph_params.h>
#include <ojph_mem.h>
#include <ojph_codestream.h>

extern "C" exr_result_t
internal_exr_undo_ht (
    exr_decode_pipeline_t* decode,
    const void*            compressed_data,
    uint64_t               comp_buf_size,
    void*                  uncompressed_data,
    uint64_t               uncompressed_size)
{
    exr_result_t rv = EXR_ERR_SUCCESS;

    std::vector<CodestreamChannelInfo> cs_to_file_ch (decode->channel_count);

    /* read the channel map */

    size_t header_sz;
    read_header (
        (uint8_t*) compressed_data, comp_buf_size, header_sz, cs_to_file_ch);
    if (decode->channel_count != cs_to_file_ch.size ())
        throw std::runtime_error ("Unexpected number of channels");

    std::vector<size_t> offsets (decode->channel_count);
    offsets[0] = 0;
    for (int file_i = 1; file_i < decode->channel_count; file_i++)
    {
        offsets[file_i] = offsets[file_i - 1] +
                          decode->channels[file_i - 1].width *
                              decode->channels[file_i - 1].bytes_per_element;
    }
    for (int cs_i = 0; cs_i < decode->channel_count; cs_i++)
    {
        cs_to_file_ch[cs_i].raster_line_offset =
            offsets[cs_to_file_ch[cs_i].file_index];
    }

    ojph::mem_infile infile;
    infile.open (
        reinterpret_cast<const ojph::ui8*> (compressed_data) + header_sz,
        comp_buf_size - header_sz);

    ojph::codestream cs;
    cs.read_headers (&infile);

    ojph::param_siz siz = cs.access_siz ();
    ojph::param_nlt nlt = cs.access_nlt ();

    ojph::ui32 image_width =
        siz.get_image_extent ().x - siz.get_image_offset ().x;
    ojph::ui32 image_height =
        siz.get_image_extent ().y - siz.get_image_offset ().y;

    int  bpl       = 0;
    bool is_planar = false;
    for (ojph::ui32 c = 0; c < decode->channel_count; c++)
    {
        bpl +=
            decode->channels[c].bytes_per_element * decode->channels[c].width;
        if (decode->channels[c].x_samples > 1 ||
            decode->channels[c].y_samples > 1)
        { is_planar = true; }
    }
    cs.set_planar (is_planar);

    assert (decode->chunk.width == image_width);
    assert (decode->chunk.height == image_height);
    assert (decode->channel_count == siz.get_num_components ());

    cs.create ();

    assert (sizeof (uint16_t) == 2);
    assert (sizeof (uint32_t) == 4);
    ojph::ui32      next_comp = 0;
    ojph::line_buf* cur_line;
    if (cs.is_planar ())
    {
        for (uint32_t c = 0; c < decode->channel_count; c++)
        {
            int file_c = cs_to_file_ch[c].file_index;
            assert (
                siz.get_recon_height (c) == decode->channels[file_c].height);
            assert (decode->channels[file_c].width == siz.get_recon_width (c));

            if (decode->channels[file_c].height == 0) continue;

            uint8_t* line_pixels = static_cast<uint8_t*> (uncompressed_data);

            for (int64_t y = decode->chunk.start_y;
                 y < image_height + decode->chunk.start_y;
                 y++)
            {
                for (ojph::ui32 line_c = 0; line_c < decode->channel_count;
                     line_c++)
                {
                    if (y % decode->channels[line_c].y_samples != 0) continue;

                    if (line_c == file_c)
                    {
                        cur_line = cs.pull (next_comp);
                        assert (next_comp == c);

                        if (decode->channels[file_c].data_type ==
                            EXR_PIXEL_HALF)
                        {
                            int16_t* channel_pixels = (int16_t*) line_pixels;
                            for (uint32_t p = 0;
                                 p < decode->channels[file_c].width;
                                 p++)
                            {
                                *channel_pixels++ = cur_line->i32[p];
                            }
                        }
                        else
                        {
                            int32_t* channel_pixels = (int32_t*) line_pixels;
                            for (uint32_t p = 0;
                                 p < decode->channels[file_c].width;
                                 p++)
                            {
                                *channel_pixels++ = cur_line->i32[p];
                            }
                        }
                    }

                    line_pixels += decode->channels[line_c].bytes_per_element *
                                   decode->channels[line_c].width;
                }
            }
        }
    }
    else
    {
        uint8_t* line_pixels = static_cast<uint8_t*> (uncompressed_data);

        assert (bpl * image_height == uncompressed_size);

        for (uint32_t y = 0; y < image_height; ++y)
        {
            for (uint32_t c = 0; c < decode->channel_count; c++)
            {
                int file_c = cs_to_file_ch[c].file_index;
                cur_line   = cs.pull (next_comp);
                assert (next_comp == c);
                if (decode->channels[file_c].data_type == EXR_PIXEL_HALF)
                {
                    int16_t* channel_pixels =
                        (int16_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                    for (uint32_t p = 0; p < decode->channels[file_c].width;
                         p++)
                    {
                        *channel_pixels++ = cur_line->i32[p];
                    }
                }
                else
                {
                    int32_t* channel_pixels =
                        (int32_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                    for (uint32_t p = 0; p < decode->channels[file_c].width;
                         p++)
                    {
                        *channel_pixels++ = cur_line->i32[p];
                    }
                }
            }
            line_pixels += bpl;
        }
    }

    infile.close ();

    return rv;
}

extern "C" exr_result_t
internal_exr_apply_ht (exr_encode_pipeline_t* encode)
{
    exr_result_t rv = EXR_ERR_SUCCESS;

    std::vector<CodestreamChannelInfo> cs_to_file_ch (encode->channel_count);
    bool                               isRGB = make_channel_map (
        encode->channel_count, encode->channels, cs_to_file_ch);

    int image_height = encode->chunk.height;
    int image_width  = encode->chunk.width;

    ojph::codestream cs;

    ojph::param_siz siz = cs.access_siz ();
    ojph::param_nlt nlt = cs.access_nlt ();

    bool isPlanar = false;
    siz.set_num_components (encode->channel_count);
    int bpl = 0;
    for (ojph::ui32 c = 0; c < encode->channel_count; c++)
    {
        int file_c = cs_to_file_ch[c].file_index;
        if (encode->channels[file_c].data_type != EXR_PIXEL_UINT)
            nlt.set_nonlinear_transform (
                c,
                ojph::param_nlt::nonlinearity::OJPH_NLT_BINARY_COMPLEMENT_NLT);
        siz.set_component (
            c,
            ojph::point (
                encode->channels[file_c].x_samples,
                encode->channels[file_c].y_samples),
            encode->channels[file_c].data_type == EXR_PIXEL_HALF ? 16 : 32,
            encode->channels[file_c].data_type != EXR_PIXEL_UINT);

        if (encode->channels[file_c].x_samples > 1 ||
            encode->channels[file_c].y_samples > 1)
        { isPlanar = true; }

        bpl += encode->channels[file_c].bytes_per_element *
               encode->channels[file_c].width;
    }

    cs.set_planar (isPlanar);

    siz.set_image_offset (ojph::point (0, 0));
    siz.set_image_extent (ojph::point (image_width, image_height));

    ojph::param_cod cod = cs.access_cod ();

    cod.set_color_transform (isRGB && !isPlanar);
    cod.set_reversible (true);
    cod.set_block_dims (128, 32);
    cod.set_num_decomposition (5);

    ojph::mem_outfile output;

    output.open ();

    cs.write_headers (&output);

    ojph::ui32      next_comp = 0;
    ojph::line_buf* cur_line  = cs.exchange (NULL, next_comp);

    if (cs.is_planar ())
    {
        for (ojph::ui32 c = 0; c < encode->channel_count; c++)
        {
            if (encode->channels[c].height == 0) continue;

            const uint8_t* line_pixels =
                static_cast<const uint8_t*> (encode->packed_buffer);
            int file_c = cs_to_file_ch[c].file_index;

            for (int64_t y = encode->chunk.start_y;
                 y < image_height + encode->chunk.start_y;
                 y++)
            {
                for (ojph::ui32 line_c = 0; line_c < encode->channel_count;
                     line_c++)
                {

                    if (y % encode->channels[line_c].y_samples != 0) continue;

                    if (line_c == file_c)
                    {
                        if (encode->channels[file_c].data_type ==
                            EXR_PIXEL_HALF)
                        {
                            int16_t* channel_pixels = (int16_t*) (line_pixels);
                            for (uint32_t p = 0;
                                 p < encode->channels[file_c].width;
                                 p++)
                            {
                                cur_line->i32[p] = *channel_pixels++;
                            }
                        }
                        else
                        {
                            int32_t* channel_pixels = (int32_t*) (line_pixels);
                            for (uint32_t p = 0;
                                 p < encode->channels[file_c].width;
                                 p++)
                            {
                                cur_line->i32[p] = *channel_pixels++;
                            }
                        }

                        assert (next_comp == c);
                        cur_line = cs.exchange (cur_line, next_comp);
                    }

                    line_pixels += encode->channels[line_c].bytes_per_element *
                                   encode->channels[line_c].width;
                }
            }
        }
    }
    else
    {
        const uint8_t* line_pixels =
            static_cast<const uint8_t*> (encode->packed_buffer);

        assert (bpl * image_height == encode->packed_bytes);

        for (int y = 0; y < image_height; y++)
        {
            for (ojph::ui32 c = 0; c < encode->channel_count; c++)
            {
                int file_c = cs_to_file_ch[c].file_index;

                if (encode->channels[file_c].data_type == EXR_PIXEL_HALF)
                {
                    int16_t* channel_pixels =
                        (int16_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                    for (uint32_t p = 0; p < encode->channels[file_c].width;
                         p++)
                    {
                        cur_line->i32[p] = *channel_pixels++;
                    }
                }
                else
                {
                    int32_t* channel_pixels =
                        (int32_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                    for (uint32_t p = 0; p < encode->channels[file_c].width;
                         p++)
                    {
                        cur_line->i32[p] = *channel_pixels++;
                    }
                }
                assert (next_comp == c);
                cur_line = cs.exchange (cur_line, next_comp);
            }
            line_pixels += bpl;
        }
    }

    cs.flush ();

    size_t header_sz = write_header (
        (uint8_t*) encode->compressed_buffer,
        encode->packed_bytes,
        cs_to_file_ch);

    assert (output.tell () >= 0);
    int compressed_sz = static_cast<size_t> (output.tell ());
    if (compressed_sz + header_sz < encode->packed_bytes)
    {
        memcpy (
            ((uint8_t*) encode->compressed_buffer) + header_sz,
            output.get_data (),
            compressed_sz);
        encode->compressed_bytes = compressed_sz + header_sz;
    }
    else
    {
        encode->compressed_bytes = encode->packed_bytes;
    }

    return rv;
}

#endif