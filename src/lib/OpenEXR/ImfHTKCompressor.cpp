//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

//-----------------------------------------------------------------------------
//
//	class HTKCompressor
//
//-----------------------------------------------------------------------------

#include "ImfHTKCompressor.h"
#include "ImfAutoArray.h"
#include "ImfChannelList.h"
#include "ImfCheckedArithmetic.h"
#include "ImfHeader.h"
#include "ImfIO.h"
#include "ImfMisc.h"
#include "ImfNamespace.h"
#include "ImfXdr.h"
#include <assert.h>
#include <string.h>

#include <ImathBox.h>

#include <algorithm>

#include "kdu_compressed.h"
#include "kdu_file_io.h"
#include "kdu_messaging.h"
#include "kdu_sample_processing.h"
#include "kdu_stripe_compressor.h"
#include "kdu_stripe_decompressor.h"


using IMATH_NAMESPACE::Box2i;

using namespace kdu_supp;

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_ENTER

class error_message_handler : public kdu_core::kdu_message {
 public:

  void put_text(const char* msg) {
    std::cout << msg;
  }

  virtual void flush(bool end_of_message = false) {
    if (end_of_message) {
        std::cout << std::endl;
    }
  }
};

static error_message_handler error_handler;

HTKCompressor::HTKCompressor (const Header& hdr, size_t maxScanLineSize, int numScanLines)
    : Compressor (hdr, EXR_COMPRESSION_LAST_TYPE, maxScanLineSize, numScanLines > 0 ? numScanLines : 16000), _num_comps (0), _buffer (NULL)
{
    /* generate channel map */

    const ChannelList& channels = header ().channels ();

    int r_index = -1;
    int g_index = -1;
    int b_index = -1;

    for (ChannelList::ConstIterator c = channels.begin (); c != channels.end ();
         ++c)
    {
        assert (c.channel ().type == HALF);
        assert (c.channel ().xSampling == 1);
        assert (c.channel ().ySampling == 1);

        std::string name (c.name ());

        if (name == "R") { r_index = this->_num_comps; }
        else if (name == "G")
        {
            g_index = this->_num_comps;
        }
        else if (name == "B")
        {
            b_index = this->_num_comps;
        }

        this->_num_comps++;
    }

    this->_cs_to_file_ch.resize (this->_num_comps);

    if (r_index >= 0 && g_index >= 0 && b_index >= 0)
    {
        this->_isRGB = true;

        this->_cs_to_file_ch[0] = r_index;
        this->_cs_to_file_ch[1] = g_index;
        this->_cs_to_file_ch[2] = b_index;

        int file_i = 0;
        int cs_i   = 3;
        for (ChannelList::ConstIterator c = channels.begin ();
             c != channels.end ();
             ++c)
        {
            if (file_i != r_index && file_i != g_index && file_i != b_index)
            { this->_cs_to_file_ch[cs_i++] = file_i; }

            file_i++;
        }
    }
    else
    {
        this->_isRGB = false;

        for (int i = 0; i < this->_num_comps; i++)
        {
            this->_cs_to_file_ch[i] = i;
        }
    }

    Box2i dw      = this->header ().dataWindow ();
    this->_height = std::min (dw.size ().y + 1, this->_numScanLines);
    this->_width  = dw.size ().x + 1;
    this->_buffer =
        new int16_t[this->_num_comps * this->_width * this->_height];

    this->_heights.resize(this->_num_comps);

    this->_sample_offsets.resize(this->_num_comps);
    for(int i = 0; i < this->_sample_offsets.size(); i++) {
        this->_sample_offsets[i] = this->_cs_to_file_ch[i] * this->_width;
    }

    this->_row_gaps.resize(this->_num_comps);
    std::fill(this->_row_gaps.begin(), this->_row_gaps.end(), this->_width * this->_num_comps);

    kdu_core::kdu_customize_errors(&error_handler);
}

HTKCompressor::~HTKCompressor ()
{
    delete[] this->_buffer;
}

int
HTKCompressor::numScanLines () const
{
    return this->_numScanLines;
}

Compressor::Format
HTKCompressor::format () const
{
    return Compressor::Format::NATIVE;
}

int
HTKCompressor::compress (
    const char* inPtr, int inSize, int minY, const char*& outPtr)
{
    Box2i    dw     = this->header ().dataWindow ();
    int height = std::min (dw.size ().y + 1 - minY, this->_numScanLines);
    int width  = dw.size ().x + 1;

    assert (this->_width == width);
    assert (this->_height >= height);
    assert (
        inSize == (this->_num_comps * pixelTypeSize (HALF) * height * width));

    siz_params siz;
    siz.set (Scomponents, 0, 0, this->_num_comps);
    siz.set (Sdims, 0, 0, height);
    siz.set (Sdims, 0, 1, width);
    siz.set (Nprecision, 0, 0, 16);
    siz.set (Nsigned, 0, 0, true);
    static_cast<kdu_params&> (siz).finalize ();

    kdu_codestream codestream;

    this->_output.close ();

    // kdu_simple_file_target target("/tmp/out.j2c");
    // codestream.create (&siz, &target);

    codestream.create (&siz, &this->_output);

    codestream.set_disabled_auto_comments (0xFFFFFFFF);

    kdu_params* cod = codestream.access_siz ()->access_cluster (COD_params);

    cod->set (Creversible, 0, 0, true);
    cod->set (Corder, 0, 0, Corder_RPCL);
    cod->set (Cmodes, 0, 0, Cmodes_HT);
    cod->set (Cblk, 0, 0, 32);
    cod->set (Cblk, 0, 1, 128);
    cod->set (Clevels, 0, 0, 5);
    cod->set (Cycc, 0, 0, this->_isRGB);

    kdu_params *nlt = codestream.access_siz ()->access_cluster(NLT_params);

    nlt->set(NLType, 0, 0, NLType_SMAG);

    codestream.access_siz ()->finalize_all ();

    kdu_stripe_compressor compressor;
    compressor.start (codestream);

    std::fill(this->_heights.begin(), this->_heights.end(), height);
    compressor.push_stripe((kdu_int16 *) inPtr, this->_heights.data(), this->_sample_offsets.data(), NULL, this->_row_gaps.data());

    compressor.finish();

    outPtr = reinterpret_cast<const char*> (this->_output.get_buffer().data());

    return static_cast<int> (this->_output.get_buffer().size());
}

int
HTKCompressor::uncompress (
    const char* inPtr, int inSize, int minY, const char*& outPtr)
{
    kdu_compressed_source_buffered infile((kdu_byte*) inPtr, inSize);

    kdu_codestream  cs;
    cs.create(&infile);

    kdu_dims dims;
    cs.get_dims(0, dims, false);
    int height = dims.size.y;
    int width = dims.size.x;

    assert (this->_width >= width);
    assert (this->_height >= height);
    assert (this->_num_comps == cs.get_num_components());
    assert (sizeof (int16_t) == pixelTypeSize (HALF));

    kdu_stripe_decompressor d;

    d.start(cs);

    std::fill(this->_heights.begin(), this->_heights.end(), height);
    d.pull_stripe((kdu_int16 *) this->_buffer, this->_heights.data(), this->_sample_offsets.data(), NULL, this->_row_gaps.data());

    d.finish();

    outPtr = (const char*) this->_buffer;

    return this->_num_comps * pixelTypeSize (HALF) * width * height;
}

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_EXIT
