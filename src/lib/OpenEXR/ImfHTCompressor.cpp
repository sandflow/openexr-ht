//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

//-----------------------------------------------------------------------------
//
//	class HTCompressor
//
//-----------------------------------------------------------------------------

#include "ImfHTCompressor.h"
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

#include <ojph_arch.h>
#include <ojph_file.h>
#include <ojph_params.h>
#include <ojph_mem.h>

using IMATH_NAMESPACE::Box2i;

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_ENTER

HTCompressor::HTCompressor (const Header& hdr, size_t maxScanLineSize, int numScanLines)
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
        else if (name ==  "B")
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

    Box2i dw     = this->header ().dataWindow ();
    this->_height = std::min (dw.size ().y + 1, this->_numScanLines);
    this->_width  = dw.size ().x + 1;
    this->_buffer = new int16_t[this->_num_comps * this->_width * this->_height];
}

HTCompressor::~HTCompressor ()
{
    delete[] this->_buffer;
}

int
HTCompressor::numScanLines () const
{
    return this->_numScanLines;
}

Compressor::Format
HTCompressor::format () const
{
    return Compressor::Format::NATIVE;
}

int
HTCompressor::compress (
    const char* inPtr, int inSize, int minY, const char*& outPtr)
{
    Box2i      dw     = this->header ().dataWindow ();
    ojph::ui32 height = std::min (dw.size ().y + 1 - minY, this->_numScanLines);
    ojph::ui32 width  = dw.size ().x + 1;

    assert(this->_width == width);
    assert(this->_height >= height);

    ojph::codestream  cs;
    cs.set_planar (false);

    ojph::param_siz siz = cs.access_siz ();

    siz.set_num_components (this->_num_comps);
    for (ojph::ui32 c = 0; c < this->_num_comps; c++)
        siz.set_component (c, ojph::point (1, 1), 16, true);

    /* TODO: extend to multiple tiles and non-coincident data and display windows */
    siz.set_image_offset (ojph::point (0, 0));
    siz.set_tile_offset (ojph::point (0, 0));
    siz.set_image_extent (ojph::point (width, height));
    siz.set_tile_size (ojph::size (width, height));

    ojph::param_cod cod = cs.access_cod ();

    cod.set_color_transform (this->_isRGB);
    cod.set_reversible (true);
    cod.set_block_dims (128, 32);
    cod.set_num_decomposition (5);

    this->_output.close ();
    this->_output.open ();

    cs.write_headers (&this->_output);

    assert (
        inSize == (this->_num_comps * pixelTypeSize (HALF) * height * width));

    const int16_t*  pixels      = (const int16_t*) inPtr;
    const int16_t*  line_pixels = pixels;
    ojph::ui32      next_comp   = 0;
    ojph::line_buf* cur_line    = cs.exchange (NULL, next_comp);

    for (ojph::ui32 i = 0; i < height; ++i)
    {
        const char* in_buf;

        for (ojph::ui32 c = 0; c < this->_num_comps; c++)
        {
            assert (next_comp == c);

            const int16_t* channel_pixels =
                line_pixels + width * this->_cs_to_file_ch[c];

            for (uint32_t p = 0; p < width; p++)
            {
                ojph::si32 c = static_cast<const ojph::si32> (*channel_pixels++);

                cur_line->i32[p] = c < 0 ? -32769 - c : c;
            }

            cur_line = cs.exchange (cur_line, next_comp);
        }

        line_pixels += width * this->_num_comps;
    }

    cs.flush ();

    assert (this->_output.tell () >= 0);

    outPtr = reinterpret_cast<const char*> (this->_output.get_data ());

    return static_cast<int> (this->_output.tell ());
}

int
HTCompressor::uncompress (
    const char* inPtr, int inSize, int minY, const char*& outPtr)
{
    ojph::mem_infile infile;
    infile.open (reinterpret_cast<const ojph::ui8*> (inPtr), inSize);

    ojph::codestream  cs;
    cs.read_headers (&infile);

    ojph::param_siz siz = cs.access_siz ();
    ojph::ui32 width    = siz.get_image_extent ().x - siz.get_image_offset ().x;
    ojph::ui32 height   = siz.get_image_extent ().y - siz.get_image_offset ().y;

    assert (this->_width >= width);
    assert (this->_height >= height);
    assert (this->_num_comps == siz.get_num_components ());

    cs.set_planar (false);

    cs.create ();

    assert (sizeof (int16_t) == pixelTypeSize (HALF));
    int16_t* line_pixels = this->_buffer;

    for (uint32_t i = 0; i < height; ++i)
    {

        for (uint32_t c = 0; c < this->_num_comps; c++)
        {
            ojph::ui32      next_comp = 0;
            ojph::line_buf* cur_line  = cs.pull (next_comp);

            assert (next_comp == c);

            int16_t* channel_pixels = line_pixels + width * this->_cs_to_file_ch[c];

            for (uint32_t p = 0; p < width; p++)
            {
                ojph::si32 c = cur_line->i32[p];

                *channel_pixels++ = (int16_t) (c < 0 ? -32769 - c : c);
            }
        }

        line_pixels += width * this->_num_comps;
    }

    infile.close();

    outPtr = (const char*) this->_buffer;

    return this->_num_comps * pixelTypeSize (HALF) * width * height;
}

OPENEXR_IMF_INTERNAL_NAMESPACE_SOURCE_EXIT
