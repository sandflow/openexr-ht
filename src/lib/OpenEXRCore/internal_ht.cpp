/*
** SPDX-License-Identifier: BSD-3-Clause
** Copyright Contributors to the OpenEXR Project.
*/

#include "internal_compress.h"
#include "internal_decompress.h"

#include "internal_coding.h"
#include "internal_structs.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <ojph_arch.h>
#include <ojph_file.h>
#include <ojph_params.h>
#include <ojph_mem.h>
#include <ojph_codestream.h>

#include "openexr_compression.h"

int
make_channel_map (
    int channel_count, exr_coding_channel_info_t* channels, int cs_to_file_ch[])
{
    int r_index = -1;
    int g_index = -1;
    int b_index = -1;

    for (size_t i = 0; i < channel_count; i++)
    {
        assert (channels[i].data_type == EXR_PIXEL_HALF);
        assert (channels[i].x_samples == 1);
        assert (channels[i].y_samples == 1);

        char c_name = channels[i].channel_name[0];

        if (c_name == 'R') { r_index = i; }
        else if (c_name == 'G') { g_index = i; }
        else if (c_name == 'B') { b_index = i; }
    }

    int isRGB;

    if (r_index >= 0 && g_index >= 0 && b_index >= 0)
    {
        isRGB = 1;

        cs_to_file_ch[0] = r_index;
        cs_to_file_ch[1] = g_index;
        cs_to_file_ch[2] = b_index;

        int cs_i = 3;
        for (size_t i = 0; i < channel_count; i++)
        {
            if (i != r_index && i != g_index && i != b_index)
            {
                cs_to_file_ch[cs_i++] = i;
            }
        }
    }
    else
    {
        isRGB = 0;

        for (size_t i = 0; i < channel_count; i++)
        {
            cs_to_file_ch[i] = i;
        }
    }

    return isRGB;
}

extern "C" exr_result_t
internal_exr_undo_ht (
    exr_decode_pipeline_t* decode,
    const void*            compressed_data,
    uint64_t               comp_buf_size,
    void*                  uncompressed_data,
    uint64_t               uncompressed_size)
{
    exr_result_t rv = EXR_ERR_SUCCESS;

    assert (decode->channel_count <= 6);
    int cs_to_file_ch[6];
    int isRGB = make_channel_map (
        decode->channel_count, decode->channels, cs_to_file_ch);

    ojph::mem_infile infile;
    infile.open (
        reinterpret_cast<const ojph::ui8*> (compressed_data), comp_buf_size);

    ojph::codestream cs;
    cs.read_headers (&infile);

    ojph::param_siz siz = cs.access_siz ();
    ojph::ui32 width    = siz.get_image_extent ().x - siz.get_image_offset ().x;
    ojph::ui32 height   = siz.get_image_extent ().y - siz.get_image_offset ().y;

    assert (decode->chunk.width == width);
    assert (decode->chunk.height == height);
    assert (decode->channel_count == siz.get_num_components ());

    assert (decode->channel_count * 2 * width * height == uncompressed_size);

    cs.set_planar (false);

    cs.create ();

    assert (sizeof (int16_t) == 2);
    int16_t* line_pixels = static_cast<int16_t*> (uncompressed_data);

    for (uint32_t i = 0; i < height; ++i)
    {

        for (uint32_t c = 0; c < decode->channel_count; c++)
        {
            ojph::ui32      next_comp = 0;
            ojph::line_buf* cur_line  = cs.pull (next_comp);

            assert (next_comp == c);

            int16_t* channel_pixels = line_pixels + width * cs_to_file_ch[c];

            for (uint32_t p = 0; p < width; p++)
            {
                ojph::si32 c = cur_line->i32[p];

                *channel_pixels++ = (int16_t) (c < 0 ? -32769 - c : c);
            }
        }

        line_pixels += width * decode->channel_count;
    }

    infile.close ();

    return rv;
}

extern "C"
exr_result_t
internal_exr_apply_ht (exr_encode_pipeline_t* encode)
{
    exr_result_t rv = EXR_ERR_SUCCESS;

    assert (encode->channel_count <= 6);
    int cs_to_file_ch[6];
    int isRGB = make_channel_map (
        encode->channel_count, encode->channels, cs_to_file_ch);

    int height = encode->channels[0].height;
    int width  = encode->channels[0].width;

    ojph::codestream cs;
    cs.set_planar (false);

    ojph::param_siz siz = cs.access_siz ();

    siz.set_num_components (encode->channel_count);
    for (ojph::ui32 c = 0; c < encode->channel_count; c++)
        siz.set_component (c, ojph::point (1, 1), 16, true);

    /* TODO: extend to multiple tiles and non-coincident data and display windows */
    siz.set_image_offset (ojph::point (0, 0));
    siz.set_tile_offset (ojph::point (0, 0));
    siz.set_image_extent (ojph::point (width, height));
    siz.set_tile_size (ojph::size (width, height));

    ojph::param_cod cod = cs.access_cod ();

    cod.set_color_transform (isRGB);
    cod.set_reversible (true);
    cod.set_block_dims (128, 32);
    cod.set_num_decomposition (5);

    ojph::mem_outfile output;

    output.open ();

    cs.write_headers (&output);

    assert (
        encode->packed_bytes == (encode->channel_count * 2 * height * width));

    const int16_t* line_pixels =
        static_cast<const int16_t*> (encode->packed_buffer);
    ojph::ui32      next_comp = 0;
    ojph::line_buf* cur_line  = cs.exchange (NULL, next_comp);

    for (ojph::ui32 i = 0; i < height; ++i)
    {
        const char* in_buf;

        for (ojph::ui32 c = 0; c < encode->channel_count; c++)
        {
            assert (next_comp == c);

            const int16_t* channel_pixels =
                line_pixels + width * cs_to_file_ch[c];

            for (uint32_t p = 0; p < width; p++)
            {
                ojph::si32 c =
                    static_cast<const ojph::si32> (*channel_pixels++);

                cur_line->i32[p] = c < 0 ? -32769 - c : c;
            }

            cur_line = cs.exchange (cur_line, next_comp);
        }

        line_pixels += width * encode->channel_count;
    }

    cs.flush ();

    assert (output.tell () >= 0);

    int compressed_sz = static_cast<size_t> (output.tell ());

    if (compressed_sz < encode->packed_bytes)
    {
        memcpy (encode->compressed_buffer, output.get_data (), compressed_sz);
        encode->compressed_bytes = compressed_sz;
    }
    else { encode->compressed_bytes = encode->packed_bytes; }

    return rv;
}
