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
#include "internal_ht_common.h"

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
    bool                               isRGB = make_channel_map (
        decode->channel_count, decode->channels, cs_to_file_ch);

    ojph::mem_infile infile;
    infile.open (
        reinterpret_cast<const ojph::ui8*> (compressed_data), comp_buf_size);

    ojph::codestream cs;
    cs.read_headers (&infile);

    ojph::param_siz siz = cs.access_siz ();
    ojph::param_nlt nlt = cs.access_nlt ();

    ojph::ui32 width  = siz.get_image_extent ().x - siz.get_image_offset ().x;
    ojph::ui32 height = siz.get_image_extent ().y - siz.get_image_offset ().y;

    int bytes_per_line = 0;
    for (ojph::ui32 c = 0; c < decode->channel_count; c++)
    {
        int file_i = cs_to_file_ch[c].file_index;
        bytes_per_line +=
            decode->channels[file_i].bytes_per_element * decode->channels[file_i].width;
    }

    assert (decode->chunk.width == width);
    assert (decode->chunk.height == height);
    assert (decode->channel_count == siz.get_num_components ());
    assert (bytes_per_line * height == uncompressed_size);

    cs.set_planar (false);

    cs.create ();

    assert (sizeof (uint16_t) == 2);
    assert (sizeof (uint32_t) == 4);

    uint8_t* line_pixels = static_cast<uint8_t*> (uncompressed_data);
    for (uint32_t i = 0; i < height; ++i)
    {
        for (uint32_t c = 0; c < decode->channel_count; c++)
        {
            int file_i = cs_to_file_ch[c].file_index;
            ojph::ui32      next_comp = 0;
            ojph::line_buf* cur_line  = cs.pull (next_comp);
            assert (next_comp == c);
            if (decode->channels[file_i].data_type == EXR_PIXEL_HALF)
            {
                int16_t* channel_pixels =
                    (int16_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                for (uint32_t p = 0; p < decode->channels[file_i].width; p++) {
                    *channel_pixels++ = (int16_t) (cur_line->i32[p]);
                    assert(*(channel_pixels - 1) == 0);
                }
            }
            else if (decode->channels[file_i].data_type == EXR_PIXEL_FLOAT)
            {
                int32_t* channel_pixels =
                    (int32_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                for (uint32_t p = 0; p < decode->channels[file_i].width; p++) {
                    *channel_pixels++ = (int32_t) (cur_line->i32[p]);
                    assert(*(channel_pixels - 1) == 0);
                }
            }
            else
            {
                uint32_t* channel_pixels =
                    (uint32_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                for (uint32_t p = 0; p < decode->channels[file_i].width; p++) {
                    *channel_pixels++ = (uint32_t) (cur_line->i32[p]);
                    assert(*(channel_pixels - 1) == 0);
                }
            }
        }
        line_pixels += bytes_per_line;
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

    int height = encode->channels[0].height;
    int width  = encode->channels[0].width;

    ojph::codestream cs;
    cs.set_planar (false);

    ojph::param_siz siz = cs.access_siz ();
    ojph::param_nlt nlt = cs.access_nlt ();

    int bytes_per_line = 0;
    siz.set_num_components (encode->channel_count);
    for (ojph::ui32 c = 0; c < encode->channel_count; c++)
    {
        int file_i = cs_to_file_ch[c].file_index;
        siz.set_component (
            c,
            ojph::point (
                encode->channels[file_i].x_samples, encode->channels[file_i].y_samples),
            encode->channels[file_i].data_type == EXR_PIXEL_HALF ? 16 : 32,
            encode->channels[file_i].data_type != EXR_PIXEL_UINT);
        bytes_per_line +=
            encode->channels[file_i].bytes_per_element * encode->channels[file_i].width;
        nlt.set_type3_transformation (
            c, encode->channels[file_i].data_type != EXR_PIXEL_UINT);
    }

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

    assert (encode->packed_bytes == (bytes_per_line * height));

    ojph::ui32      next_comp = 0;
    ojph::line_buf* cur_line  = cs.exchange (NULL, next_comp);

    const uint8_t* line_pixels =
        static_cast<const uint8_t*> (encode->packed_buffer);
    for (ojph::ui32 i = 0; i < height; ++i)
    {
        for (ojph::ui32 c = 0; c < encode->channel_count; c++)
        {
            assert (next_comp == c);
            int file_i = cs_to_file_ch[c].file_index;

            if (encode->channels[file_i].data_type == EXR_PIXEL_HALF)
            {
                int16_t* channel_pixels =
                    (int16_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                for (uint32_t p = 0; p < encode->channels[file_i].width; p++) {
                    // assert(*channel_pixels == 0);
                    cur_line->i32[p] =
                        (ojph::si32) (*channel_pixels++);
                }
            }
            else if (encode->channels[file_i].data_type == EXR_PIXEL_FLOAT)
            {
                int32_t* channel_pixels =
                    (int32_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                for (uint32_t p = 0; p < encode->channels[file_i].width; p++) {
                    // assert(*channel_pixels == 0);
                    cur_line->i32[p] =
                        (ojph::si32) (*channel_pixels++);
                }
            }
            else
            {
                uint32_t* channel_pixels =
                    (uint32_t*) (line_pixels + cs_to_file_ch[c].raster_line_offset);
                for (uint32_t p = 0; p < encode->channels[file_i].width; p++) {
                    // assert(*channel_pixels == 0);
                    cur_line->i32[p] =
                       (ojph::si32) (*channel_pixels++);
                }
            }
            cur_line = cs.exchange (cur_line, next_comp);
        }
        line_pixels += bytes_per_line;
    }

    cs.flush ();

    assert (output.tell () >= 0);

    int compressed_sz = static_cast<size_t> (output.tell ());

    if (compressed_sz < encode->packed_bytes)
    {
        memcpy (encode->compressed_buffer, output.get_data (), compressed_sz);
        encode->compressed_bytes = compressed_sz;
    }
    else
    {
        encode->compressed_bytes = encode->packed_bytes;
    }

    return rv;
}
