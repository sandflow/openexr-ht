/*
** SPDX-License-Identifier: BSD-3-Clause
** Copyright Contributors to the OpenEXR Project.
*/

#ifndef OPENEXR_PRIVATE_HT_COMMON_H
#define OPENEXR_PRIVATE_HT_COMMON_H

#include <vector>
#include <stdexcept>
#include <stdlib.h>
#include "openexr_coding.h"

struct CodestreamChannelInfo {
    int file_index;
    size_t raster_line_offset;
};

bool
make_channel_map (
    int channel_count, exr_coding_channel_info_t* channels, std::vector<CodestreamChannelInfo>& cs_to_file_ch);

/***********************************

Structure of the HTJ2K chunk
- MAGIC = 0x4854: magic number
- PLEN: length of header payload (big endian uint32_t)
- header payload
    - NCH: number of channels in channel map (big endian uint16_t)
    - for(i = 0; i < NCH; i++)
        - CS_TO_F[i]: OpenEXR channel index corresponding to J2K component index i (big endian uint16_t)
    - any number of opaque bytes
- CS: JPEG 2000 Codestream

***********************************/

class MemoryReader
{
public:
    MemoryReader (uint8_t* buffer, size_t max_sz)
        : buffer (buffer), cur (buffer), end (buffer + max_sz){};

    uint32_t pull_uint32 ();

    uint16_t pull_uint16 ();

protected:
    uint8_t* buffer;
    uint8_t* cur;
    uint8_t* end;
};

class MemoryWriter
{
public:
    MemoryWriter (uint8_t* buffer, size_t max_sz)
        : buffer (buffer), cur (buffer), end (buffer + max_sz){};

    void push_uint32 (uint32_t value);

    void push_uint16 (uint16_t value);

    size_t get_size () { return this->cur - this->buffer; }

    uint8_t* get_buffer () { return this->buffer; }

    uint8_t* get_cur () { return this->cur; }

protected:
    uint8_t* buffer;
    uint8_t* cur;
    uint8_t* end;
};

size_t
write_header (
    uint8_t*                                  buffer,
    size_t                                    max_sz,
    const std::vector<CodestreamChannelInfo>& map);

void
read_header (
    void*                               buffer,
    size_t                              max_sz,
    size_t&                             length,
    std::vector<CodestreamChannelInfo>& map);

#endif /* OPENEXR_PRIVATE_HT_COMMON_H */
