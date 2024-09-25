//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

#ifndef INCLUDED_IMF_HTK_COMPRESSOR_H
#define INCLUDED_IMF_HTK_COMPRESSOR_H

//-----------------------------------------------------------------------------
//
//	class HTKCompressor -- uses High-Throughput JPEG 2000.
//
//-----------------------------------------------------------------------------

#include <vector>

#include "ImfNamespace.h"

#include "ImfCompressor.h"

#include "kdu_elementary.h"
#include "kdu_params.h"
#include "kdu_stripe_compressor.h"

OPENEXR_IMF_INTERNAL_NAMESPACE_HEADER_ENTER

using namespace kdu_supp;

class mem_compressed_target : public kdu_compressed_target {
 public:
  mem_compressed_target() {}

  bool close() {
    this->buf.clear();
    return true;
  }

  bool write(const kdu_byte* buf, int num_bytes) {
    std::copy(buf, buf + num_bytes, std::back_inserter(this->buf));
    return true;
  }

  void set_target_size(kdu_long num_bytes) { this->buf.reserve(num_bytes); }

  bool prefer_large_writes() const { return false; }

  std::vector<uint8_t>& get_buffer() { return this->buf; }

 private:
  std::vector<uint8_t> buf;
};

class HTKCompressor : public Compressor
{
public:
    HTKCompressor (const Header& hdr, size_t maxScanLineSize, int numScanLines = 0);

    virtual ~HTKCompressor ();

    HTKCompressor (const HTKCompressor& other) = delete;
    HTKCompressor& operator= (const HTKCompressor& other) = delete;
    HTKCompressor (HTKCompressor&& other)                 = delete;
    HTKCompressor& operator= (HTKCompressor&& other) = delete;

    virtual int numScanLines () const;

    virtual Format format () const;

    virtual int
    compress (const char* inPtr, int inSize, int minY, const char*& outPtr);

    virtual int
    uncompress (const char* inPtr, int inSize, int minY, const char*& outPtr);

private:

    uint32_t                   _width;
    uint32_t                   _height;
    std::vector<int>           _heights;
    std::vector<int>           _sample_offsets;
    std::vector<int>           _row_gaps;
    mem_compressed_target      _output;
    int                        _num_comps;
    int16_t*                   _buffer;
    int                        _numScanLines;
    std::vector<int>           _cs_to_file_ch;    /* maps from codestream channel to file channel */
    bool                       _isRGB;
};

OPENEXR_IMF_INTERNAL_NAMESPACE_HEADER_EXIT

#endif
