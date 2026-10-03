#pragma once
#include <cstddef>
// Host concurrency tests exercise uncompressed reads. The actual ROM zlib
// decoder is exercised on the ESP32, not replaced with a pretend success.
struct tinfl_decompressor { unsigned unused; };
constexpr unsigned TINFL_FLAG_PARSE_ZLIB_HEADER = 1, TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF = 2;
constexpr int TINFL_STATUS_DONE = 0;
inline void tinfl_init(tinfl_decompressor*) {}
inline int tinfl_decompress(tinfl_decompressor*, const unsigned char*, size_t*, unsigned char*, unsigned char*, size_t*, unsigned) { return -1; }
