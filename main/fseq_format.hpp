#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace pov {
struct FseqHeader {
    uint16_t dataOffset = 0;
    uint8_t major = 0, minor = 0, compression = 0, stepMs = 0;
    uint32_t channels = 0, frames = 0;
    uint64_t uniqueId = 0;
};
struct SparseRange { uint32_t start, count, offset; };
struct CompressionBlock { uint32_t firstFrame, length; uint64_t fileOffset; };
inline uint32_t little(const uint8_t* p, unsigned bytes) {
    uint32_t result = 0;
    for (unsigned i = 0; i < bytes; ++i) result |= uint32_t(p[i]) << (8 * i);
    return result;
}

// FPP's FSEQ format: compression table entries contain FIRST FRAME, not
// uncompressed size. A block may contain multiple frames.
inline bool readFseqMetadata(FILE* file, FseqHeader& h, std::vector<SparseRange>& ranges,
                             std::vector<CompressionBlock>& blocks, std::string& error) {
    auto fail = [&](const char* message) { error = message; return false; };
    h = {}; ranges.clear(); blocks.clear();
    uint8_t data[32] = {};
    if (fseek(file, 0, SEEK_END) != 0) return fail("Cannot seek FSEQ");
    const long fileSize = ftell(file);
    rewind(file);
    if (fileSize < 28 || fread(data, 1, 28, file) != 28) return fail("Truncated FSEQ header");
    if (memcmp(data, "PSEQ", 4) && memcmp(data, "FSEQ", 4)) return fail("Not an FSEQ file");
    h.major = data[7]; h.minor = data[6];
    if (h.major != 1 && h.major != 2) return fail("Unsupported FSEQ version");
    if (h.major == 2 && fread(data + 28, 1, 4, file) != 4) return fail("Truncated FSEQ v2 header");
    h.dataOffset = static_cast<uint16_t>(little(data + 4, 2));
    const unsigned fixedHeader = little(data + 8, 2);
    h.channels = little(data + 10, 4);
    h.frames = little(data + 14, 4);
    h.stepMs = data[18];
    if (!h.channels || h.channels > 4 * 1024 * 1024 || !h.frames) return fail("Invalid or oversized FSEQ dimensions");
    unsigned blockCount = 0, sparseCount = 0;
    if (h.major == 2) {
        h.compression = data[20] & 15;
        blockCount = data[21] | ((data[20] & 0xf0) << 4);
        sparseCount = data[22];
        h.uniqueId = little(data + 24, 4) | (uint64_t(little(data + 28, 4)) << 32);
    }
    if (h.compression != 0 && h.compression != 2) return fail("Zstd FSEQ unsupported; export uncompressed or zlib");
    const unsigned tableEnd = (h.major == 2 ? 32 : 28) + blockCount * 8 + sparseCount * 6;
    if (fixedHeader < tableEnd || h.dataOffset < fixedHeader || h.dataOffset > fileSize)
        return fail("FSEQ tables overlap channel data or exceed file");
    uint64_t offset = h.dataOffset;
    for (unsigned i = 0; i < blockCount; ++i) {
        if (fread(data, 1, 8, file) != 8) return fail("Truncated compression table");
        const uint32_t first = little(data, 4), length = little(data + 4, 4);
        if (!length) continue; // FPP can pad the block index with empty entries.
        if (!h.compression || first >= h.frames || (blocks.empty() ? first != 0 : first <= blocks.back().firstFrame))
            return fail("Invalid compression frame index");
        if (offset + length > uint64_t(fileSize)) return fail("Truncated compressed block");
        blocks.push_back({first, length, offset});
        offset += length;
    }
    if (h.compression && blocks.empty()) return fail("Missing compression index");
    uint32_t accumulated = 0;
    for (unsigned i = 0; i < sparseCount; ++i) {
        if (fread(data, 1, 6, file) != 6) return fail("Truncated sparse table");
        const uint32_t start = little(data, 3), count = little(data + 3, 3);
        if (!count || uint64_t(start) + count > 0x1000000 || uint64_t(accumulated) + count > h.channels)
            return fail("Invalid sparse channel range");
        for (const auto& range : ranges)
            if (start < range.start + range.count && range.start < start + count) return fail("Overlapping sparse ranges");
        ranges.push_back({start, count, accumulated});
        accumulated += count;
    }
    if (sparseCount && accumulated != h.channels) return fail("Sparse ranges do not match channel count");
    if (!h.compression && uint64_t(h.dataOffset) + uint64_t(h.channels) * h.frames > uint64_t(fileSize))
        return fail("Truncated FSEQ frame data");
    return true;
}

inline int64_t sparseOffset(uint64_t channel, uint32_t channels, const std::vector<SparseRange>& ranges) {
    if (ranges.empty()) return channel < channels ? static_cast<int64_t>(channel) : -1;
    for (const auto& r : ranges)
        if (channel >= r.start && channel < uint64_t(r.start) + r.count) return r.offset + channel - r.start;
    return -1;
}
}
