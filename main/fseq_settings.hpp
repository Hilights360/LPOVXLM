#pragma once
#include <algorithm>
#include <array>
#include "fseq_format.hpp"

namespace pov {
struct FseqSettings {
    uint32_t frameUs = 0;
    unsigned spokes = 0, channelsPerSpoke = 0;
    const char* timingNote = "Load a sequence to read its frame timing.";
    const char* layoutNote = "Load a sequence to infer its spokes.";
};

// FSEQ specifies milliseconds per frame, not model dimensions. Spoke inference
// assumes one complete RGB spinner image, shared by all physical arms. Require
// a contiguous channel span starting at their common start and whole RGB rows.
// Separate per-arm streams, sparse gaps and partial rows remain manual.
inline FseqSettings deriveFseqSettings(const FseqHeader& header, const std::vector<SparseRange>& ranges,
                                      unsigned pixels, unsigned arms, const std::array<uint32_t, 4>& starts) {
    FseqSettings result;
    if (!header.frames || !header.channels) return result;
    if (!header.stepMs) result.timingNote = "The file has no valid frame interval; using manual FPS.";
    else if (unsigned(header.stepMs) * 120 < 1000)
        result.timingNote = "The file exceeds 120 FPS; using manual FPS.";
    else { result.frameUs = header.stepMs * 1000U; result.timingNote = "Frame interval read from FSEQ."; }

    result.layoutNote = "Spoke inference needs matching arm starts for one shared RGB image; using manual spokes.";
    if (!pixels || !arms || arms > starts.size() || !starts[0]) return result;
    for (unsigned arm = 1; arm < arms; ++arm) if (starts[arm] != starts[0]) return result;

    uint64_t first = 0, end = header.channels, stored = 0;
    if (!ranges.empty()) {
        first = UINT64_MAX; end = 0;
        for (const auto& range : ranges) {
            first = std::min<uint64_t>(first, range.start);
            end = std::max<uint64_t>(end, uint64_t(range.start) + range.count);
            stored += range.count;
        }
        // The metadata parser already rejects overlapping ranges.
        if (stored != header.channels || end - first != stored) {
            result.layoutNote = "Sparse channel gaps make the image width ambiguous; using manual spokes.";
            return result;
        }
    }
    if (first != uint64_t(starts[0]) - 1) {
        result.layoutNote = "The image start does not match the file's channel span; using manual spokes.";
        return result;
    }
    const uint64_t row = uint64_t(pixels) * 3;
    const uint64_t spokes = header.channels / row;
    if (header.channels % row || !spokes || spokes > 65535) {
        result.layoutNote = "The channel count is not a supported number of complete RGB spokes; using manual spokes.";
        return result;
    }
    result.spokes = static_cast<unsigned>(spokes);
    result.channelsPerSpoke = static_cast<unsigned>(row);
    result.layoutNote = "Spokes inferred from one shared RGB image and the configured pixels per arm.";
    return result;
}

inline uint64_t fseqImageChannel(uint32_t start, unsigned spoke, unsigned pixel, unsigned stride) {
    return uint64_t(start) - 1 + uint64_t(spoke) * stride + uint64_t(pixel) * 3;
}
}
