#pragma once
#include <cstdlib>
constexpr unsigned MALLOC_CAP_SPIRAM = 1, MALLOC_CAP_8BIT = 2, MALLOC_CAP_INTERNAL = 4,
    MALLOC_CAP_DMA = 8;
inline void* heap_caps_malloc(size_t bytes, unsigned) { return std::malloc(bytes); }
