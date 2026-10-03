#include "fseq.hpp"
#include "sd_recovery.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cstring>
#include <mutex>
#include <thread>
#include "esp_heap_caps.h"
#include "miniz.h"

namespace pov {
Fseq sequence;
namespace {
uint8_t* allocate(size_t bytes) {
    auto* memory = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!memory) memory = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return memory;
}
// The ESP32-S3's SDMMC host cannot DMA into PSRAM, so FatFs falls back to one
// 512-byte CMD17 per sector, each with its own bounce allocation, whenever a
// read lands in external RAM. Newlib's default stream buffer is also smaller
// than one sector, which keeps FatFs off its multi-sector path entirely. Frame
// data therefore arrives through an internal DMA-capable stream buffer and is
// copied to PSRAM, the same arrangement the SD read benchmark measures.
constexpr size_t StreamBufferBytes = 8 * 1024;
constexpr unsigned SettleMs = 20; // Pause before reopening after a stalled read.
char* allocateStream(size_t& size) {
    for (size_t bytes : {StreamBufferBytes, StreamBufferBytes / 2, size_t(1024)}) {
        if (auto* memory = static_cast<char*>(heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA))) {
            size = bytes;
            return memory;
        }
    }
    size = 0;
    return nullptr;
}
bool resize(uint8_t*& memory, size_t& capacity, size_t needed) {
    if (capacity >= needed) return true;
    free(memory); memory = nullptr; capacity = 0;
    memory = allocate(needed);
    if (!memory) return false;
    capacity = needed;
    return true;
}
}

struct Fseq::Source {
    std::mutex io;
    std::atomic<bool> pending{false};
    FILE* file = nullptr;
    // A null file means the handle was dropped after an I/O error and has to be
    // reopened; only close() sets cancelled, which ends the job for good.
    bool cancelled = false;
    std::string native;
    FseqHeader header;
    std::vector<CompressionBlock> blocks;
    uint8_t* next = nullptr;
    uint8_t* compressed = nullptr;
    uint8_t* expanded = nullptr;
    char* stream = nullptr;
    size_t streamBytes = 0;
    size_t compressedCapacity = 0, expandedCapacity = 0;
    int cachedBlock = -1;
    ~Source() {
        if (file) fclose(file);
        free(stream); // The stream buffer has to outlive the FILE it serves.
        free(next); free(compressed); free(expanded);
    }
    bool tryClose() {
        std::unique_lock<std::mutex> guard(io, std::try_to_lock);
        if (!guard.owns_lock()) return false;
        if (file) fclose(file);
        file = nullptr;
        cancelled = true;
        return true;
    }
};

Fseq::~Fseq() { close(); }
void Fseq::close() {
    if (source_) {
        // No waiting for SD under the display lock. Retain a busy source until
        // the worker releases it; quiescent() closes it before SD can unmount.
        retired_.push_back(std::move(source_));
    }
    quiescent();
    free(frame_); frame_ = nullptr;
    free(pending_); pending_ = nullptr;
    pendingReady_ = false;
    pendingFrame_ = displayedFrame_ = 0;
    header = {}; blocks.clear(); ranges.clear(); path.clear();
}
bool Fseq::quiescent() {
    for (auto it = retired_.begin(); it != retired_.end();) {
        if ((*it)->tryClose()) it = retired_.erase(it);
        else ++it;
    }
    return retired_.empty();
}
bool Fseq::loading() const {
    return !retired_.empty() || (source_ && source_->pending.load());
}

bool Fseq::open(const std::string& relative, const std::string& native, std::string& error) {
    close();
    openIoFailure_ = false;
    if (!quiescent()) { error = "Previous SD read is still finishing; retry shortly"; return false; }
    source_ = std::make_shared<Source>();
    source_->native = native;
    source_->file = fopen(native.c_str(), "rb");
    if (!source_->file) { openIoFailure_ = sdIoError(errno); error = "Cannot open sequence"; close(); return false; }
    // setvbuf only takes effect before the first read, so claim the buffer here.
    // Failing to get internal RAM is slow, not fatal; newlib keeps its default.
    source_->stream = allocateStream(source_->streamBytes);
    if (source_->stream && setvbuf(source_->file, source_->stream, _IOFBF, source_->streamBytes) != 0) {
        free(source_->stream);
        source_->stream = nullptr;
        source_->streamBytes = 0;
    }
    errno = 0;
    if (!readFseqMetadata(source_->file, header, ranges, blocks, error)) {
        openIoFailure_ = sdIoError(errno) || ferror(source_->file); close(); return false;
    }
    source_->header = header; source_->blocks = blocks;
    frame_ = allocate(header.channels);
    pending_ = allocate(header.channels);
    source_->next = allocate(header.channels);
    if (!frame_ || !pending_ || !source_->next) { error = "Insufficient memory for FSEQ frames"; close(); return false; }
    auto job = prepare(0);
    if (!job.run(error) || !publish(job) || !present()) { openIoFailure_ = job.ioFailure(); close(); return false; }
    path = relative;
    return true;
}

Fseq::ReadJob Fseq::prepare(uint32_t frame) const {
    ReadJob job;
    if (source_ && frame < header.frames && !source_->pending.exchange(true)) {
        job.source_ = source_; job.frame_ = frame;
    }
    return job;
}
bool Fseq::current(const ReadJob& job) const { return source_ && source_ == job.source_; }
bool Fseq::publish(ReadJob& job) {
    if (!current(job) || !job.complete_) return false;
    // Three buffers let SD keep advancing the animation clock without
    // modifying the image that the arms are still sweeping out.
    std::swap(pending_, source_->next);
    pendingFrame_ = job.frame_;
    pendingReady_ = true;
    job.complete_ = false;
    return true;
}
bool Fseq::present() {
    if (!pendingReady_) return false;
    std::swap(frame_, pending_);
    displayedFrame_ = pendingFrame_;
    pendingReady_ = false;
    return true;
}
bool Fseq::ReadJob::run(std::string& error) {
    complete_ = false;
    ioFailure_ = false;
    auto fail = [&](const char* why) { error = why; return false; };
    if (!source_) return fail("Invalid FSEQ read job");
    auto& s = *source_;
    std::lock_guard<std::mutex> guard(s.io);
    struct Finished {
        std::atomic<bool>& pending;
        ~Finished() { pending.store(false); }
    } finished{s.pending};
    if (s.cancelled || !s.next || frame_ >= s.header.frames) return fail("FSEQ read cancelled");
    auto attempt = [&](void* destination, long offset, size_t bytes) {
        if (!s.file) return false;
        errno = 0;
        clearerr(s.file);
        if (fseek(s.file, offset, SEEK_SET) == 0 && fread(destination, 1, bytes, s.file) == bytes) return true;
        if (sdIoError(errno) || ferror(s.file)) ioFailure_ = true;
        return false;
    };
    // One retry, through a fresh handle. A card that stalls a transfer past the
    // host's 100 ms data timeout answers the next one, but FatFs latches that
    // disk error on the open file and fails every later read from it, so
    // clearerr() alone leaves the sequence dead until it is reopened. Only bus
    // errors are worth repeating; a short read at end of file cannot change.
    auto reopen = [&] {
        if (s.file) { fclose(s.file); s.file = nullptr; }
        // The card is still finishing the transfer it stalled on, and reopening
        // reads a directory sector, so give it a moment before asking again.
        std::this_thread::sleep_for(std::chrono::milliseconds(SettleMs));
        s.file = fopen(s.native.c_str(), "rb");
        if (s.file && s.stream) setvbuf(s.file, s.stream, _IOFBF, s.streamBytes);
        if (s.file) s.cachedBlock = -1; // The block buffer may hold a partial read.
        return s.file != nullptr;
    };
    auto read = [&](void* destination, long offset, size_t bytes) {
        if (s.file && attempt(destination, offset, bytes)) return true;
        // Retry through a fresh handle: FatFs latches a disk error on the open
        // file and fails every later read from it, so the sequence stays dead
        // until it is reopened. A reopen that fails is not fatal either; the
        // next frame tries again rather than losing the sequence outright.
        if (s.file && !ioFailure_) return false;
        return reopen() && attempt(destination, offset, bytes);
    };
    if (!s.header.compression) {
        const uint64_t offset = s.header.dataOffset + uint64_t(frame_) * s.header.channels;
        if (offset > LONG_MAX) return fail("FSEQ frame offset exceeds file limits");
        if (!read(s.next, static_cast<long>(offset), s.header.channels)) return fail("SD frame read failed");
    } else {
        unsigned block = 0;
        while (block + 1 < s.blocks.size() && s.blocks[block + 1].firstFrame <= frame_) ++block;
        const auto& entry = s.blocks[block];
        const uint32_t end = block + 1 < s.blocks.size() ? s.blocks[block + 1].firstFrame : s.header.frames;
        const uint64_t expandedSize = uint64_t(end - entry.firstFrame) * s.header.channels;
        if (expandedSize > 4 * 1024 * 1024 || entry.length > 4 * 1024 * 1024)
            return fail("Compressed FSEQ block exceeds 4 MB; export smaller blocks or uncompressed");
        if (s.cachedBlock != static_cast<int>(block)) {
            s.cachedBlock = -1;
            if (!resize(s.compressed, s.compressedCapacity, entry.length) ||
                !resize(s.expanded, s.expandedCapacity, expandedSize)) return fail("Insufficient PSRAM for compressed block");
            if (entry.fileOffset > LONG_MAX) return fail("FSEQ block offset exceeds file limits");
            if (!read(s.compressed, static_cast<long>(entry.fileOffset), entry.length))
                return fail("SD compressed block read failed");
            auto* decoder = static_cast<tinfl_decompressor*>(calloc(1, sizeof(tinfl_decompressor)));
            if (!decoder) return fail("Insufficient memory for zlib decoder");
            tinfl_init(decoder);
            size_t inputBytes = entry.length, outputBytes = expandedSize;
            const auto status = tinfl_decompress(decoder, s.compressed, &inputBytes, s.expanded, s.expanded, &outputBytes,
                                                TINFL_FLAG_PARSE_ZLIB_HEADER | TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
            free(decoder);
            if (status != TINFL_STATUS_DONE || outputBytes != expandedSize || inputBytes != entry.length)
                return fail("Invalid zlib FSEQ block");
            s.cachedBlock = block;
        }
        memcpy(s.next, s.expanded + size_t(frame_ - entry.firstFrame) * s.header.channels, s.header.channels);
    }
    complete_ = true;
    return true;
}

uint8_t Fseq::channel(uint64_t absolute) const {
    const int64_t offset = sparseOffset(absolute, header.channels, ranges);
    return frame_ && offset >= 0 ? frame_[offset] : 0;
}
const uint8_t* Fseq::channelSpan(uint64_t absolute, unsigned count) const {
    if (!frame_ || !count) return nullptr;
    if (ranges.empty())
        return absolute < header.channels && count <= header.channels - absolute ? frame_ + absolute : nullptr;
    for (const auto& range : ranges)
        if (absolute >= range.start && absolute - range.start < range.count &&
            count <= range.count - (absolute - range.start))
            return frame_ + range.offset + (absolute - range.start);
    return nullptr;
}
uint64_t Fseq::logicalChannels() const {
    uint64_t count = header.channels;
    for (const auto& range : ranges) count = std::max(count, uint64_t(range.start) + range.count);
    return count;
}
}
