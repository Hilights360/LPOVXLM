#pragma once
#include <memory>
#include "fseq_format.hpp"

namespace pov {
class Fseq {
    struct Source;
public:
    // Only the playback worker runs a job. publish() queues the latest due
    // frame; only the display task presents it at a complete sweep boundary.
    class ReadJob {
        friend class Fseq;
        std::shared_ptr<Source> source_;
        uint32_t frame_ = 0;
        bool complete_ = false;
        bool ioFailure_ = false;
    public:
        explicit operator bool() const { return bool(source_); }
        bool run(std::string& error);
        // True once the frame is decoded and waiting for its publication
        // deadline; a job that exists but is not complete still has to run.
        bool complete() const { return complete_; }
        bool ioFailure() const { return ioFailure_; }
        uint32_t frame() const { return frame_; }
    };
    ~Fseq();
    bool open(const std::string& relative, const std::string& native, std::string& error);
    bool openIoFailure() const { return openIoFailure_; }
    void close();
    // Nonblocking: false means a cancelled SD read is still finishing. Never
    // unmount or start another SD operation until this returns true.
    bool quiescent();
    bool loading() const;
    ReadJob prepare(uint32_t frame) const;
    bool current(const ReadJob& job) const;
    bool publish(ReadJob& job);
    bool present();
    void discardPending() { pendingReady_ = false; }
    bool pending() const { return pendingReady_; }
    uint32_t displayedFrame() const { return displayedFrame_; }
    uint8_t channel(uint64_t absolute) const;
    // A whole RGB row can be read without repeating sparse lookup per color.
    // The pointer is valid until the next present()/close(), under stateMutex.
    const uint8_t* channelSpan(uint64_t absolute, unsigned count) const;
    uint64_t logicalChannels() const;
    FseqHeader header;
    std::vector<SparseRange> ranges;
    std::vector<CompressionBlock> blocks;
    std::string path;
private:
    bool openIoFailure_ = false;
    std::shared_ptr<Source> source_;
    std::vector<std::shared_ptr<Source>> retired_;
    uint8_t* frame_ = nullptr;
    uint8_t* pending_ = nullptr;
    bool pendingReady_ = false;
    uint32_t pendingFrame_ = 0, displayedFrame_ = 0;
};
extern Fseq sequence;
}
