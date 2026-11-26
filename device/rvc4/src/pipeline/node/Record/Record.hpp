#pragma once

#include <memory>
#include <cstdint>
#include <vector>

#include "depthai/pipeline/node/host/Record.hpp"

namespace dai {
namespace gate {

class RecordVideo : public ::dai::node::RecordVideo {
   public:
    constexpr static const char* NAME = ::dai::node::RecordVideo::NAME;

    RecordVideo();
    explicit RecordVideo(std::unique_ptr<Properties> props);
    ~RecordVideo() override;

    bool runOnHost() const override {
        return true;
    }
    void buildStage1() override;
    void run() override;
    // The prepare-download RPC joins writers before releasing their store reservations.
    void releaseRecording();
    // Called only after wait(), so no writer can update the snapshot concurrently.
    std::vector<std::uint8_t> statusMetadata() const;

   private:
    struct State;
    std::unique_ptr<State> state;
};

}  // namespace gate
}  // namespace dai
