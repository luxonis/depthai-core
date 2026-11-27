#pragma once

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/FrameEvent.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "utilities/DmaHeapMemory.hpp"

namespace dai {
namespace gate {

class DmaFramePool : public std::enable_shared_from_this<DmaFramePool> {
   public:
    DmaFramePool() = delete;
    DmaFramePool(const DmaFramePool&) = delete;
    DmaFramePool& operator=(const DmaFramePool&) = delete;

    static std::shared_ptr<DmaFramePool> create(size_t numFrames, size_t maxFrameSize, std::string heapPath = DmaHeapMemory::kDefaultHeapPath) {
        return std::shared_ptr<DmaFramePool>(new DmaFramePool(numFrames, maxFrameSize, std::move(heapPath)));
    }

    std::shared_ptr<ImgFrame> get() {
        std::shared_ptr<ImgFrame> frame;
        if(!availableFrames.waitAndPop(frame)) {
            throw std::runtime_error("Failed to retrieve ImgFrame from pool");
        }

        auto self = shared_from_this();
        return std::shared_ptr<ImgFrame>(frame.get(), [self, frame = std::move(frame)](ImgFrame*) mutable {
            dai::gate::DmaFramePool::resetFrame(*frame);
            self->availableFrames.push(std::move(frame));
        });
    }

    std::shared_ptr<ImgFrame> tryGet(std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) {
        std::shared_ptr<ImgFrame> frame;
        if(!availableFrames.tryWaitAndPop(frame, timeout)) {
            return {};
        }

        auto self = shared_from_this();
        return std::shared_ptr<ImgFrame>(frame.get(), [self, frame = std::move(frame)](ImgFrame*) mutable {
            dai::gate::DmaFramePool::resetFrame(*frame);
            self->availableFrames.push(std::move(frame));
        });
    }

    size_t getCapacity() const {
        return frames.size();
    }

    size_t getFrameDataSize() const {
        return maxFrameSize;
    }

    size_t getNumFree() const {
        return availableFrames.getSize();
    }

   private:
    DmaFramePool(size_t numFrames, size_t maxFrameSize, std::string heapPath)
        : availableFrames(numFrames), maxFrameSize(maxFrameSize), heapPath(std::move(heapPath)) {
        if(numFrames == 0) {
            throw std::invalid_argument("DmaFramePool requires at least one frame");
        }
        if(maxFrameSize == 0) {
            throw std::invalid_argument("DmaFramePool requires a non-zero frame size");
        }

        frames.reserve(numFrames);
        for(size_t i = 0; i < numFrames; ++i) {
            auto frame = std::make_shared<ImgFrame>();
            frame->data = std::make_shared<DmaHeapMemory>(maxFrameSize, this->heapPath);
            resetFrame(*frame);
            availableFrames.push(frame);
            frames.push_back(std::move(frame));
        }
    }

    static void resetFrame(ImgFrame& frame) {
        if(frame.data != nullptr) {
            frame.data->setSize(frame.data->getMaxSize());
            auto offsetMem = std::dynamic_pointer_cast<OffsetMemory>(frame.data);
            if(offsetMem != nullptr) {
                offsetMem->setOffset(0);
            }
        }
        frame.sequenceNum = 0;
        frame.ts = {};
        frame.tsDevice = {};
        frame.fb = {};
        frame.sourceFb = {};
        frame.cam = {};
        frame.category = 0;
        frame.instanceNum = 0;
        frame.event = dai::FrameEvent::NONE;
        frame.transformation = {};
    }

    dai::LockingQueue<std::shared_ptr<ImgFrame>> availableFrames;
    std::vector<std::shared_ptr<ImgFrame>> frames;
    size_t maxFrameSize{0};
    std::string heapPath;
};

}  // namespace gate
}  // namespace dai
