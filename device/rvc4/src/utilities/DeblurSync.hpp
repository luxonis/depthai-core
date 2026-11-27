#pragma once
#include <chrono>
#include <deque>
#include <memory>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"

namespace dai {
namespace gate {
namespace deblur {
inline bool sameFrame(const Buffer& a, const Buffer& b) {
    return a.getSequenceNum() == b.getSequenceNum() && a.getTimestampDevice() == b.getTimestampDevice();
}
template <typename T>
std::shared_ptr<T> member(const MessageGroup& group, const char* name) {
    const auto it = group.group.find(name);
    return it == group.group.end() ? nullptr : std::dynamic_pointer_cast<T>(it->second);
}
/** Bounded, ordered rendezvous. Call take after each insertion to enforce the pending bound. */
class Synchronizer {
   public:
    using Clock = std::chrono::steady_clock;
    struct Ready {
        std::shared_ptr<ImgFrame> target, reference;
        std::shared_ptr<MessageGroup> flow;
    };
    Synchronizer(std::size_t capacity, std::chrono::milliseconds timeout) : capacity(capacity), timeout(timeout) {}
    void addImage(std::shared_ptr<ImgFrame> image, Clock::time_point now) {
        pending.push_back({image, now});
        images.push_back(std::move(image));
        while(images.size() > capacity) images.pop_front();
    }
    void addFlow(std::shared_ptr<MessageGroup> flow) {
        const auto ref = member<Buffer>(*flow, "reference"), target = member<Buffer>(*flow, "target");
        if(!ref || !target || target->getTimestampDevice() <= ref->getTimestampDevice()) return;
        flows.push_back(std::move(flow));
        while(flows.size() > capacity) flows.pop_front();
    }
    Ready take(Clock::time_point now) {
        if(pending.empty()) return {};
        const auto target = pending.front().image;
        for(auto it = flows.begin(); it != flows.end(); ++it) {
            const auto metadata = member<Buffer>(**it, "target");
            if(!sameFrame(*metadata, *target)) continue;
            const auto ref = member<Buffer>(**it, "reference");
            for(const auto& image : images) {
                if(sameFrame(*image, *ref)) {
                    Ready result{target, image, *it};
                    flows.erase(it);
                    pending.pop_front();
                    return result;
                }
            }
            // A missing reference may still arrive within the wait window.
            break;
        }
        if(pending.size() >= capacity || now - pending.front().arrival >= timeout) {
            pending.pop_front();
            return {target, nullptr, nullptr};
        }
        return {};
    }

   private:
    struct Pending {
        std::shared_ptr<ImgFrame> image;
        Clock::time_point arrival;
    };
    std::size_t capacity;
    std::chrono::milliseconds timeout;
    std::deque<Pending> pending;
    std::deque<std::shared_ptr<ImgFrame>> images;
    std::deque<std::shared_ptr<MessageGroup>> flows;
};
}  // namespace deblur
}  // namespace gate
}  // namespace dai
