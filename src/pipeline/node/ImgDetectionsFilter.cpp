#include "depthai/pipeline/node/ImgDetectionsFilter.hpp"

#include <algorithm>
#include <stdexcept>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/utilities/ImgDetectionsFilter/ImgDetectionsFilterImpl.hpp"

namespace dai {
namespace node {
ImgDetectionsFilter::~ImgDetectionsFilter() = default;
ImgDetectionsFilter::ImgDetectionsFilter(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ImgDetectionsFilter, ImgDetectionsFilterProperties>(std::move(props)),
      initialConfig(std::make_shared<ImgDetectionsFilterConfig>(properties.initialConfig)) {}

ImgDetectionsFilter::Properties& ImgDetectionsFilter::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}
ImgDetectionsFilter& ImgDetectionsFilter::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
    return *this;
}
bool ImgDetectionsFilter::runOnHost() const {
    if(getDevice() == nullptr || getDevice()->getPlatform() == Platform::RVC2) return true;
    if(runOnHostVar.has_value()) return *runOnHostVar;
    return std::count_if(inputs.begin(), inputs.end(), [](const auto& entry) { return entry.second.isConnected(); }) > 1;
}
void ImgDetectionsFilter::buildStage1() {
    linkedInputs.clear();
    for(auto& entry : inputs)
        if(entry.second.isConnected()) linkedInputs.emplace_back(entry.second.getName(), &entry.second);
    std::sort(linkedInputs.begin(), linkedInputs.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    if(linkedInputs.empty()) throw std::invalid_argument("ImgDetectionsFilter requires at least one linked input");
    if(!initialConfig->validate()) throw std::invalid_argument("ImgDetectionsFilter requires minimum < maximum for confidence, area, width and height ranges");
    if(linkedInputs.size() > 1 && runOnHostVar == false) throw std::invalid_argument("ImgDetectionsFilter device execution supports only one linked input");
    if(linkedInputs.size() > 1 && !inputReference.isConnected() && !initialConfig->reference)
        throw std::invalid_argument("ImgDetectionsFilter with multiple inputs requires inputReference or a config reference");
}
void ImgDetectionsFilter::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    auto config = getProperties().initialConfig;
    auto reference = config.reference;
    bool inputReferenceReceived = false, dropping = false;
    while(mainLoop()) {
        const auto beginning = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<ImgDetections>> messages;
        {
            auto blockEvent = inputBlockEvent();
            for(const auto& input : linkedInputs) {
                auto raw = input.second->get();
                if(!raw) break;
                auto message = std::dynamic_pointer_cast<ImgDetections>(raw);
                if(!message) throw std::invalid_argument("ImgDetectionsFilter inputs accept only ImgDetections");
                messages.push_back(std::move(message));
            }
        }
        if(messages.size() != linkedInputs.size()) continue;
        const auto gotInput = std::chrono::steady_clock::now();
        std::shared_ptr<ImgDetectionsFilterConfig> nextConfig;
        while(auto next = inputConfig.tryGet<ImgDetectionsFilterConfig>()) nextConfig = std::move(next);
        if(nextConfig) {
            if(!nextConfig->validate())
                logger->warn("ImgDetectionsFilter ignored invalid runtime config: each minimum must be less than its maximum");
            else {
                config = *nextConfig;
                if(!inputReferenceReceived && config.reference) reference = config.reference;
            }
        }
        while(auto frame = inputReference.tryGet<ImgFrame>()) {
            const auto& next = frame->getTransformation();
            if(!next.isValid()) {
                logger->warn("ImgDetectionsFilter ignored reference frame with invalid ImgTransformation");
                continue;
            }
            inputReferenceReceived = true;
            if(!reference || !reference->isEqualTransformation(next)) reference = next;
        }
        if(!reference && inputReference.isConnected()) {
            if(!dropping) logger->warn("ImgDetectionsFilter dropping rounds until a valid reference arrives");
            dropping = true;
            continue;
        }
        dropping = false;
        auto output = impl::filterDetectionRound(messages, config, reference);
        if(output->detections.size() > 255 && output->getSegmentationMaskWidth() > 0)
            logger->warn("ImgDetectionsFilter mask represents only output detections 0 through 254; all detections remain in the list");
        const auto processed = std::chrono::steady_clock::now();
        {
            auto blockEvent = outputBlockEvent();
            out.send(output);
        }
        logTiming(logger, beginning, gotInput, processed, std::chrono::steady_clock::now());
    }
}
}  // namespace node
}  // namespace dai
