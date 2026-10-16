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
    if(std::any_of(inputSourceMasks.begin(), inputSourceMasks.end(), [](const auto& entry) { return entry.second.isConnected(); })) return true;
    return std::count_if(inputs.begin(), inputs.end(), [](const auto& entry) { return entry.second.isConnected(); }) > 1;
}
void ImgDetectionsFilter::buildStage1() {
    linkedInputs.clear();
    for(auto& entry : inputs)
        if(entry.second.isConnected()) linkedInputs.emplace_back(entry.second.getName(), &entry.second);
    std::sort(linkedInputs.begin(), linkedInputs.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    linkedSourceMasks.assign(linkedInputs.size(), nullptr);
    for(auto& entry : inputSourceMasks) {
        if(!entry.second.isConnected()) continue;
        const auto input = std::find_if(linkedInputs.begin(), linkedInputs.end(), [&](const auto& input) { return input.first == entry.second.getName(); });
        if(input == linkedInputs.end()) throw std::invalid_argument("ImgDetectionsFilter source mask requires a matching linked detection input");
        if(runOnHostVar == false) throw std::invalid_argument("ImgDetectionsFilter source masks require host execution");
        linkedSourceMasks[std::distance(linkedInputs.begin(), input)] = &entry.second;
    }
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
    std::vector<std::shared_ptr<ImgFrame>> sourceMasks(linkedInputs.size());
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
        bool masksReady = true;
        const auto& target = reference ? reference : messages.front()->transformation;
        for(std::size_t key = 0; key < linkedSourceMasks.size(); ++key) {
            if(!linkedSourceMasks[key]) continue;
            while(auto mask = linkedSourceMasks[key]->tryGet<ImgFrame>()) sourceMasks[key] = std::move(mask);
            if(!sourceMasks[key] || !target || !sourceMasks[key]->getTransformation().isEqualTransformation(*target)) masksReady = false;
        }
        if(!reference && inputReference.isConnected()) {
            if(!dropping) logger->warn("ImgDetectionsFilter dropping rounds until a valid reference arrives");
            dropping = true;
            continue;
        }
        dropping = false;
        if(!masksReady) continue;
        auto output = impl::filterDetectionRound(messages, config, reference, sourceMasks);
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
