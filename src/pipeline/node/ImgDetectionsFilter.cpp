#include "depthai/pipeline/node/ImgDetectionsFilter.hpp"

#include <algorithm>
#include <stdexcept>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/utilities/ImgDetectionsFilter/ImgDetectionsFilterImpl.hpp"

namespace dai {
namespace node {
namespace {
// Inputs latched between rounds: the runtime config, the reference transformation and the source mask of each key.
struct Latched {
    ImgDetectionsFilterConfig config;
    std::optional<ImgTransformation> reference;
    std::vector<std::shared_ptr<ImgFrame>> sourceMasks;
    bool referenceReceived = false;

    // Apply the last queued valid config. Its reference applies until the first reference frame arrives.
    void updateConfig(Node::Input& input, spdlog::logger& logger) {
        std::shared_ptr<ImgDetectionsFilterConfig> next;
        while(auto queued = input.tryGet<ImgDetectionsFilterConfig>()) next = std::move(queued);
        if(!next) return;
        if(!next->validate()) {
            logger.warn("ImgDetectionsFilter ignored invalid runtime config: check ranges, IoU threshold, and pixel ROI coordinates");
            return;
        }
        config = *next;
        if(!referenceReceived && config.reference) reference = config.reference;
    }
    // Latch the newest valid reference frame. It supersedes the config reference permanently.
    void updateReference(Node::Input& input, spdlog::logger& logger) {
        while(auto frame = input.tryGet<ImgFrame>()) {
            const auto& next = frame->getTransformation();
            if(!next.isValid()) {
                logger.warn("ImgDetectionsFilter ignored reference frame with invalid ImgTransformation");
                continue;
            }
            referenceReceived = true;
            if(!reference || !reference->isEqualTransformation(next)) reference = next;
        }
    }
    // Latch the newest mask of each linked key. Return whether every linked mask matches the output transformation.
    bool updateMasks(const std::vector<Node::Input*>& inputs, const std::optional<ImgTransformation>& target) {
        bool ready = true;
        for(std::size_t key = 0; key < inputs.size(); ++key) {
            if(!inputs[key]) continue;
            while(auto mask = inputs[key]->tryGet<ImgFrame>()) sourceMasks[key] = std::move(mask);
            if(!sourceMasks[key] || !target || !sourceMasks[key]->getTransformation().isEqualTransformation(*target)) ready = false;
        }
        return ready;
    }
};
}  // namespace

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
    if(!initialConfig->validate())
        throw std::invalid_argument("ImgDetectionsFilter requires minimum < maximum ranges, a finite nonnegative IoU threshold, and a pixel ROI");
    if(linkedInputs.size() > 1 && runOnHostVar == false) throw std::invalid_argument("ImgDetectionsFilter device execution supports only one linked input");
    if(linkedInputs.size() > 1 && !inputReference.isConnected() && !initialConfig->reference)
        throw std::invalid_argument("ImgDetectionsFilter with multiple inputs requires inputReference or a config reference");
}
void ImgDetectionsFilter::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    // Read the member, not getProperties(): that writes the properties, which Pipeline::start() serializes concurrently for telemetry.
    Latched latched{*initialConfig, initialConfig->reference, std::vector<std::shared_ptr<ImgFrame>>(linkedInputs.size())};
    std::vector<impl::MaskRemapCache> maskRemaps(linkedInputs.size());
    bool dropping = false, droppingMasks = false;
    // Warn once for each episode of dropped rounds.
    const auto drop = [&logger](bool& episode, const char* reason) {
        if(!episode) logger->warn(reason);
        episode = true;
    };
    while(mainLoop()) {
        const auto beginning = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<ImgDetections>> messages;
        {
            auto blockEvent = inputBlockEvent();
            for(const auto& input : linkedInputs) {
                auto raw = input.second->get();
                if(!raw) break;
                auto message = std::dynamic_pointer_cast<ImgDetections>(raw);
                if(!message) throw std::runtime_error("ImgDetectionsFilter inputs accept only ImgDetections");
                messages.push_back(std::move(message));
            }
        }
        if(messages.size() != linkedInputs.size()) continue;
        const auto gotInput = std::chrono::steady_clock::now();
        latched.updateConfig(inputConfig, *logger);
        latched.updateReference(inputReference, *logger);
        const bool masksReady = latched.updateMasks(linkedSourceMasks, latched.reference ? latched.reference : messages.front()->transformation);
        if(!latched.reference && inputReference.isConnected()) {
            drop(dropping, "ImgDetectionsFilter dropping rounds until a valid reference arrives");
            continue;
        }
        dropping = false;
        if(!masksReady) {
            drop(droppingMasks, "ImgDetectionsFilter dropping rounds until all linked source masks match the output transformation");
            continue;
        }
        droppingMasks = false;
        auto output = impl::filterDetectionRound(messages, latched.config, latched.reference, latched.sourceMasks, &maskRemaps);
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
