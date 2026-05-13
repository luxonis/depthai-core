#include "depthai/pipeline/node/FocusedDepth.hpp"

#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

FocusedDepth::FocusedDepth(const std::shared_ptr<Device>& device) : DeviceNodeGroup(device) {
    DAI_CHECK_V(device != nullptr, "FocusedDepth requires a device.");
}

FocusedDepth::Backend FocusedDepth::resolveSessionBackend() const {
    if(backend_ == Backend::STEREO) {
        return Backend::STEREO;
    }
    if(backend_ == Backend::NEURAL) {
        return Backend::NEURAL;
    }
    // AUTO heuristic: large ROI or high FPS → StereoDepth; otherwise NeuralDepth.
    const uint64_t area = static_cast<uint64_t>(maxRoiWidth_) * static_cast<uint64_t>(maxRoiHeight_);
    const uint64_t budget = 576u * 360u;
    if(area > budget || targetFps_ > 35.F) {
        return Backend::STEREO;
    }
    return Backend::NEURAL;
}

DeviceModelZoo FocusedDepth::pickNeuralModelForRoi(int width, int height) {
    static const struct {
        DeviceModelZoo zoo;
        int w;
        int h;
    } kModels[] = {
        {DeviceModelZoo::NEURAL_DEPTH_192X120, 192, 120},   {DeviceModelZoo::NEURAL_DEPTH_288X180, 288, 180},
        {DeviceModelZoo::NEURAL_DEPTH_384X240, 384, 240}, {DeviceModelZoo::NEURAL_DEPTH_480X300, 480, 300},
        {DeviceModelZoo::NEURAL_DEPTH_576X360, 576, 360},  {DeviceModelZoo::NEURAL_DEPTH_768X480, 768, 480},
        {DeviceModelZoo::NEURAL_DEPTH_864X540, 864, 540},  {DeviceModelZoo::NEURAL_DEPTH_960X600, 960, 600},
        {DeviceModelZoo::NEURAL_DEPTH_1056X660, 1056, 660}, {DeviceModelZoo::NEURAL_DEPTH_1248X780, 1248, 780},
    };
    for(const auto& m : kModels) {
        if(m.w >= width && m.h >= height) {
            return m.zoo;
        }
    }
    return DeviceModelZoo::NEURAL_DEPTH_1248X780;
}

void FocusedDepth::buildInternal() {
    if(graphBuilt_) {
        return;
    }
    if(parent.lock() == nullptr) {
        return;
    }
    const auto device = getDevice();
    DAI_CHECK_V(device != nullptr, "FocusedDepth requires a device.");
    DAI_CHECK_V(device->getPlatform() == Platform::RVC4, "FocusedDepth is only supported on RVC4.");

    Pipeline pipeline = getParentPipeline();
    DAI_CHECK_V(pipeline.impl() != nullptr, "FocusedDepth must be part of a pipeline.");

    const Backend session = resolveSessionBackend();

    sync->setRunOnHost(true);
    crop_->setRunOnHost(true);
    crop_->getProperties().imageManip.outputFrameSize = 16 * 1024 * 1024;
    crop_->getProperties().maxDisparityPixels = stereoInitialConfig->getMaxDisparity();

    sync->out.link(crop_->input);

    if(session == Backend::STEREO) {
        stereoBackend_ = std::make_unique<Subnode<StereoDepth>>(*this, "focusedStereoDepth");
        (**stereoBackend_).initialConfig = stereoInitialConfig;
        crop_->getProperties().neuralResizeWidth = 0;
        crop_->getProperties().neuralResizeHeight = 0;
        (**stereoBackend_)
            .build(crop_->leftOut, crop_->rightOut, StereoDepth::PresetMode::DEFAULT);
        depthOut_ = &(**stereoBackend_).depth;
        confidenceOut_ = &(**stereoBackend_).confidenceMap;
    } else {
        neuralBackend_ = std::make_unique<Subnode<NeuralDepth>>(*this, "focusedNeuralDepth");
        DeviceModelZoo model = hasNeuralModelOverride_ ? neuralModelOverride_ : pickNeuralModelForRoi(static_cast<int>(maxRoiWidth_), static_cast<int>(maxRoiHeight_));
        const auto isz = NeuralDepth::getInputSize(model);
        crop_->getProperties().neuralResizeWidth = static_cast<uint32_t>(isz.first);
        crop_->getProperties().neuralResizeHeight = static_cast<uint32_t>(isz.second);
        (**neuralBackend_).build(crop_->leftOut, crop_->rightOut, model);
        depthOut_ = &(**neuralBackend_).depth;
        confidenceOut_ = &(**neuralBackend_).confidence;
    }

    graphBuilt_ = true;
}

Node::Output& FocusedDepth::depth() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(depthOut_ != nullptr, "FocusedDepth: depth output missing.");
    return *depthOut_;
}

Node::Output& FocusedDepth::confidence() {
    if(!graphBuilt_) {
        buildInternal();
    }
    DAI_CHECK_V(confidenceOut_ != nullptr, "FocusedDepth: confidence output missing.");
    return *confidenceOut_;
}

}  // namespace node
}  // namespace dai
