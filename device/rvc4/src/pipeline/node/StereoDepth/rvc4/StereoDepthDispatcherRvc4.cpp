#include "pipeline/node/StereoDepth/rvc4/StereoDepthDispatcherRvc4.hpp"

#include <utility>

#include "utilities/ErrorMacros.hpp"
#include "utilities/EvaDataMemory.hpp"
#include "vendor/DeviceError.hpp"

namespace {

constexpr const char* UNIT = "Hardware stereo matcher";

uint8_t toFlag(bool value) {
    return value ? 1 : 0;
}

uint32_t toDeviceSearchDir(dai::gate::StereoSearchDir dir) {
    switch(dir) {
        case dai::gate::StereoSearchDir::L2R:
            return DAI_STEREO_SEARCH_L2R;
        case dai::gate::StereoSearchDir::R2L:
            return DAI_STEREO_SEARCH_R2L;
    }
    // Not a direction: the vendor module rejects it with this value in the text
    return static_cast<uint32_t>(dir);
}

}  // namespace

StereoDepthDispatcher::StereoDepthDispatcher() : handle(nullptr, &dai_device_stereo_destroy) {
    dai::device::installDeviceLog();
    dai_device_stereo* stereo = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_stereo_create(&stereo));
    handle.reset(stereo);
}

StereoDepthDispatcher::~StereoDepthDispatcher() = default;

void StereoDepthDispatcher::RunAsync(const std::shared_ptr<const dai::ImgFrame>& inLeftFrame,
                                     const std::shared_ptr<const dai::ImgFrame>& inRightFrame,
                                     std::shared_ptr<dai::ImgFrame> outDispFrame,
                                     std::shared_ptr<dai::ImgFrame> outConfFrame,
                                     std::shared_ptr<dai::ImgFrame> outSLIC) {
    // The accelerator buffers, in the order of the former vendor code. The outputs get their buffer as frame data.
    RunBuffers buffers;
    buffers.disparity = dai::EvaDataMemory::convert(outDispFrame->data);
    outDispFrame->data = buffers.disparity;
    buffers.left = dai::EvaDataMemory::convert(inLeftFrame->data);
    buffers.right = dai::EvaDataMemory::convert(inRightFrame->data);
    if(outConfFrame) {
        buffers.confidence = dai::EvaDataMemory::convert(outConfFrame->data);
        outConfFrame->data = buffers.confidence;
    }
    if(outSLIC) {
        buffers.slic = dai::EvaDataMemory::convert(outSLIC->data);
        outSLIC->data = buffers.slic;
    }

    const dai_device_image left = dai::device::toDeviceImage(*inLeftFrame, *buffers.left);
    const dai_device_image right = dai::device::toDeviceImage(*inRightFrame, *buffers.right);
    const dai_device_image disparity = dai::device::toDeviceImage(*outDispFrame, *buffers.disparity);
    dai_device_image confidence{};
    if(outConfFrame) confidence = dai::device::toDeviceImage(*outConfFrame, *buffers.confidence);
    dai_device_image slic{};
    if(outSLIC) slic = dai::device::toDeviceImage(*outSLIC, *buffers.slic);

    DAI_CHECK_DEVICE(UNIT,
                     dai_device_stereo_run_async(
                         handle.get(), &left, &right, &disparity, outConfFrame ? &confidence : nullptr, outSLIC ? &slic : nullptr));
    running = std::move(buffers);
}

void StereoDepthDispatcher::Run(const std::shared_ptr<const dai::ImgFrame>& inLeftFrame,
                                const std::shared_ptr<const dai::ImgFrame>& inRightFrame,
                                std::shared_ptr<dai::ImgFrame> outDispFrame,
                                std::shared_ptr<dai::ImgFrame> outConfFrame,
                                std::shared_ptr<dai::ImgFrame> outSLIC) {
    RunAsync(inLeftFrame, inRightFrame, std::move(outDispFrame), std::move(outConfFrame), std::move(outSLIC));
    Wait();
}

int StereoDepthDispatcher::Wait() {
    const dai_status status = dai_device_stereo_wait(handle.get());
    running = RunBuffers{};
    // As before the C interface: the negated EVA status of the work (0: done)
    return -dai::device::returnDriverStatus(status, UNIT, "dai_device_stereo_wait");
}

uint32_t StereoDepthDispatcher::Open() {
    return dai::device::returnDriverStatus(dai_device_stereo_open(handle.get()), UNIT, "dai_device_stereo_open");
}

uint32_t StereoDepthDispatcher::Init(StereoInit& init) {
    dai_device_stereo_init_params params{};
    params.conf_output_enable = toFlag(init.bConfOutputEnable);
    params.slic_output_enable = toFlag(init.bSlicOutputEnable);
    params.actual_fps = init.nActualFps;
    params.operational_fps = init.nOperationalFps;
    params.width = init.nWidth;
    params.height = init.nHeight;
    params.format = dai::device::toDeviceFormat(init.colorFormat);
    return dai::device::returnDriverStatus(dai_device_stereo_init(handle.get(), &params), UNIT, "dai_device_stereo_init");
}

uint32_t StereoDepthDispatcher::Config(StereoConfig& c) {
    const dai::StereoDepthConfig& cfg = c.cfg;
    dai_device_stereo_config_params params{};

    params.census_noise_threshold_offset = cfg.censusTransform.noiseThresholdOffset;
    params.census_noise_threshold_scale = cfg.censusTransform.noiseThresholdScale;

    params.occlusion_confidence_weight = cfg.confidenceMetrics.occlusionConfidenceWeight;
    params.motion_vector_confidence_weight = cfg.confidenceMetrics.motionVectorConfidenceWeight;
    params.motion_vector_confidence_threshold = cfg.confidenceMetrics.motionVectorConfidenceThreshold;
    params.flatness_confidence_weight = cfg.confidenceMetrics.flatnessConfidenceWeight;
    params.flatness_confidence_threshold = cfg.confidenceMetrics.flatnessConfidenceThreshold;
    params.flatness_override = toFlag(cfg.confidenceMetrics.flatnessOverride);

    params.p1.enable_adaptive = toFlag(cfg.costAggregation.p1Config.enableAdaptive);
    params.p1.default_value = cfg.costAggregation.p1Config.defaultValue;
    params.p1.edge_value = cfg.costAggregation.p1Config.edgeValue;
    params.p1.smooth_value = cfg.costAggregation.p1Config.smoothValue;
    params.p1.edge_threshold = cfg.costAggregation.p1Config.edgeThreshold;
    params.p1.smooth_threshold = cfg.costAggregation.p1Config.smoothThreshold;

    params.p2.enable_adaptive = toFlag(cfg.costAggregation.p2Config.enableAdaptive);
    params.p2.default_value = cfg.costAggregation.p2Config.defaultValue;
    params.p2.edge_value = cfg.costAggregation.p2Config.edgeValue;
    params.p2.smooth_value = cfg.costAggregation.p2Config.smoothValue;

    params.hole_filling_enable = toFlag(cfg.postProcessing.holeFilling.enable);
    params.hole_filling_high_confidence_threshold = cfg.postProcessing.holeFilling.highConfidenceThreshold;
    params.hole_filling_fill_confidence_threshold = cfg.postProcessing.holeFilling.fillConfidenceThreshold;
    params.hole_filling_min_valid_disparity = cfg.postProcessing.holeFilling.minValidDisparity;
    params.hole_filling_invalidate_disparities = toFlag(cfg.postProcessing.holeFilling.invalidateDisparities);

    params.adaptive_median_enable = toFlag(cfg.postProcessing.adaptiveMedianFilter.enable);
    params.adaptive_median_confidence_threshold = cfg.postProcessing.adaptiveMedianFilter.confidenceThreshold;

    params.search_dir = toDeviceSearchDir(c.eSearchDir);

    return dai::device::returnDriverStatus(dai_device_stereo_config(handle.get(), &params), UNIT, "dai_device_stereo_config");
}

uint32_t StereoDepthDispatcher::Close() {
    return dai::device::returnDriverStatus(dai_device_stereo_close(handle.get()), UNIT, "dai_device_stereo_close");
}

uint32_t StereoDepthDispatcher::getMinDisparityImgSize() const {
    return dai_device_stereo_min_disparity_bytes(handle.get());
}

uint32_t StereoDepthDispatcher::getMinConfImgSize() const {
    return dai_device_stereo_min_confidence_bytes(handle.get());
}

uint32_t StereoDepthDispatcher::getMinSlicImgSize() const {
    return dai_device_stereo_min_slic_bytes(handle.get());
}
