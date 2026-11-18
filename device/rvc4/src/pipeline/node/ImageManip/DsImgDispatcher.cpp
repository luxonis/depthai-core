#include "pipeline/node/ImageManip/DsImgDispatcher.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "utilities/ErrorMacros.hpp"
#include "utilities/EvaDataMemory.hpp"
#include "vendor/DeviceError.hpp"

namespace {

constexpr const char* UNIT = "Hardware downscaler";

uint32_t toDeviceScaledownInterpolation(dai::gate::ScaledownInterpolation interpolation) {
    switch(interpolation) {
        case dai::gate::ScaledownInterpolation::BILINEAR:
            return DAI_INTERPOLATION_BILINEAR;
        case dai::gate::ScaledownInterpolation::BICUBIC:
            return DAI_INTERPOLATION_BICUBIC;
    }
    // Not a known value: the vendor module rejects it ("Unsupported scaledown interpolation type")
    return static_cast<uint32_t>(interpolation);
}

// A minimum buffer size of the vendor module: 0 when the query failed. The former methods threw in that case.
uint32_t checkBytes(uint32_t bytes, const char* call) {
    if(bytes == 0) {
        throw std::runtime_error(std::string(UNIT) + ": " + call + " failed: " + dai_device_last_error());
    }
    return bytes;
}

}  // namespace

DsImgDispatcher::DsImgDispatcher() : handle(nullptr, &dai_device_dsimg_destroy) {
    dai::device::installDeviceLog();
    dai_device_dsimg* created = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_create(&created));
    handle.reset(created);
}

DsImgDispatcher::~DsImgDispatcher() = default;

void DsImgDispatcher::startSession() {
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_start_session(handle.get()));
}

void DsImgDispatcher::RunAsync(std::shared_ptr<dai::ImgFrame> inFrame, std::shared_ptr<dai::ImgFrame> outFrame, bool useMetadata) {
    // Input: the frame data in an accelerator buffer of at least the minimum input size. The frame keeps the buffer.
    const uint32_t inMinBytes = GetInMinDsImgBytes(inFrame->getWidth(), inFrame->getHeight());
    std::shared_ptr<dai::EvaDataMemory> inImg = dai::EvaDataMemory::convert(inFrame->data, inMinBytes);
    inFrame->data = inImg;

    // Output: the frame data must be large enough before the conversion. The buffer is not assigned to the frame.
    const uint32_t outMinBytes = GetOutMinDsImgBytes(outFrame->getWidth(), outFrame->getHeight());
    if(!(outFrame->data->getData().size() >= outMinBytes)) {
        throw std::runtime_error("DS: Assert (outFrame->data->getData().size() >= minMemSize)");
    }
    std::shared_ptr<dai::EvaDataMemory> outImg = dai::EvaDataMemory::convert(outFrame->data);

    const dai_device_image in = dai::device::toDeviceImage(*inFrame, *inImg);
    dai_device_image out = dai::device::toDeviceImage(*outFrame, *outImg);

    inMem = std::move(inImg);
    outMem = std::move(outImg);
    const dai_status status = dai_device_dsimg_run_async(handle.get(), &in, &out, useMetadata ? 1 : 0);
    if(status != DAI_STATUS_OK) {
        inMem.reset();
        outMem.reset();
        dai::device::throwDeviceError(status, UNIT, "dai_device_dsimg_run_async", nullptr);
    }

    if(!useMetadata) {
        // The unit computed the layout of the output
        outFrame->setType(inFrame->getType());
        outFrame->fb.p1Offset = out.p1_offset;
        outFrame->fb.p2Offset = out.p2_offset;
        outFrame->fb.stride = out.stride;
    }
}

void DsImgDispatcher::Run(std::shared_ptr<dai::ImgFrame> inFrame, std::shared_ptr<dai::ImgFrame> outFrame, bool useMetadata) {
    RunAsync(std::move(inFrame), std::move(outFrame), useMetadata);
    Wait();
}

int DsImgDispatcher::Wait() {
    const dai_status status = dai_device_dsimg_wait(handle.get());
    inMem.reset();
    outMem.reset();
    // The C call gives the EVA status of the work (0 = OK); this method returns it negated, as before
    return -dai::device::returnDriverStatus(status, UNIT, "dai_device_dsimg_wait");
}

void DsImgDispatcher::registerNode() {
    dai_device_dsimg_register_node(handle.get());
}

void DsImgDispatcher::firstFrameOver() {
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_first_frame_over(handle.get()));
}

// Open, Init, Config and Close return 0 or throw, as before the C interface: the nodes ignore the return value of
// Config, so an error (also a driver status) must throw.
uint32_t DsImgDispatcher::Open() {
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_open(handle.get()));
    return 0;
}

uint32_t DsImgDispatcher::Init(DsImgInit& init) {
    dai_device_dsimg_init_params params{};
    params.actual_fps = init.nActualFps;
    params.operational_fps = init.nOperationalFps;
    params.max_src_width = init.nMaxSrcWidth;
    params.max_src_height = init.nMaxSrcHeight;
    params.min_dst_width = init.nMinDstWidth;
    params.min_dst_height = init.nMinDstHeight;
    params.format = dai::device::toDeviceFormat(init.colorFormat);
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_init(handle.get(), &params));
    initColorFormat = init.colorFormat;
    return 0;
}

uint32_t DsImgDispatcher::Config(DsImgConfig& c) {
    dai_device_dsimg_config_params params{};
    params.interpolation = toDeviceScaledownInterpolation(c.eDsInterpolation);
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_config(handle.get(), &params));
    return 0;
}

uint32_t DsImgDispatcher::Close() {
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_close(handle.get()));
    return 0;
}

uint32_t DsImgDispatcher::GetInMinDsImgBytes(int width, int height) {
    return checkBytes(dai_device_dsimg_in_min_bytes(handle.get(), width, height), "dai_device_dsimg_in_min_bytes");
}

uint32_t DsImgDispatcher::GetOutMinDsImgBytes(int width, int height) {
    return checkBytes(dai_device_dsimg_out_min_bytes(handle.get(), width, height), "dai_device_dsimg_out_min_bytes");
}

void DsImgDispatcher::PrintFrameInfo(int width, int height, spdlog::logger& logger) {
    dai_device_image_info info{};
    DAI_CHECK_DEVICE(UNIT, dai_device_dsimg_image_info(handle.get(), width, height, &info));
    logger.debug("DS_IMG: Input image expected propr: format={} width={} height={} size={} plane={} stride={} aligned size={} colorFormat={}",
                 info.driver_format,
                 info.width,
                 info.height,
                 info.total_size,
                 info.planes,
                 info.stride,
                 info.aligned_size,
                 (int)initColorFormat);
}
