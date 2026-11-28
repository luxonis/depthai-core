#include "pipeline/node/StereoDepth/PyrImgDispatcher.hpp"

#include <utility>

#include "utilities/AcceleratorMemory.hpp"
#include "utilities/ErrorMacros.hpp"
#include "vendor/DeviceError.hpp"

namespace {

constexpr const char* UNIT = "Hardware image pyramid";

uint32_t toDeviceScaledownInterpolation(dai::gate::ScaledownInterpolation interpolation) {
    switch(interpolation) {
        case dai::gate::ScaledownInterpolation::BILINEAR:
            return DAI_INTERPOLATION_BILINEAR;
        case dai::gate::ScaledownInterpolation::BICUBIC:
            return DAI_INTERPOLATION_BICUBIC;
    }
    return static_cast<uint32_t>(interpolation);  // Unknown value: the vendor module rejects it
}

}  // namespace

PyrImgDispatcher::PyrImgDispatcher() : handle(nullptr, &dai_device_pyrimg_destroy) {
    dai::device::installDeviceLog();
    dai_device_pyrimg* created = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_pyrimg_create(&created));
    handle.reset(created);
}

PyrImgDispatcher::~PyrImgDispatcher() = default;

void PyrImgDispatcher::RunAsync(const std::shared_ptr<const dai::ImgFrame>& inFrame, std::vector<std::shared_ptr<dai::ImgFrame>> outFrames) {
    std::vector<std::shared_ptr<dai::AcceleratorMemory>> outImgs;  // A level without a frame gets an image without memory: the unit skips it
    outImgs.reserve(outFrames.size());
    std::vector<dai_device_image> outs(outFrames.size());
    for(size_t n = 0; n < outFrames.size(); n++) {
        if(outFrames[n] == nullptr) continue;
        auto out = dai::AcceleratorMemory::convert(outFrames[n]->data);
        outs[n] = dai::device::toDeviceImage(*outFrames[n], *out);
        outImgs.push_back(std::move(out));
    }
    auto inImg = dai::AcceleratorMemory::convert(inFrame->data);
    const dai_device_image in = dai::device::toDeviceImage(*inFrame, *inImg);

    inMem = std::move(inImg);
    outMems = std::move(outImgs);
    const dai_status status = dai_device_pyrimg_run_async(handle.get(), &in, outs.data(), outs.size());
    if(status != DAI_STATUS_OK) {
        inMem.reset();
        outMems.clear();
        dai::device::throwDeviceError(status, UNIT, "dai_device_pyrimg_run_async", nullptr);
    }
}

void PyrImgDispatcher::Run(const std::shared_ptr<const dai::ImgFrame>& inFrame, std::vector<std::shared_ptr<dai::ImgFrame>> outFrames) {
    RunAsync(inFrame, std::move(outFrames));
    Wait();
}

int PyrImgDispatcher::Wait() {
    const dai_status status = dai_device_pyrimg_wait(handle.get());
    inMem.reset();
    outMems.clear();
    return -dai::device::returnDriverStatus(status, UNIT, "dai_device_pyrimg_wait");  // Returns the negated accelerator driver status (0 = OK)
}

uint32_t PyrImgDispatcher::Open() {  // Open, Init, Config and Close return 0 or throw: the nodes ignore the return value of Config
    DAI_CHECK_DEVICE(UNIT, dai_device_pyrimg_open(handle.get()));
    return 0;
}

uint32_t PyrImgDispatcher::Init(PyrImgInit& init) {
    dai_device_pyrimg_init_params params{};
    params.out_base_image = init.bOutBaseimage ? 1 : 0;
    params.actual_fps = init.nActualFps;
    params.operational_fps = init.nOperationalFps;
    params.width = init.nWidth;
    params.height = init.nHeight;
    params.format = dai::device::toDeviceFormat(init.colorFormat);
    params.octaves = init.nOctaves;
    params.scales_per_octave = init.nScalesPerOctave;
    DAI_CHECK_DEVICE(UNIT, dai_device_pyrimg_init(handle.get(), &params));
    return 0;
}

uint32_t PyrImgDispatcher::Config(PyrImgConfig& c) {
    dai_device_pyrimg_config_params params{};
    params.interpolation = toDeviceScaledownInterpolation(c.eDsInterpolation);
    DAI_CHECK_DEVICE(UNIT, dai_device_pyrimg_config(handle.get(), &params));
    return 0;
}

uint32_t PyrImgDispatcher::Close() {
    DAI_CHECK_DEVICE(UNIT, dai_device_pyrimg_close(handle.get()));
    return 0;
}

uint32_t PyrImgDispatcher::GetMinPyrImgBytes(int level) const {
    return dai_device_pyrimg_min_bytes(handle.get(), level);
}
