// The hardware stages of the FeatureTracker node over the C interface of the vendor module (vendor/dai_device_accel.h)

#include "pipeline/node/FeatureTracker/FeatureTrackerAccel.hpp"

#include "utilities/ErrorMacros.hpp"
#include "utilities/EvaDataMemory.hpp"
#include "vendor/DeviceError.hpp"

namespace dai {
namespace gate {

namespace {

constexpr const char* CORNER_UNIT = "Hardware corner detector";
constexpr const char* FLOW_UNIT = "Hardware optical flow";
constexpr const char* DESCRIPTOR_UNIT = "Hardware descriptor calculator";

/// The image of a frame without memory: the template frame of an init call, which gives only the size and the format
dai_device_image templateImage(const ImgFrame& frame) {
    dai_device_image image{};
    image.mem = nullptr;
    image.width = frame.getWidth();
    image.height = frame.getHeight();
    image.stride = frame.getStride();
    image.format = device::toDeviceFormat(frame.getType());
    image.p1_offset = frame.fb.p1Offset;
    image.p2_offset = frame.fb.p2Offset;
    image.p3_offset = frame.fb.p3Offset;
    return image;
}

/// Converts the data of the frame to an accelerator buffer (no copy for DMA-BUF memory) and puts it into the frame,
/// as before the C interface: the frame keeps the buffer alive, and the passthrough output sends it. `size` is the
/// input size of the unit (dai_device_*_in_min_bytes): the size of a copy.
std::shared_ptr<EvaDataMemory> toAcceleratorMemory(ImgFrame& frame, uint32_t size) {
    auto memory = EvaDataMemory::convert(frame.data, size);
    frame.data = memory;
    return memory;
}

}  // namespace

// ---------------------------------------------------------------------------
// CornerDetector

CornerDetector::CornerDetector() : handle(nullptr, &dai_device_corner_destroy) {
    device::installDeviceLog();
    dai_device_corner* corner = nullptr;
    DAI_CHECK_DEVICE(CORNER_UNIT, dai_device_corner_create(&corner));
    handle.reset(corner);
}

CornerDetector::~CornerDetector() = default;

int CornerDetector::Open() {
    return device::returnDriverStatus(dai_device_corner_open(handle.get()), CORNER_UNIT, "dai_device_corner_open");
}

int CornerDetector::Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync) {
    dai_device_image image{};
    if(inFrame) image = templateImage(*inFrame);
    return device::returnDriverStatus(
        dai_device_corner_init(handle.get(), inFrame ? &image : nullptr, runAsync ? 1 : 0), CORNER_UNIT, "dai_device_corner_init");
}

int CornerDetector::Config() {
    return device::returnDriverStatus(dai_device_corner_config(handle.get()), CORNER_UNIT, "dai_device_corner_config");
}

void CornerDetector::Run(std::shared_ptr<dai::ImgFrame> inImageData) {
    // No frame: the vendor module throws "ImgData error"
    std::shared_ptr<EvaDataMemory> memory;
    dai_device_image image{};
    if(inImageData) {
        memory = toAcceleratorMemory(*inImageData, dai_device_corner_in_min_bytes(handle.get()));
        image = device::toDeviceImage(*inImageData, *memory);
    }
    DAI_CHECK_DEVICE(CORNER_UNIT, dai_device_corner_run(handle.get(), inImageData ? &image : nullptr));
}

int CornerDetector::Close() {
    return device::returnDriverStatus(dai_device_corner_close(handle.get()), CORNER_UNIT, "dai_device_corner_close");
}

void CornerDetector::setThreshold(uint32_t threshold) {
    dai_device_corner_set_threshold(handle.get(), threshold);
}

void CornerDetector::setRobustness(uint8_t robustness) {
    dai_device_corner_set_robustness(handle.get(), robustness);
}

void CornerDetector::setGradientFilterShift(uint32_t shift) {
    dai_device_corner_set_gradient_filter_shift(handle.get(), shift);
}

const FeaturePoint* CornerDetector::points() const {
    const dai_device_feature_point* points = nullptr;
    size_t count = 0;
    DAI_CHECK_DEVICE(CORNER_UNIT, dai_device_corner_points(handle.get(), &points, &count));
    // Same layout (static_assert in the header): the output buffer of the hardware, no copy
    return reinterpret_cast<const FeaturePoint*>(points);
}

size_t CornerDetector::numPoints() const {
    const dai_device_feature_point* points = nullptr;
    size_t count = 0;
    DAI_CHECK_DEVICE(CORNER_UNIT, dai_device_corner_points(handle.get(), &points, &count));
    return count;
}

// ---------------------------------------------------------------------------
// OpticalFlow

OpticalFlow::OpticalFlow() : handle(nullptr, &dai_device_oflow_destroy) {
    device::installDeviceLog();
    dai_device_oflow* flow = nullptr;
    DAI_CHECK_DEVICE(FLOW_UNIT, dai_device_oflow_create(&flow));
    handle.reset(flow);
}

OpticalFlow::~OpticalFlow() = default;

int OpticalFlow::Open() {
    return device::returnDriverStatus(dai_device_oflow_open(handle.get()), FLOW_UNIT, "dai_device_oflow_open");
}

int OpticalFlow::Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync) {
    dai_device_image image{};
    if(inFrame) image = templateImage(*inFrame);
    // runAsync: the vendor module returns DAI_STATUS_UNSUPPORTED, so this throws
    return device::returnDriverStatus(
        dai_device_oflow_init(handle.get(), inFrame ? &image : nullptr, runAsync ? 1 : 0), FLOW_UNIT, "dai_device_oflow_init");
}

int OpticalFlow::Config() {
    return device::returnDriverStatus(dai_device_oflow_config(handle.get()), FLOW_UNIT, "dai_device_oflow_config");
}

void OpticalFlow::Run(std::shared_ptr<dai::ImgFrame> refImageData, std::shared_ptr<dai::ImgFrame> curImageData) {
    std::shared_ptr<EvaDataMemory> refMemory;
    std::shared_ptr<EvaDataMemory> curMemory;
    dai_device_image refImage{};
    dai_device_image curImage{};
    if(refImageData && curImageData) {
        // The current frame first, as before the C interface. Both frames have the input size of the unit.
        const uint32_t size = dai_device_oflow_in_min_bytes(handle.get());
        curMemory = toAcceleratorMemory(*curImageData, size);
        curImage = device::toDeviceImage(*curImageData, *curMemory);
        refMemory = toAcceleratorMemory(*refImageData, size);
        refImage = device::toDeviceImage(*refImageData, *refMemory);
    } else {
        // Only one frame: the vendor module does not read it (no conversion, as before); no frame: it throws
        if(refImageData) refImage = templateImage(*refImageData);
        if(curImageData) curImage = templateImage(*curImageData);
    }
    DAI_CHECK_DEVICE(FLOW_UNIT, dai_device_oflow_run(handle.get(), refImageData ? &refImage : nullptr, curImageData ? &curImage : nullptr));
}

int OpticalFlow::Close() {
    return device::returnDriverStatus(dai_device_oflow_close(handle.get()), FLOW_UNIT, "dai_device_oflow_close");
}

dai_device_oflow_result OpticalFlow::result() const {
    dai_device_oflow_result result{};
    DAI_CHECK_DEVICE(FLOW_UNIT, dai_device_oflow_result_get(handle.get(), &result));
    return result;
}

const int16_t* OpticalFlow::forwardMotionVectors() const {
    return result().forward_mv;
}

const int16_t* OpticalFlow::backwardMotionVectors() const {
    return result().backward_mv;
}

const uint8_t* OpticalFlow::confidenceMap() const {
    return result().confidence;
}

// ---------------------------------------------------------------------------
// DescriptorCalculator

DescriptorCalculator::DescriptorCalculator() : handle(nullptr, &dai_device_descriptor_destroy) {
    device::installDeviceLog();
    dai_device_descriptor* descriptor = nullptr;
    DAI_CHECK_DEVICE(DESCRIPTOR_UNIT, dai_device_descriptor_create(&descriptor));
    handle.reset(descriptor);
}

DescriptorCalculator::~DescriptorCalculator() = default;

int DescriptorCalculator::Open() {
    return device::returnDriverStatus(dai_device_descriptor_open(handle.get()), DESCRIPTOR_UNIT, "dai_device_descriptor_open");
}

int DescriptorCalculator::Init(std::shared_ptr<dai::ImgFrame> inFrame, bool runAsync) {
    dai_device_image image{};
    if(inFrame) image = templateImage(*inFrame);
    // runAsync: the vendor module returns DAI_STATUS_UNSUPPORTED, so this throws
    return device::returnDriverStatus(
        dai_device_descriptor_init(handle.get(), inFrame ? &image : nullptr, runAsync ? 1 : 0), DESCRIPTOR_UNIT, "dai_device_descriptor_init");
}

int DescriptorCalculator::Config() {
    return device::returnDriverStatus(dai_device_descriptor_config(handle.get()), DESCRIPTOR_UNIT, "dai_device_descriptor_config");
}

void DescriptorCalculator::Run(std::shared_ptr<dai::ImgFrame> inImageData, const std::vector<DescriptorSearchPosition>& positions) {
    // Nothing to do (and no conversion of the frame data) for an empty list, as before the C interface
    if(positions.empty()) {
        return;
    }
    searchPositions.resize(positions.size());
    for(size_t i = 0; i < positions.size(); i++) {
        searchPositions[i].x = positions[i].x;
        searchPositions[i].y = positions[i].y;
    }
    // No frame: the vendor module returns DAI_STATUS_INVALID_ARGUMENT (no image)
    std::shared_ptr<EvaDataMemory> memory;
    dai_device_image image{};
    if(inImageData) {
        memory = toAcceleratorMemory(*inImageData, dai_device_descriptor_in_min_bytes(handle.get()));
        image = device::toDeviceImage(*inImageData, *memory);
    }
    DAI_CHECK_DEVICE(DESCRIPTOR_UNIT,
                     dai_device_descriptor_run(handle.get(), inImageData ? &image : nullptr, searchPositions.data(), searchPositions.size()));
}

int DescriptorCalculator::Close() {
    return device::returnDriverStatus(dai_device_descriptor_close(handle.get()), DESCRIPTOR_UNIT, "dai_device_descriptor_close");
}

void DescriptorCalculator::setNumPoints(uint32_t numPoints) {
    // The vendor module clamps it to MAX_POINTS
    dai_device_descriptor_set_num_points(handle.get(), numPoints);
}

bool DescriptorCalculator::descriptorLowPassFilter() const {
    return dai_device_descriptor_low_pass_filter(handle.get()) != 0;
}

const uint8_t* DescriptorCalculator::descriptors() const {
    return dai_device_descriptor_data(handle.get());
}

}  // namespace gate
}  // namespace dai
