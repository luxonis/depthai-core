#include "pipeline/node/Warp/rvc4/WarpDispatcherRvc4.hpp"

#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

#include "utilities/AcceleratorMemory.hpp"
#include "utilities/ErrorMacros.hpp"
#include "vendor/DeviceError.hpp"

namespace {
constexpr const char* UNIT = "Hardware warp";
}  // namespace

WarpDispatcher::WarpDispatcher() : handle(nullptr, &dai_device_warp_destroy) {
    dai::device::installDeviceLog();
    dai_device_warp* warp = nullptr;
    DAI_CHECK_DEVICE(UNIT, dai_device_warp_create(&warp));
    handle.reset(warp);
}

WarpDispatcher::~WarpDispatcher() = default;

void WarpDispatcher::Run(const std::shared_ptr<const dai::ImgFrame>& inputFrame, std::shared_ptr<dai::ImgFrame> outputFrame, size_t inputIndex) {
    RunAsync(inputFrame, std::move(outputFrame), inputIndex);
    Wait();
}

void WarpDispatcher::RunAsync(const std::shared_ptr<const dai::ImgFrame>& inputFrame, std::shared_ptr<dai::ImgFrame> outputFrame, size_t inputIndex) {
    // The output must be large enough before the conversion, as in the former vendor code
    if(!(outputFrame->data->getData().size() >= getMinOutImgSize())) {
        throw std::runtime_error("WRAP: Assert (outputFrame->data->getData().size() >= getMinOutImgSize())");
    }

    // The accelerator buffers, in the order of the former vendor code. The output gets its buffer as frame data; a
    // copy of the input gets the input size of the hardware.
    std::shared_ptr<dai::AcceleratorMemory> output = dai::AcceleratorMemory::convert(outputFrame->data);
    outputFrame->data = output;
    std::shared_ptr<dai::AcceleratorMemory> input = dai::AcceleratorMemory::convert(inputFrame->data, dai_device_warp_min_in_bytes(handle.get()));

    const dai_device_image inImage = dai::device::toDeviceImage(*inputFrame, *input);
    const dai_device_image outImage = dai::device::toDeviceImage(*outputFrame, *output);
    DAI_CHECK_DEVICE(UNIT, dai_device_warp_run_async(handle.get(), &inImage, &outImage, inputIndex));
    runningInput = std::move(input);
    runningOutput = std::move(output);
}

int WarpDispatcher::Wait() {
    const dai_status status = dai_device_warp_wait(handle.get());
    runningInput.reset();
    runningOutput.reset();
    // As before the C interface: the negated accelerator driver status of the work (0: done)
    return -dai::device::returnDriverStatus(status, UNIT, "dai_device_warp_wait");
}

uint32_t WarpDispatcher::Open() {
    return dai::device::returnDriverStatus(dai_device_warp_open(handle.get()), UNIT, "dai_device_warp_open");
}

uint32_t WarpDispatcher::Init(dai::WarpProperties config, uint32_t inWidth, uint32_t inHeight, dai::ImgFrame::Type inClrFmt, dai::ImgFrame::Type outClrFmt) {
    dai_device_warp_init_params params{};
    params.output_width = config.outputWidth;
    params.output_height = config.outputHeight;
    params.mesh_width = config.meshWidth;
    params.mesh_height = config.meshHeight;
    // Throws "Unsupported interpolation type: <n>" for the interpolations that the hardware does not have
    params.interpolation = dai::device::toDeviceInterpolation(config.interpolation);
    params.in_width = inWidth;
    params.in_height = inHeight;
    params.in_format = dai::device::toDeviceFormat(inClrFmt);
    params.out_format = dai::device::toDeviceFormat(outClrFmt);
    return dai::device::returnDriverStatus(dai_device_warp_init(handle.get(), &params), UNIT, "dai_device_warp_init");
}

uint32_t WarpDispatcher::Close() {
    return dai::device::returnDriverStatus(dai_device_warp_close(handle.get()), UNIT, "dai_device_warp_close");
}

uint32_t WarpDispatcher::getMinOutImgSize() const {
    return dai_device_warp_min_out_bytes(handle.get());
}

void WarpDispatcher::SetWarpMesh(std::vector<double>& rawGridX, std::vector<double>& rawGridY, size_t inputIndex) {
    DAI_CHECK_DEVICE(UNIT, dai_device_warp_set_mesh(handle.get(), rawGridX.data(), rawGridY.data(), rawGridX.size(), inputIndex));
}

void WarpDispatcher::SetWarpMesh(cv::Mat mesh_x, cv::Mat mesh_y, size_t inputIndex) {
    std::vector<double> rawGridX, rawGridY;

    if(mesh_x.type() == CV_32FC1 && mesh_y.type() == CV_32FC1) {
        for(int i = 0; i < mesh_x.rows; i++) {
            for(int j = 0; j < mesh_x.cols; j++) {
                rawGridX.push_back(mesh_x.at<float>(i, j));
                rawGridY.push_back(mesh_y.at<float>(i, j));
            }
        }
    } else if(mesh_x.type() == CV_64FC1 && mesh_y.type() == CV_64FC1) {
        for(int i = 0; i < mesh_x.rows; i++) {
            for(int j = 0; j < mesh_x.cols; j++) {
                rawGridX.push_back(mesh_x.at<double>(i, j));
                rawGridY.push_back(mesh_y.at<double>(i, j));
            }
        }
    } else {
        throw std::runtime_error("Mesh data type is not supported.");
    }

    SetWarpMesh(rawGridX, rawGridY, inputIndex);
}

void WarpDispatcher::ParseMeshResourceData(std::vector<uint8_t>& data, int width, int height, std::vector<double>& rawGridX, std::vector<double>& rawGridY) {
    if(width <= 0 || height <= 0) {
        throw std::runtime_error("Mesh dimensions must be positive");
    }
    // Rows of (y, x) float pairs: tightly packed (StereoDepth mesh files) or padded to 16 bytes (Warp::setWarpMesh)
    constexpr size_t ALIGNMENT = 16;
    const size_t packedStride = static_cast<size_t>(width) * sizeof(float) * 2;
    const size_t alignedStride = (packedStride + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1);
    if(static_cast<size_t>(height) > std::numeric_limits<size_t>::max() / alignedStride) {
        throw std::runtime_error("Mesh dimensions overflow the resource size");
    }
    size_t meshStride = 0;
    if(data.size() == packedStride * static_cast<size_t>(height)) {
        meshStride = packedStride;
    } else if(data.size() == alignedStride * static_cast<size_t>(height)) {
        meshStride = alignedStride;
    } else {
        throw std::runtime_error("Mesh resource size does not match its dimensions");
    }

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            float point[2];
            std::memcpy(point, data.data() + meshStride * i + j * sizeof(point), sizeof(point));
            // these are reversed
            rawGridX.push_back(point[1]);
            rawGridY.push_back(point[0]);
        }
    }
}
