#include "GPUStereo.hpp"

#include <CL/cl.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <spdlog/async_logger.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "gpu_stereo/config.hpp"
#include "gpu_stereo/PyramidStereo.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "pipeline/NodeTiming.hpp"

namespace {

class ScopeGuard {
   public:
    explicit ScopeGuard(std::function<void()> fn) : cleanup_(std::move(fn)) {}
    ~ScopeGuard() {
        if(cleanup_) cleanup_();
    }
    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;

private:
    std::function<void()> cleanup_;
};

void checkOpenClStatus(cl_int status, const char* step) {
    if(status != CL_SUCCESS) {
        throw std::runtime_error(std::string(step) + " failed, OpenCL status: " + std::to_string(status));
    }
}

constexpr double kGeometryEpsilon = 1e-6;

struct StereoGeometry {
    double baseline_m = 0.0;
    double focal_length_px = 0.0;
};

StereoGeometry computeStereoGeometry(const dai::ImgFrame& leftFrame,
                                     const dai::ImgFrame& rightFrame,
                                     const dai::CalibrationHandler& calib,
                                     int width,
                                     int height,
                                     const std::shared_ptr<spdlog::async_logger>& logger) {
    const auto M1 = leftFrame.transformation.getIntrinsicMatrix();
    StereoGeometry geom;
    geom.focal_length_px = static_cast<double>(M1[0][0]);

    try {
        const auto translationM =
            leftFrame.transformation.getTranslationVectorTo(rightFrame.transformation, false, dai::LengthUnit::METER);
        geom.baseline_m = std::abs(translationM[0]);
    } catch(const std::exception&) {
        geom.baseline_m = 0.0;
    }

    const bool needsBaseline = geom.baseline_m <= kGeometryEpsilon;
    const bool needsFocal = geom.focal_length_px <= kGeometryEpsilon;
    if(!needsBaseline && !needsFocal) {
        return geom;
    }

    try {
        const auto leftSocket = static_cast<dai::CameraBoardSocket>(leftFrame.getInstanceNum());
        const auto rightSocket = static_cast<dai::CameraBoardSocket>(rightFrame.getInstanceNum());

        if(needsBaseline) {
            const auto T = calib.getCameraTranslationVector(leftSocket, rightSocket, false, dai::LengthUnit::METER);
            geom.baseline_m = std::abs(T[0]);
        }

        if(needsFocal) {
            const auto K = calib.getCameraIntrinsics(leftSocket, width, height);
            geom.focal_length_px = static_cast<double>(K[0][0]);
        }

        if(logger) {
            logger->warn(
                "GPUStereo depth: frame extrinsics missing or zero baseline; using pipeline calibration "
                "(baseline={:.4f} m, focal={:.2f} px)",
                geom.baseline_m,
                geom.focal_length_px);
        }
    } catch(const std::exception& ex) {
        if(logger) {
            logger->warn("GPUStereo depth: calibration fallback failed: {}", ex.what());
        }
    }

    return geom;
}

const char* kDepthKernelSource = R"CL(
__kernel void disp_to_depth(__global const ushort* disp, __global ushort* depth, float num_x_scale, uint n) {
    uint i = get_global_id(0);
    if (i >= n) return;
    float d = (float)disp[i];
    depth[i] = (d <= 0.f) ? (ushort)0 : (ushort)clamp(num_x_scale / d, 0.f, 65535.f);
}
)CL";

}  // namespace

namespace dai {
namespace gate {

void GPUStereo::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    const int confidenceThreshold = properties.initialConfig.confidenceThreshold;

    std::shared_ptr<ImgFrame> leftFrame;
    std::shared_ptr<ImgFrame> rightFrame;
    {
        auto blockEvent = this->inputBlockEvent();
        leftFrame = leftInternal.get<ImgFrame>();
        rightFrame = rightInternal.get<ImgFrame>();
    }

    const int width = static_cast<int>(leftFrame->getWidth());
    const int height = static_cast<int>(leftFrame->getHeight());
    const int rightWidth = static_cast<int>(rightFrame->getWidth());
    const int rightHeight = static_cast<int>(rightFrame->getHeight());
    if(width <= 0 || height <= 0 || rightWidth <= 0 || rightHeight <= 0) {
        throw std::runtime_error("GPUStereo invalid frame size");
    }
    if(rightWidth != width || rightHeight != height) {
        throw std::runtime_error("GPUStereo expects left/right frames to have the same resolution");
    }
    if(leftFrame->getType() != ImgFrame::Type::RAW8 || rightFrame->getType() != ImgFrame::Type::RAW8) {
        throw std::runtime_error("GPUStereo expects RAW8 rectified inputs");
    }

    const bool depthConnected = !depth.getConnections().empty();

    StereoGeometry stereoGeom{};
    dai::CalibrationHandler calib;
    dai::Extrinsics leftExtr;
    dai::Extrinsics rightExtr;
    if(depthConnected) {
        calib = getParentPipeline().getCalibrationData();
        stereoGeom = computeStereoGeometry(*leftFrame, *rightFrame, calib, width, height, logger);
        leftExtr = leftFrame->transformation.getExtrinsics();
        rightExtr = rightFrame->transformation.getExtrinsics();
    }

    cl_int status = CL_SUCCESS;
    cl_uint numPlatforms = 0;
    checkOpenClStatus(clGetPlatformIDs(0, nullptr, &numPlatforms), "clGetPlatformIDs(count)");
    if(numPlatforms == 0) {
        throw std::runtime_error("No OpenCL platform found");
    }
    std::vector<cl_platform_id> platforms(numPlatforms);
    checkOpenClStatus(clGetPlatformIDs(numPlatforms, platforms.data(), nullptr), "clGetPlatformIDs(list)");

    cl_device_id deviceId = nullptr;
    for(auto platform : platforms) {
        cl_uint numDevices = 0;
        auto devStatus = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &deviceId, &numDevices);
        if(devStatus == CL_SUCCESS && numDevices > 0 && deviceId != nullptr) {
            break;
        }
    }
    if(deviceId == nullptr) {
        throw std::runtime_error("No OpenCL GPU device found");
    }

    cl_context context = clCreateContext(nullptr, 1, &deviceId, nullptr, nullptr, &status);
    checkOpenClStatus(status, "clCreateContext");
    ScopeGuard contextGuard([&] { clReleaseContext(context); });

    cl_command_queue commandQueue = clCreateCommandQueueWithProperties(context, deviceId, nullptr, &status);
    checkOpenClStatus(status, "clCreateCommandQueueWithProperties");
    ScopeGuard queueGuard([&] { clReleaseCommandQueue(commandQueue); });

    const bool disparityConnected = !disparity.getConnections().empty();
    const bool confidenceConnected = !confidenceMap.getConnections().empty();

    const size_t monoBytes = static_cast<size_t>(width) * static_cast<size_t>(height);
    const size_t disparityBytes = monoBytes * sizeof(uint16_t);

    cl_mem leftBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, monoBytes, nullptr, &status);
    checkOpenClStatus(status, "clCreateBuffer(left)");
    ScopeGuard leftGuard([&] { clReleaseMemObject(leftBuffer); });

    cl_mem rightBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR, monoBytes, nullptr, &status);
    checkOpenClStatus(status, "clCreateBuffer(right)");
    ScopeGuard rightGuard([&] { clReleaseMemObject(rightBuffer); });

    cl_mem disparityBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, disparityBytes, nullptr, &status);
    checkOpenClStatus(status, "clCreateBuffer(disparity)");
    ScopeGuard disparityGuard([&] { clReleaseMemObject(disparityBuffer); });

    cl_mem depthBuffer = nullptr;
    ScopeGuard depthGuard([&] {
        if(depthBuffer) clReleaseMemObject(depthBuffer);
    });

    cl_mem confidenceBuffer = nullptr;
    ScopeGuard confidenceGuard([&] {
        if(confidenceBuffer) clReleaseMemObject(confidenceBuffer);
    });

    cl_program depthProgram = nullptr;
    ScopeGuard programGuard([&] {
        if(depthProgram) clReleaseProgram(depthProgram);
    });

    cl_kernel depthKernel = nullptr;
    ScopeGuard kernelGuard([&] {
        if(depthKernel) clReleaseKernel(depthKernel);
    });

    if(depthConnected) {
        depthBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, disparityBytes, nullptr, &status);
        checkOpenClStatus(status, "clCreateBuffer(depth)");

        size_t depthSrcLen = std::strlen(kDepthKernelSource);
        depthProgram = clCreateProgramWithSource(context, 1, &kDepthKernelSource, &depthSrcLen, &status);
        checkOpenClStatus(status, "clCreateProgramWithSource(depth)");

        checkOpenClStatus(clBuildProgram(depthProgram, 1, &deviceId, nullptr, nullptr, nullptr), "clBuildProgram(depth)");

        depthKernel = clCreateKernel(depthProgram, "disp_to_depth", &status);
        checkOpenClStatus(status, "clCreateKernel(disp_to_depth)");
    }

    gpu_stereo::Config stereoConfig(width, height, confidenceThreshold);
    stereoConfig.output_confidence = confidenceConnected;
    auto stereo = std::make_unique<gpu_stereo::PyramidStereo>(context, deviceId, stereoConfig);
#ifdef TIMING
    stereo->enable_profiling(true);
#endif

    if(confidenceConnected) {
        confidenceBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, monoBytes, nullptr, &status);
        checkOpenClStatus(status, "clCreateBuffer(confidence)");
    }

    while(mainLoop()) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        {
            auto blockEvent = this->inputBlockEvent();
            leftFrame = leftInternal.get<ImgFrame>();
            rightFrame = rightInternal.get<ImgFrame>();
        }
        auto tGotInput = std::chrono::steady_clock::now();

        if(leftFrame->getType() != ImgFrame::Type::RAW8 || rightFrame->getType() != ImgFrame::Type::RAW8) {
            throw std::runtime_error("GPUStereo expects RAW8 rectified inputs");
        }
        if(static_cast<int>(leftFrame->getWidth()) != width || static_cast<int>(leftFrame->getHeight()) != height
           || static_cast<int>(rightFrame->getWidth()) != width || static_cast<int>(rightFrame->getHeight()) != height) {
            throw std::runtime_error("GPUStereo frame dimensions changed mid-stream");
        }

        if(depthConnected) {
            const auto curLeftExtr = leftFrame->transformation.getExtrinsics();
            const auto curRightExtr = rightFrame->transformation.getExtrinsics();
            if(!curLeftExtr.isEqualExtrinsics(leftExtr) || !curRightExtr.isEqualExtrinsics(rightExtr)) {
                leftExtr = curLeftExtr;
                rightExtr = curRightExtr;
                stereoGeom = computeStereoGeometry(*leftFrame, *rightFrame, calib, width, height, logger);
            }
        }

        checkOpenClStatus(  // PyramidStereo still runs buffer_to_image: its pyramid images are CL_FLOAT, so the upload converts uint8 to float
            clEnqueueWriteBuffer(commandQueue, leftBuffer, CL_TRUE, 0, monoBytes, leftFrame->getData().data(), 0, nullptr, nullptr),
            "clEnqueueWriteBuffer(left)");
        checkOpenClStatus(
            clEnqueueWriteBuffer(commandQueue, rightBuffer, CL_TRUE, 0, monoBytes, rightFrame->getData().data(), 0, nullptr, nullptr),
            "clEnqueueWriteBuffer(right)");

        stereo->execute(commandQueue, leftBuffer, rightBuffer, disparityBuffer, confidenceBuffer);

#ifdef TIMING
        {
            const auto& pd = stereo->get_profiling_data();
            if(pd.enabled && logger) {
                for(const auto& kv : pd.timings_ms) {
                    logger->debug("GPUStereo {} took {:.3f} ms", kv.first, kv.second);
                }
                auto itTotal = pd.timings_ms.find("total");
                auto itU = pd.timings_ms.find("upload");
                auto itD = pd.timings_ms.find("download");
                if(itTotal != pd.timings_ms.end() && itU != pd.timings_ms.end() && itD != pd.timings_ms.end()) {
                    const double net = itTotal->second - itU->second - itD->second;
                    if(net >= 0.0 && std::isfinite(net)) {
                        logger->debug("GPUStereo net took {:.3f} ms", net);
                    }
                }
            }
        }
#endif

        if(depthConnected) {
            const auto numXScale =
                static_cast<float>(stereoGeom.baseline_m * stereoConfig.depth_unit_multiplier * stereoGeom.focal_length_px
                                   * static_cast<double>(1 << stereoConfig.subpixel_bits));
            cl_uint pixelCount = static_cast<cl_uint>(monoBytes);
            checkOpenClStatus(clSetKernelArg(depthKernel, 0, sizeof(cl_mem), &disparityBuffer), "depth arg 0");
            checkOpenClStatus(clSetKernelArg(depthKernel, 1, sizeof(cl_mem), &depthBuffer), "depth arg 1");
            checkOpenClStatus(clSetKernelArg(depthKernel, 2, sizeof(float), &numXScale), "depth arg 2");
            checkOpenClStatus(clSetKernelArg(depthKernel, 3, sizeof(cl_uint), &pixelCount), "depth arg 3");
            size_t globalSize = ((monoBytes + 255) / 256) * 256;
            size_t localSize = 256;
            checkOpenClStatus(
                clEnqueueNDRangeKernel(commandQueue, depthKernel, 1, nullptr, &globalSize, &localSize, 0, nullptr, nullptr), "disp_to_depth");
        }

        auto makeFrame = [&](size_t bytes, ImgFrame::Type type) {
            auto frame = std::make_shared<ImgFrame>();
            frame->setData(std::vector<std::uint8_t>(bytes));
            frame->setMetadata(*leftFrame);
            frame->setWidth(width);
            frame->setHeight(height);
            frame->setType(type);
            frame->fb.stride = frame->fb.width * frame->getBytesPerPixel();
            return frame;
        };

        std::shared_ptr<ImgFrame> disparityFrame;
        if(disparityConnected) {
            disparityFrame = makeFrame(disparityBytes, ImgFrame::Type::RAW16);
            auto* mapped =
                clEnqueueMapBuffer(commandQueue, disparityBuffer, CL_TRUE, CL_MAP_READ, 0, disparityBytes, 0, nullptr, nullptr, &status);
            checkOpenClStatus(status, "clEnqueueMapBuffer(disparity read)");
            std::memcpy(disparityFrame->getData().data(), mapped, disparityBytes);
            checkOpenClStatus(clEnqueueUnmapMemObject(commandQueue, disparityBuffer, mapped, 0, nullptr, nullptr), "unmap disparity");
        }

        std::shared_ptr<ImgFrame> depthFrame;
        if(depthConnected) {
            depthFrame = makeFrame(disparityBytes, ImgFrame::Type::RAW16);
            auto* mapped = clEnqueueMapBuffer(commandQueue, depthBuffer, CL_TRUE, CL_MAP_READ, 0, disparityBytes, 0, nullptr, nullptr, &status);
            checkOpenClStatus(status, "clEnqueueMapBuffer(depth read)");
            std::memcpy(depthFrame->getData().data(), mapped, disparityBytes);
            checkOpenClStatus(clEnqueueUnmapMemObject(commandQueue, depthBuffer, mapped, 0, nullptr, nullptr), "unmap depth");
        }

        std::shared_ptr<ImgFrame> confidenceFrame;
        if(confidenceConnected) {
            confidenceFrame = makeFrame(monoBytes, ImgFrame::Type::RAW8);
            auto* mapped =
                clEnqueueMapBuffer(commandQueue, confidenceBuffer, CL_TRUE, CL_MAP_READ, 0, monoBytes, 0, nullptr, nullptr, &status);
            checkOpenClStatus(status, "clEnqueueMapBuffer(confidence read)");
            std::memcpy(confidenceFrame->getData().data(), mapped, monoBytes);
            checkOpenClStatus(clEnqueueUnmapMemObject(commandQueue, confidenceBuffer, mapped, 0, nullptr, nullptr), "unmap confidence");
        }

        auto tProcessed = std::chrono::steady_clock::now();
        {
            auto blockEvent = this->outputBlockEvent();
            if(disparityFrame) disparity.send(disparityFrame);
            if(depthFrame) depth.send(depthFrame);
            if(confidenceFrame) confidenceMap.send(confidenceFrame);
        }
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

}  // namespace gate
}  // namespace dai
