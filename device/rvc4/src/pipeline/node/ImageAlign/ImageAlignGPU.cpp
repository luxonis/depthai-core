#include "ImageAlignGPU.hpp"

#include <climits>
#include <cmath>
#include <stdexcept>
#include <string>

#include "pipeline/node/ImageManip/rvc4/gpu/ImageManipGPU.hpp"
#include "pipeline/utilities/Alignment/ImageAlignCpu.hpp"

namespace dai::gate {
namespace {
const char* sourceCode = R"CLC(
#pragma OPENCL FP_CONTRACT OFF
__kernel void scatter_depth(__global const ushort* src, __global const int* map,
                            __global const float* rays, __global const float* shifts, __global uint* winners,
                            int width, int count, float shift) {
    int i = get_global_id(0);
    if(i >= count || map[i] < 0) return;
    ushort depth = src[map[i]];
    if(!depth || rays[i] == 0.0f) return;
    int x = i % width;
    float projected = x + shifts[depth] * rays[i];
    if(!(projected > -1.5f && projected < width + 0.5f)) return;
    int u = projected >= 0.5f ? (int)(projected + 0.5f) : (projected <= -0.5f ? -1 : 0);
    // Match the DSP's ordered overwrite, including its one-pixel hole fill.
    // High bits reproduce traversal order; low bits carry the winning source depth.
    uint rank = ((uint)(shift > 0 ? width - x : x + 1) << 16) | depth;
    int row = i - x;
    if(u >= 0 && u < width) atomic_max(winners + row + u, rank);
    if(u != x) {
        u += shift > 0 ? 1 : -1;
        if(u >= 0 && u < width) atomic_max(winners + row + u, rank);
    }
}
__kernel void gather_depth(__global const int* map2, __global const uint* winners,
                           __global const float* scales, __global ushort* dst,
                           int count, float offset) {
    int i = get_global_id(0);
    if(i >= count) return;
    int p = map2[i];
    ushort result = 0;
    if(p >= 0 && winners[p]) {
        ushort depth = (ushort)(winners[p] & 0xffffu);
        float z = (depth + offset) * scales[i];
        if(z >= 0.5f && z < 65535.5f) result = (ushort)(z + 0.5f);
    }
    dst[i] = result;
}
)CLC";
using utility::checkStatus;
std::vector<int> indices(const cv::Mat& x, const cv::Mat& y, int width, int height) {
    std::vector<int> result(x.total(), -1);
    for(int row = 0; row < x.rows; ++row) {
        for(int col = 0; col < x.cols; ++col) {
            const float u = x.at<float>(row, col), v = y.at<float>(row, col);
            if(!std::isfinite(u) || !std::isfinite(v) || u < -1 || v < -1 || u > width || v > height) continue;
            const int ix = cvRound(u), iy = cvRound(v);
            if(ix >= 0 && iy >= 0 && ix < width && iy < height) result[row * x.cols + col] = iy * width + ix;
        }
    }
    return result;
}
}  // namespace

ImageAlignGPU::ImageAlignGPU() {
    try {
        cl_platform_id platform;
        cl_device_id device;
        checkStatus(clGetPlatformIDs(1, &platform, nullptr), "ImageAlign GPU platform");
        checkStatus(clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr), "ImageAlign GPU device");
        cl_int status;
        context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &status);
        checkStatus(status, "ImageAlign GPU context");
        queue = clCreateCommandQueue(context, device, 0, &status);
        checkStatus(status, "ImageAlign GPU queue");
        program = clCreateProgramWithSource(context, 1, &sourceCode, nullptr, &status);
        checkStatus(status, "ImageAlign GPU program");
        status = clBuildProgram(program, 1, &device, "", nullptr, nullptr);
        if(status != CL_SUCCESS) {
            size_t size = 0;
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &size);
            std::string log(size, '\0');
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, size, log.data(), nullptr);
            throw std::runtime_error("ImageAlign GPU build: " + log);
        }
        scatter = clCreateKernel(program, "scatter_depth", &status);
        checkStatus(status, "ImageAlign GPU scatter");
        gather = clCreateKernel(program, "gather_depth", &status);
        checkStatus(status, "ImageAlign GPU gather");
    } catch(...) {
        release();
        throw;
    }
}

void ImageAlignGPU::release() {
    if(queue) clFinish(queue);
    for(auto buffer : {source, destination, firstMap, secondMap, rayZ, depthScales, winners})
        if(buffer) clReleaseMemObject(buffer);
    if(depthLut) clReleaseMemObject(depthLut);
    if(scatter) clReleaseKernel(scatter);
    if(gather) clReleaseKernel(gather);
    if(program) clReleaseProgram(program);
    if(queue) clReleaseCommandQueue(queue);
    if(context) clReleaseContext(context);
}
ImageAlignGPU::~ImageAlignGPU() {
    release();
}

void ImageAlignGPU::configure(const cv::Mat& x1,
                              const cv::Mat& y1,
                              const cv::Mat& x2,
                              const cv::Mat& y2,
                              int inputWidth,
                              int inputHeight,
                              const std::vector<float>& rays,
                              const std::vector<float>& scales) {
    CV_Assert(x1.type() == CV_32FC1 && y1.type() == CV_32FC1 && x2.type() == CV_32FC1 && y2.type() == CV_32FC1);
    CV_Assert(!x1.empty() && !x2.empty() && x1.size() == y1.size() && x2.size() == y2.size());
    CV_Assert(x1.cols <= 65535 && x1.total() <= INT_MAX && x2.total() <= INT_MAX);
    CV_Assert(inputWidth > 0 && inputHeight > 0 && int64_t(inputWidth) * inputHeight <= INT_MAX);
    CV_Assert(rays.size() == x1.total() && scales.size() == x2.total());
    auto map1 = indices(x1, y1, inputWidth, inputHeight);
    auto map2 = indices(x2, y2, x1.cols, x1.rows);
    width = x1.cols;
    count = x1.total();
    outputCount = x2.total();
    sourceCount = inputWidth * inputHeight;
    auto buffer = [&](cl_mem& target, size_t bytes, const void* data) {
        if(target) clReleaseMemObject(target);
        target = nullptr;
        cl_int status;
        target = clCreateBuffer(context, data ? CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR : CL_MEM_READ_WRITE, bytes, const_cast<void*>(data), &status);
        checkStatus(status, "ImageAlign GPU buffer");
    };
    buffer(source, sourceCount * sizeof(uint16_t), nullptr);
    buffer(destination, outputCount * sizeof(uint16_t), nullptr);
    buffer(firstMap, map1.size() * sizeof(int), map1.data());
    buffer(secondMap, map2.size() * sizeof(int), map2.data());
    buffer(rayZ, rays.size() * sizeof(float), rays.data());
    buffer(depthScales, scales.size() * sizeof(float), scales.data());
    buffer(winners, count * sizeof(cl_uint), nullptr);
}

void ImageAlignGPU::run(const cv::Mat& input, cv::Mat output, float shift, float offset) {
    CV_Assert(input.type() == CV_16UC1 && output.type() == CV_16UC1 && input.isContinuous() && output.isContinuous());
    CV_Assert(input.total() == static_cast<size_t>(sourceCount) && output.total() == static_cast<size_t>(outputCount));
    if(!depthLut || cachedShift != shift) {
        // Match the DSP lookup and avoid GPU division rounding at half-pixel boundaries.
        std::vector<float> shifts(detail::kDepthShiftLutSize);
        detail::fillDepthShiftLut(shifts, shift);
        if(depthLut) clReleaseMemObject(depthLut);
        cl_int status;
        depthLut = clCreateBuffer(context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, shifts.size() * sizeof(float), shifts.data(), &status);
        checkStatus(status, "ImageAlign GPU shift lookup");
        cachedShift = shift;
    }
    try {
        checkStatus(clEnqueueWriteBuffer(queue, source, CL_FALSE, 0, sourceCount * sizeof(uint16_t), input.data, 0, nullptr, nullptr), "ImageAlign GPU upload");
        cl_uint zero = 0;
        checkStatus(clEnqueueFillBuffer(queue, winners, &zero, sizeof(zero), 0, count * sizeof(cl_uint), 0, nullptr, nullptr), "ImageAlign GPU clear");
        unsigned arg = 0;
        auto set = [&](cl_kernel kernel, const auto& value) { checkStatus(clSetKernelArg(kernel, arg++, sizeof(value), &value), "ImageAlign GPU argument"); };
        for(auto buffer : {source, firstMap, rayZ, depthLut, winners}) set(scatter, buffer);
        set(scatter, width);
        set(scatter, count);
        set(scatter, shift);
        size_t global = count;
        checkStatus(clEnqueueNDRangeKernel(queue, scatter, 1, nullptr, &global, nullptr, 0, nullptr, nullptr), "ImageAlign GPU scatter");
        arg = 0;
        for(auto buffer : {secondMap, winners, depthScales, destination}) set(gather, buffer);
        set(gather, outputCount);
        set(gather, offset);
        global = outputCount;
        checkStatus(clEnqueueNDRangeKernel(queue, gather, 1, nullptr, &global, nullptr, 0, nullptr, nullptr), "ImageAlign GPU gather");
        checkStatus(clEnqueueReadBuffer(queue, destination, CL_TRUE, 0, outputCount * sizeof(uint16_t), output.data, 0, nullptr, nullptr),
                    "ImageAlign GPU download");
    } catch(...) {
        clFinish(queue);
        throw;
    }
}

}  // namespace dai::gate
