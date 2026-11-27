#include "DeblurOpenCL.hpp"

#include <CL/cl.h>

#include <array>
#include <stdexcept>
#include <string>
#include <vector>
namespace dai {
namespace gate {
namespace deblur {
namespace {
void check(cl_int status, const char* operation) {
    if(status != CL_SUCCESS) throw std::runtime_error(std::string("Deblur OpenCL ") + operation + ": " + std::to_string(status));
}
template <typename T, cl_int (*Release)(T)>
struct Handle {
    T value = nullptr;
    Handle() = default;
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    ~Handle() {
        reset();
    }
    void reset(T next = nullptr) {
        if(value) Release(value);
        value = next;
    }
};
constexpr const char* SOURCE = R"CLC(
int reflect101(int p,int n) {
    while(p<0 || p>=n) { if(p<0)p=-p; if(p>=n)p=2*n-2-p; }
    return p;
}
__kernel void clustered_filter(__global const float* src,__global const int* labels,
                              __global const float* filters,__global float* dst,int width,int height,int size) {
    int id=get_global_id(0); if(id>=width*height)return;
    int label=labels[id]; if(label<0){dst[id]=src[id];return;}
    int x=id%width,y=id/width,r=size/2;float value=0;
    for(int ky=0;ky<size;++ky) for(int kx=0;kx<size;++kx) {
        int sy=reflect101(y+ky-r,height),sx=reflect101(x+kx-r,width);
        value+=filters[label*size*size+ky*size+kx]*src[sy*width+sx];
    }
    dst[id]=value;
}
)CLC";
}  // namespace
struct OpenCLFilter::Impl {
    Handle<cl_context, clReleaseContext> context;
    Handle<cl_command_queue, clReleaseCommandQueue> queue;
    Handle<cl_program, clReleaseProgram> program;
    Handle<cl_kernel, clReleaseKernel> kernel;
    std::array<Handle<cl_mem, clReleaseMemObject>, 4> buffers;
    std::array<std::size_t, 4> sizes{};
    Impl() {
        cl_uint count = 0;
        check(clGetPlatformIDs(0, nullptr, &count), "platform count");
        std::vector<cl_platform_id> platforms(count);
        if(!count) throw std::runtime_error("Deblur: no OpenCL platform");
        check(clGetPlatformIDs(count, platforms.data(), nullptr), "platforms");
        cl_device_id device = nullptr;
        for(const auto platform : platforms)
            if(clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr) == CL_SUCCESS) break;
        if(!device) throw std::runtime_error("Deblur: no OpenCL GPU");
        cl_int status = 0;
        context.reset(clCreateContext(nullptr, 1, &device, nullptr, nullptr, &status));
        check(status, "context");
#if CL_TARGET_OPENCL_VERSION >= 200
        queue.reset(clCreateCommandQueueWithProperties(context.value, device, nullptr, &status));
#else
        queue.reset(clCreateCommandQueue(context.value, device, 0, &status));
#endif
        check(status, "queue");
        const char* source = SOURCE;
        program.reset(clCreateProgramWithSource(context.value, 1, &source, nullptr, &status));
        check(status, "program");
        status = clBuildProgram(program.value, 1, &device, "-cl-std=CL1.2", nullptr, nullptr);
        if(status != CL_SUCCESS) {
            std::size_t bytes = 0;
            clGetProgramBuildInfo(program.value, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &bytes);
            std::string log(bytes, '\0');
            if(bytes) clGetProgramBuildInfo(program.value, device, CL_PROGRAM_BUILD_LOG, bytes, log.data(), nullptr);
            throw std::runtime_error("Deblur OpenCL build failed: " + log);
        }
        kernel.reset(clCreateKernel(program.value, "clustered_filter", &status));
        check(status, "kernel");
    }
    void allocate(std::size_t index, std::size_t bytes) {
        if(sizes[index] == bytes) return;
        cl_int status = 0;
        buffers[index].reset(clCreateBuffer(context.value, CL_MEM_READ_WRITE, bytes, nullptr, &status));
        check(status, "buffer");
        sizes[index] = bytes;
    }
};
OpenCLFilter::OpenCLFilter() : impl(std::make_unique<Impl>()) {}
OpenCLFilter::~OpenCLFilter() = default;
cv::Mat OpenCLFilter::apply(const Field& field) {
    if(field.kernels.empty()) return field.image.clone();
    const int size = field.kernels.front().rows;
    std::vector<float> filters;
    filters.reserve(field.kernels.size() * size * size);
    for(const auto& kernel : field.kernels) filters.insert(filters.end(), kernel.ptr<float>(), kernel.ptr<float>() + kernel.total());
    const std::size_t pixels = field.image.total(), bytes = pixels * sizeof(float);
    impl->allocate(0, bytes);
    impl->allocate(1, pixels * sizeof(int));
    impl->allocate(2, filters.size() * sizeof(float));
    impl->allocate(3, bytes);
    const std::array<const void*, 3> data{field.image.data, field.labels.data, filters.data()};
    for(std::size_t i = 0; i < data.size(); ++i)
        check(clEnqueueWriteBuffer(impl->queue.value, impl->buffers[i].value, CL_TRUE, 0, impl->sizes[i], data[i], 0, nullptr, nullptr), "upload");
    for(cl_uint i = 0; i < 4; ++i) check(clSetKernelArg(impl->kernel.value, i, sizeof(cl_mem), &impl->buffers[i].value), "buffer argument");
    const std::array<int, 3> dimensions{field.image.cols, field.image.rows, size};
    for(cl_uint i = 0; i < 3; ++i) check(clSetKernelArg(impl->kernel.value, i + 4, sizeof(int), &dimensions[i]), "dimension argument");
    check(clEnqueueNDRangeKernel(impl->queue.value, impl->kernel.value, 1, nullptr, &pixels, nullptr, 0, nullptr, nullptr), "dispatch");
    cv::Mat output(field.image.size(), CV_32F);
    check(clEnqueueReadBuffer(impl->queue.value, impl->buffers[3].value, CL_TRUE, 0, bytes, output.data, 0, nullptr, nullptr), "download");
    return output;
}
}  // namespace deblur
}  // namespace gate
}  // namespace dai
