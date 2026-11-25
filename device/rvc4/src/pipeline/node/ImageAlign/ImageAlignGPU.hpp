#pragma once

#include <CL/cl.h>

#include <opencv2/core.hpp>
#include <vector>

namespace dai::gate {

// RAW16 path: fuse the first remap with the shift, and the second with axial-Z conversion.
class ImageAlignGPU {
   public:
    ImageAlignGPU();
    ~ImageAlignGPU();
    ImageAlignGPU(const ImageAlignGPU&) = delete;
    ImageAlignGPU& operator=(const ImageAlignGPU&) = delete;
    void configure(const cv::Mat& x1,
                   const cv::Mat& y1,
                   const cv::Mat& x2,
                   const cv::Mat& y2,
                   int inputWidth,
                   int inputHeight,
                   const float* rays,
                   const std::vector<float>& scales);
    void run(const cv::Mat& input, cv::Mat output, float shift, float offset);

   private:
    void release();
    cl_context context = nullptr;
    cl_command_queue queue = nullptr;
    cl_program program = nullptr;
    cl_kernel scatter = nullptr, gather = nullptr;
    cl_mem depthLut = nullptr;
    float cachedShift = 0;
    cl_mem source = nullptr, destination = nullptr, firstMap = nullptr, secondMap = nullptr;
    cl_mem rayZ = nullptr, depthScales = nullptr, winners = nullptr;
    int width = 0, count = 0, outputCount = 0, sourceCount = 0;
};
}  // namespace dai::gate
