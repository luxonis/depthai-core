#pragma once

#include <stdint.h>

#include <depthai/properties/WarpProperties.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "vendor/dai_device_accel.h"

namespace dai {
class AcceleratorMemory;
}  // namespace dai

class WarpDispatcher {
   public:
    WarpDispatcher();
    ~WarpDispatcher();
    WarpDispatcher(const WarpDispatcher&) = delete;
    WarpDispatcher& operator=(const WarpDispatcher&) = delete;

    void Run(const std::shared_ptr<const dai::ImgFrame>& inputFrame, std::shared_ptr<dai::ImgFrame> outputFrame, size_t inputIndex = 0);
    void RunAsync(const std::shared_ptr<const dai::ImgFrame>& inputFrame, std::shared_ptr<dai::ImgFrame> outputFrame, size_t inputIndex = 0);
    int Wait();

    uint32_t Open();
    uint32_t Init(dai::WarpProperties config, uint32_t inWidth, uint32_t inHeight, dai::ImgFrame::Type inClrFmt, dai::ImgFrame::Type outClrFmt);
    uint32_t Close();

    uint32_t getMinOutImgSize() const;  // Minimum output frame buffer size in bytes; valid after Init

    void SetWarpMesh(std::vector<double>& rawGridX, std::vector<double>& rawGridY, size_t inputIndex = 0);
    void SetWarpMesh(cv::Mat mesh_x, cv::Mat mesh_y, size_t inputIndex = 0);
    static void ParseMeshResourceData(std::vector<uint8_t>& data, int width, int height, std::vector<double>& rawGridX, std::vector<double>& rawGridY);

   private:
    std::shared_ptr<dai::AcceleratorMemory> runningInput, runningOutput;  // Buffers of the last RunAsync; they stay valid until Wait

    std::unique_ptr<dai_device_warp, void (*)(dai_device_warp*)> handle;  // Declared last: destroyed (and closed) before the buffers
};
