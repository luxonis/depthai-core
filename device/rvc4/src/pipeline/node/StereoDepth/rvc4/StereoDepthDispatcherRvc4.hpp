#pragma once

#include <stdint.h>

#include <memory>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "pipeline/node/AccelTypes.hpp"
#include "vendor/dai_device_accel.h"

namespace dai {
class AcceleratorMemory;
}  // namespace dai

struct StereoInit {
    bool bConfOutputEnable = true;
    bool bSlicOutputEnable = false;
    uint32_t nActualFps = 30;
    uint32_t nOperationalFps = 30;
    uint32_t nWidth = 1280;
    uint32_t nHeight = 800;
    dai::ImgFrame::Type colorFormat = dai::ImgFrame::Type::GRAY8;
};

struct StereoConfig {
    explicit StereoConfig(const dai::StereoDepthConfig& cfg) : cfg(cfg) {}

    dai::StereoDepthConfig cfg;
    dai::gate::StereoSearchDir eSearchDir = dai::gate::StereoSearchDir::L2R;
};

class StereoDepthDispatcher {
   public:
    StereoDepthDispatcher();
    ~StereoDepthDispatcher();
    StereoDepthDispatcher(const StereoDepthDispatcher&) = delete;
    StereoDepthDispatcher& operator=(const StereoDepthDispatcher&) = delete;

    void RunAsync(const std::shared_ptr<const dai::ImgFrame>& inLeftFrame,
                  const std::shared_ptr<const dai::ImgFrame>& inRightFrame,
                  std::shared_ptr<dai::ImgFrame> outDispFrame,
                  std::shared_ptr<dai::ImgFrame> outConfFrame,
                  std::shared_ptr<dai::ImgFrame> outSLIC = nullptr);
    void Run(const std::shared_ptr<const dai::ImgFrame>& inLeftFrame,
             const std::shared_ptr<const dai::ImgFrame>& inRightFrame,
             std::shared_ptr<dai::ImgFrame> outDispFrame,
             std::shared_ptr<dai::ImgFrame> outConfFrame,
             std::shared_ptr<dai::ImgFrame> outSLIC = nullptr);
    int Wait();

    uint32_t Open();
    uint32_t Init(StereoInit& init);
    uint32_t Config(StereoConfig& c);
    uint32_t Close();

    uint32_t getMinDisparityImgSize() const;  // Valid only after Init
    uint32_t getMinConfImgSize() const;
    uint32_t getMinSlicImgSize() const;

   private:
    struct RunBuffers {  // Kept alive: the hardware uses them from RunAsync until Wait
        std::shared_ptr<dai::AcceleratorMemory> left, right, disparity, confidence, slic;
    };
    RunBuffers running;

    std::unique_ptr<dai_device_stereo, void (*)(dai_device_stereo*)> handle;  // Declared last: closed before the buffers above are released
};
