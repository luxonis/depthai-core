#pragma once

#include <stdint.h>

#include <memory>
#include <vector>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "pipeline/node/AccelTypes.hpp"
#include "vendor/dai_device_accel.h"

namespace dai {
class AcceleratorMemory;
}  // namespace dai

struct PyrImgInit {
    bool bOutBaseimage = false;
    uint32_t nActualFps = 30;
    uint32_t nOperationalFps = 30;
    uint32_t nWidth = 1280;
    uint32_t nHeight = 800;
    dai::ImgFrame::Type colorFormat = dai::ImgFrame::Type::GRAY8;
    uint32_t nOctaves = 2;
    uint32_t nScalesPerOctave = 1;
};

struct PyrImgConfig {
    dai::gate::ScaledownInterpolation eDsInterpolation = dai::gate::ScaledownInterpolation::BILINEAR;
};

/// The hardware image pyramid (accelerator) of the vendor module: dai_device_pyrimg_* of vendor/dai_device_accel.h
class PyrImgDispatcher {
   public:
    PyrImgDispatcher();
    ~PyrImgDispatcher();
    PyrImgDispatcher(const PyrImgDispatcher&) = delete;
    PyrImgDispatcher& operator=(const PyrImgDispatcher&) = delete;

    void RunAsync(const std::shared_ptr<const dai::ImgFrame>& inFrame, std::vector<std::shared_ptr<dai::ImgFrame>> outFrames);
    void Run(const std::shared_ptr<const dai::ImgFrame>& inFrame, std::vector<std::shared_ptr<dai::ImgFrame>> outFrames);
    int Wait();

    uint32_t Open();
    uint32_t Init(PyrImgInit& init);
    uint32_t Config(PyrImgConfig& c);
    uint32_t Close();

    /// Minimum size in bytes of the output frame buffer of a pyramid level; valid after Init
    uint32_t GetMinPyrImgBytes(int level = 1) const;

   private:
    // The accelerator buffers of the last RunAsync: the hardware uses them until Wait
    std::shared_ptr<dai::AcceleratorMemory> inMem;
    std::vector<std::shared_ptr<dai::AcceleratorMemory>> outMems;
    // Declared last: destroyed (and closed) before the buffers are released
    std::unique_ptr<dai_device_pyrimg, void (*)(dai_device_pyrimg*)> handle;
};
