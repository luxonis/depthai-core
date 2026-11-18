#pragma once

#include <spdlog/spdlog.h>
#include <stdint.h>

#include <memory>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "pipeline/node/AccelTypes.hpp"

struct DsImgInit {
    uint32_t nActualFps = 30;
    uint32_t nOperationalFps = 30;
    uint32_t nMaxSrcWidth = 3840;
    uint32_t nMaxSrcHeight = 2160;
    uint32_t nMinDstWidth = 64;
    uint32_t nMinDstHeight = 64;
    dai::ImgFrame::Type colorFormat = dai::ImgFrame::Type::GRAY8;
};

struct DsImgConfig {
    dai::gate::ScaledownInterpolation eDsInterpolation = dai::gate::ScaledownInterpolation::BILINEAR;
};

class DsImgDispatcher {
   public:
    DsImgDispatcher();
    ~DsImgDispatcher();
    DsImgDispatcher(const DsImgDispatcher&) = delete;
    DsImgDispatcher& operator=(const DsImgDispatcher&) = delete;

    void startSession();
    void RunAsync(std::shared_ptr<dai::ImgFrame> inFrame, std::shared_ptr<dai::ImgFrame> outFrame, bool useMetadata = false);
    void Run(std::shared_ptr<dai::ImgFrame> inFrame, std::shared_ptr<dai::ImgFrame> outFrame, bool useMetadata = false);
    int Wait();

    void registerNode();
    void firstFrameOver();
    uint32_t Open();
    uint32_t Init(DsImgInit& init);
    uint32_t Config(DsImgConfig& c);
    uint32_t Close();

    uint32_t GetInMinDsImgBytes(int width, int height);
    uint32_t GetOutMinDsImgBytes(int width, int height);
    void PrintFrameInfo(int width, int height, spdlog::logger& logger);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
