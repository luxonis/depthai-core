/**
 * RVC4 ToF example: build(), initialConfig tuning, and runtime inputConfig updates.
 */

#include <iostream>
#include <memory>

#include "depthai/common/ToFPreset.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/node/ToF.hpp"

int main() {
    dai::Pipeline pipeline;

    auto tof = pipeline.create<dai::node::ToF>();

    dai::ToFBuildOptions options;
    options.boardSocket = dai::CameraBoardSocket::AUTO;
    options.fps = 10.f;  // VD55H1 capture mode fixed to F3_FULL internally
    options.preset = dai::ToFPreset::MID_RANGE;
    tof->build(options);

    // Tune IPP fields after build() so build(preset=...) does not overwrite them.
    auto& cfg = *tof->initialConfig;
    cfg.enableBilateralFilter = true;
    cfg.enableTemporalNoiseReduction = true;
    cfg.enableFlyingPixelCorrection = true;
    cfg.enableRadialToPerp = false;
    cfg.phaseUnwrapErrorThreshold = 75;

    const auto depthQueue = tof->depth.createOutputQueue();
    const auto cfgQueue = tof->inputConfig.createInputQueue();

    pipeline.start();

    const auto [outW, outH] = tof->getOutputResolution();
    const auto [rawW, rawH] = tof->getRawResolution();
    std::cout << "socket=" << static_cast<int>(tof->getBoardSocket()) << " output=" << outW << "x" << outH << " raw=" << rawW << "x" << rawH
              << " auto_cam=" << (tof->getCamera() != nullptr) << std::endl;

    if(const auto depth = depthQueue->get<dai::ImgFrame>()) {
        std::cout << "depth frame: " << depth->getWidth() << "x" << depth->getHeight() << std::endl;
    }

    cfg.enableRadialToPerp = true;
    cfgQueue->send(std::make_shared<dai::ToFConfig>(cfg));

    if(const auto depth2 = depthQueue->get<dai::ImgFrame>()) {
        std::cout << "depth frame after runtime config: " << depth2->getWidth() << "x" << depth2->getHeight() << std::endl;
    }

    return 0;
}
