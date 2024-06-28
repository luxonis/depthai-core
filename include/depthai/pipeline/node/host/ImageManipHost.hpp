#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>

#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"

namespace dai {
namespace node {

class ImageManipHost : public NodeCRTP<ThreadedHostNode, ImageManipHost> {
   public:
    constexpr static const char* NAME = "ImageManipHost";

    explicit ImageManipHost() = default;

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input inputImage{*this, {.name = "inputImage", .queueSize = 15, .types = {{DatatypeEnum::ImgFrame, false}}}};
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::ImgFrame, false}}}};

    void run() override;

    ImageManipConfigV2 initialConfig;
};
}  // namespace node

namespace impl {

class ImageManipOperations {
    std::array<std::array<float, 3>, 3> matrix{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    ImageManipOpsBase base;
    uint32_t numPlanes;

    std::shared_ptr<Buffer> trAuxFrame;

    std::shared_ptr<spdlog::async_logger> logger;

   public:
    ImageManipOperations(std::shared_ptr<spdlog::async_logger> logger = nullptr) : logger(logger) {}

    ImageManipOperations& build(const ImageManipOpsBase& base, uint32_t inputWidth, uint32_t inputHeight, uint8_t bpp);

    bool apply(const std::shared_ptr<ImgFrame> src, span<uint8_t> dst);

    std::tuple<uint32_t, uint32_t, uint8_t> getOutputSize() const;
};

}  // namespace impl
}  // namespace dai
