#pragma once

#include <spdlog/async_logger.h>

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageManipConfigV2.hpp>

// shared
#include <depthai/properties/ImageManipPropertiesV2.hpp>
#include <functional>

namespace dai {
namespace node {

/**
 * @brief ImageManip node. Capability to crop, resize, warp, ... incoming image frames
 */
class ImageManipV2 : public DeviceNodeCRTP<DeviceNode, ImageManipV2, ImageManipPropertiesV2> {
   private:
    bool runOnHostVar = false;

   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "ImageManipV2";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    ImageManipV2() = default;
    ImageManipV2(std::unique_ptr<Properties> props);

    std::shared_ptr<ImageManipV2> build() {
        return std::static_pointer_cast<ImageManipV2>(shared_from_this());
    }
    /**
     * Initial config to use when manipulating frames
     */
    ImageManipConfigV2 initialConfig;

    /**
     * Input ImageManipConfigV2 message with ability to modify parameters in runtime
     */
    Input inputConfig{*this, {.name = "inputConfig", .types = {{DatatypeEnum::ImageManipConfigV2, true}}, .waitForMessage = false}};

    /**
     * Input image to be modified
     */
    Input inputImage{*this, {.name = "inputImage", .types = {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Outputs ImgFrame message that carries modified image.
     */
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Specify maximum size of output image.
     * @param maxFrameSize Maximum frame size in bytes
     */
    void setMaxOutputFrameSize(int maxFrameSize);

    /**
     * Specify whether to run on host or device
     * @param runOnHost Run node on host
     */
    ImageManipV2& setRunOnHost(bool runOnHost = true);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

    template <typename N, template <typename T> typename ImageManipBuffer, typename ImageManipData>
    static void loop(N& node,
              const ImageManipConfigV2& initialConfig,
              std::shared_ptr<spdlog::async_logger> logger,
              std::function<size_t(const ImageManipConfigV2&, const ImgFrame&)> build,
              std::function<bool(std::shared_ptr<Memory>&, span<uint8_t>)> apply,
              std::function<void(const ImageManipConfigV2&, const ImgFrame&, ImgFrame&)> getFrame);
};

}  // namespace node
}  // namespace dai

namespace dai {
namespace node {

template <typename N, template <typename T> typename ImageManipBuffer, typename ImageManipData>
void ImageManipV2::loop(N& node,
                        const ImageManipConfigV2& initialConfig,
                        std::shared_ptr<spdlog::async_logger> logger,
                        std::function<size_t(const ImageManipConfigV2&, const ImgFrame&)> build,
                        std::function<bool(std::shared_ptr<Memory>&, span<uint8_t>)> apply,
                        std::function<void(const ImageManipConfigV2&, const ImgFrame&, ImgFrame&)> getFrame) {
    using namespace std::chrono;
    auto config = initialConfig;

    std::shared_ptr<ImgFrame> inImage;

    while(node.isRunning()) {
        std::shared_ptr<ImageManipConfigV2> pConfig;
        std::shared_ptr<Memory> inImageData;
        bool hasConfig = false;
        bool needsImage = true;
        bool skipImage = false;
        if(node.inputConfig.getWaitForMessage()) {
            std::cout << "trying to receive config..." << std::endl;

            pConfig = node.inputConfig.template get<ImageManipConfigV2>();
            hasConfig = true;
            if(inImage != nullptr && hasConfig && pConfig->getReusePreviousImage()) {
                needsImage = false;
            }
            skipImage = pConfig->getSkipCurrentImage();
        } else {
            pConfig = node.inputConfig.template tryGet<ImageManipConfigV2>();
            if(pConfig != nullptr) {
                hasConfig = true;
            }
        }

        if(needsImage) {
            inImage = node.inputImage.template get<ImgFrame>();
            inImageData = inImage->data;
            if(!hasConfig) {
                auto _pConfig = node.inputConfig.template tryGet<ImageManipConfigV2>();
                if(_pConfig != nullptr) {
                    pConfig = _pConfig;
                    hasConfig = true;
                }
            }
            if(skipImage) {
                continue;
            }
        }

        // if has new config, parse and check if any changes
        if(hasConfig) {
            config = *pConfig;
        }

        auto startP = std::chrono::steady_clock::now();

        auto t1 = steady_clock::now();
        auto outputSize = build(config, *inImage);
        auto t2 = steady_clock::now();

        // Check the output image size requirements, and check whether pool has the size required
        if(outputSize == 0) {
            node.out.send(inImage);
        } else if((long)outputSize <= (long)node.properties.outputFrameSize) {
            auto outImage = std::make_shared<ImgFrame>();
            outImage->setData(std::vector<uint8_t>(node.properties.outputFrameSize));

            bool success = true;
            {
                auto t3 = steady_clock::now();
                success = apply(inImageData, outImage->getData());
                auto t4 = steady_clock::now();

                getFrame(config, *inImage, *outImage);

                logger->trace("Build time: {}us, Process time: {}us, Total time: {}us, image manip id: {}",
                              duration_cast<microseconds>(t2 - t1).count(),
                              duration_cast<microseconds>(t4 - t3).count(),
                              duration_cast<microseconds>(t4 - t1).count(),
                              node.id);
            }
            if(!success) {
                logger->error("Processing failed, potentially unsupported config");
            }
            node.out.send(outImage);
        } else {
            logger->error(
                "Output image is bigger ({}B) than maximum frame size specified in properties ({}B) - skipping frame.\nPlease use the setMaxOutputFrameSize "
                "API to explicitly config the [maximum] output size.",
                outputSize,
                node.properties.outputFrameSize);
        }

        // Update previousConfig of preprocessor, to be able to check if it needs to be updated
        auto loopNanos = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - startP).count();
        logger->trace("ImageManip | total process took {}ns ({}ms)", loopNanos, (double)loopNanos / 1e6);
    }
}

}  // namespace node
}  // namespace dai
