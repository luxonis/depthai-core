#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/TofFusionConfig.hpp>
#include <depthai/properties/TofFusionProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief TofFusion node. Fuses ToF depth with neural depth using texture-guided weighted blending.
 * RVC4 only. Direct inputs/outputs, no subnodes.
 */
class TofFusion : public DeviceNodeCRTP<DeviceNode, TofFusion, TofFusionProperties> {
   public:
    constexpr static const char* NAME = "TofFusion";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties() override;

   public:
    TofFusion() = default;
    TofFusion(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for TofFusion.
     */
    std::shared_ptr<TofFusionConfig> initialConfig = std::make_shared<TofFusionConfig>();

    /**
     * Input config to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this,
                      {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::TofFusionConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input: left texture (grayscale float32 or uint8 ImgFrame, used as texture strength signal).
     */
    Input inputTexture{*this, {"inputTexture", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Input: ToF depth (float32 meters, ImgFrame).
     */
    Input inputTofDepth{*this, {"inputTofDepth", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Input: ToF confidence (float32 [0,1], ImgFrame).
     */
    Input inputTofConfidence{*this, {"inputTofConfidence", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Input: Neural depth (float32 meters, ImgFrame).
     */
    Input inputNeuralDepth{*this, {"inputNeuralDepth", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Input: Neural confidence (float32 [0,1], ImgFrame).
     */
    Input inputNeuralConfidence{*this, {"inputNeuralConfidence", DEFAULT_GROUP, true, 4, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output: Fused depth (float32 meters, ImgFrame).
     */
    Output fusedDepth{*this, {"fusedDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
};

}  // namespace node
}  // namespace dai
