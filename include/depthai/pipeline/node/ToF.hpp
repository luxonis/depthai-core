#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/ImageFilters.hpp>

// standard
#include <fstream>

// shared
#include <depthai/properties/ToFProperties.hpp>

#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief ToFBase node.
 * Performs feature tracking and reidentification using motion estimation between 2 consecutive frames.
 */
class ToFBase : public DeviceNodeCRTP<DeviceNode, ToFBase, ToFProperties> {
   public:
    constexpr static const char* NAME = "ToF";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();

   public:
    ToFBase() = default;
    ToFBase(std::unique_ptr<Properties> props);

    /**
     * Initial config to use for feature tracking.
     */
    std::shared_ptr<ToFConfig> initialConfig = std::make_shared<ToFConfig>();

    /**
     * Input ToFConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this,
                      {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ToFConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    Output amplitude{*this, {"amplitude", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output intensity{*this, {"intensity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output phase{*this, {"phase", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Build with a specific board socket
     */
    std::shared_ptr<ToFBase> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO,
                                   dai::ImageFiltersPresetMode presetMode = dai::ImageFiltersPresetMode::TOF_MID_RANGE,
                                   std::optional<float> fps = std::nullopt);

    /**
     * Set profile preset for ToFConfig
     * @param presetMode Preset mode for ToFConfig
     */
    void setProfilePreset(dai::ImageFiltersPresetMode presetMode) {
        initialConfig->setProfilePreset(presetMode);
    }

    /**
     * Retrieves which board socket to use
     * @returns Board socket to use
     */
    CameraBoardSocket getBoardSocket() const;

   private:
    bool isBuilt = false;
    uint32_t maxWidth = 0;
    uint32_t maxHeight = 0;
};

class ToF : public DeviceNodeGroup {
   public:
    ToF(const std::shared_ptr<Device>& device)
        : DeviceNodeGroup(device),
          rawDepth{tofBase->depth},
          depth{imageFilters->output},
          confidence{tofDepthConfidenceFilter->confidence},
          amplitude{tofBase->amplitude},
          intensity{tofBase->intensity},
          phase{tofBase->phase},
          tofBaseInputConfig{tofBase->inputConfig},
          tofDepthConfidenceFilterInputConfig{tofDepthConfidenceFilter->inputConfig},
          imageFiltersInputConfig{imageFilters->inputConfig},
          tofBaseNode{*tofBase},
          tofDepthConfidenceFilterNode{*tofDepthConfidenceFilter},
          imageFiltersNode{*imageFilters} {}

    [[nodiscard]] static std::shared_ptr<ToF> create(const std::shared_ptr<Device>& device) {
        auto tofPtr = std::make_shared<ToF>(device);
        tofPtr->buildInternal();
        return tofPtr;
    }

    void buildInternal() override {
        // Build all subnodes, call their internal build functions
        tofBase->buildInternal();
        tofDepthConfidenceFilter->buildInternal();
        imageFilters->buildInternal();

        // Link subnodes together
        tofBase->depth.link(tofDepthConfidenceFilter->depth);
        tofBase->amplitude.link(tofDepthConfidenceFilter->amplitude);

        tofDepthConfidenceFilter->filteredDepth.link(imageFilters->input);

        // Important note:
        // imageFilters->output and depth are implicitly linked via the reference
        // tofDepthConfidenceFilter->confidence and confidence are implicitly linked via the reference
        // the same goes for the input configs: tofBaseInputConfig, tofDepthConfidenceFilterInputConfig, imageFiltersInputConfig
    }

    std::shared_ptr<ToF> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO,
                               dai::ImageFiltersPresetMode presetMode = dai::ImageFiltersPresetMode::TOF_MID_RANGE,
                               std::optional<float> fps = std::nullopt) {
        tofBase->build(boardSocket, presetMode, fps);
        tofDepthConfidenceFilter->build(presetMode);
        imageFilters->build(presetMode);
        return std::static_pointer_cast<ToF>(shared_from_this());
    }

    Subnode<ToFBase> tofBase{*this, "tofBase"};
    Subnode<ToFDepthConfidenceFilter> tofDepthConfidenceFilter{*this, "tofDepthConfidenceFilter"};
    Subnode<ImageFilters> imageFilters{*this, "imageFilters"};

    /**
     * Raw depth output from ToF sensor
     */
    Output& rawDepth;

    /**
     * Filtered depth output
     */
    Output& depth;

    /**
     * Confidence output
     */
    Output& confidence;

    /**
     * Amplitude output
     */
    Output& amplitude;

    /**
     * Intensity output
     */
    Output& intensity;

    /**
     * Phase output
     */
    Output& phase;

    /**
     * Input config for ToF base node
     */
    Input& tofBaseInputConfig;

    /**
     * Input config for ToF depth confidence filter
     */
    Input& tofDepthConfidenceFilterInputConfig;

    /**
     * Input config for image filters
     */
    Input& imageFiltersInputConfig;

    /**
     * ToF base node
     */
    ToFBase& tofBaseNode;

    /**
     * ToF depth confidence filter node
     */
    ToFDepthConfidenceFilter& tofDepthConfidenceFilterNode;

    /**
     * Image filters node
     */
    ImageFilters& imageFiltersNode;
};

}  // namespace node
}  // namespace dai
