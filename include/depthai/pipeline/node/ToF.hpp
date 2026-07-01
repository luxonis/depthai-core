#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Camera.hpp>
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

    /**
     * Input for raw sensor frames used by the RVC4 host implementation.
     * This stays internal to the node group API, but must remain on the base
     * node so the auto-created ToF camera can be linked in the pipeline schema.
     */
    Input rawInput{*this, {"rawInput", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    Output amplitude{*this, {"amplitude", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output intensity{*this, {"intensity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output confidence{*this, {"confidence", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output phase{*this, {"phase", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output raw{*this, {"raw", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Build with a specific board socket
     */
    std::shared_ptr<ToFBase> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO,
                                   ToFConfig::Profile profile = dai::ToFConfig::Profile::MID_RANGE,
                                   std::optional<float> fps = std::nullopt);

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
    ToF(const std::shared_ptr<Device>& device);

    ~ToF() override;

    [[nodiscard]] static std::shared_ptr<ToF> create(const std::shared_ptr<Device>& device) {
        auto tofPtr = std::make_shared<ToF>(device);
        tofPtr->buildInternal();
        return tofPtr;
    }

    void buildInternal() override;

    /**
     * Build the ToF node with a specific board socket, legacy preset mode, and optional FPS.
     * @param boardSocket Board socket to use, or AUTO to select an available ToF socket automatically
     * @param presetMode Legacy ToF image filter preset mode
     * @param fps Requested ToF camera FPS
     */
    [[deprecated("Use build(boardSocket, dai::ToFConfig::Profile, fps) instead")]] std::shared_ptr<ToF> build(
        dai::CameraBoardSocket boardSocket, dai::ImageFiltersPresetMode presetMode, std::optional<float> fps);

    /**
     * Build the ToF node with a specific board socket, profile, and optional FPS.
     * @param boardSocket Board socket to use, or AUTO to select an available ToF socket automatically
     * @param presetMode ToF processing profile to apply
     * @param fps Requested ToF camera FPS
     */
    std::shared_ptr<ToF> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO,
                               dai::ToFConfig::Profile presetMode = dai::ToFConfig::Profile::MID_RANGE,
                               std::optional<float> fps = std::nullopt);

    Subnode<ToFBase> tofBase{*this, "tofBase"};
    void postBuildStage() override;

   private:
    std::unique_ptr<Subnode<ImageFilters>> imageFilters = nullptr;
    std::unique_ptr<Subnode<Camera>> autoCamera = nullptr;

   public:

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    /**
     * Raw depth output from ToF sensor
     */
    Output& rawDepth{tofBase->depth};

    /**
     * Filtered depth output
     */
    Output& depth;

    /**
     * Amplitude output
     */
    Output& amplitude{tofBase->amplitude};

    /**
     * Intensity output
     */
    Output& intensity{tofBase->intensity};

    /**
     * Confidence output
     */
    Output& confidence{tofBase->confidence};

    /**
     * Phase output
     */
    Output& phase{tofBase->phase};

    /**
     * Raw data coming from the sensor
     */
    Output& raw{tofBase->raw};

    /** Runtime ToFConfig input for the ToF base node (decoder on RVC2, IPP on RVC4).
     *
     * RVC4: directly reconfigures the IPP that produces `depth`.
     * RVC2: reconfigures the decoder that feeds the host ImageFilters. The host-filter
     *       stage that actually produces `depth` is NOT controlled here — to retune the
     *       host filters at runtime send an ImageFiltersConfig to `imageFiltersInputConfig`.
     */
    Input& inputConfig{tofBase->inputConfig};

    /**
     * Input config for ToF base node
     */
    Input& tofBaseInputConfig{tofBase->inputConfig};

    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * Input config for image filters
     */
    Input* imageFiltersInputConfig = nullptr;
    #endif
#endif

    /**
     * ToF base node
     */
    ToFBase& tofBaseNode{*tofBase};

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * Image filters node
     */
    ImageFilters* imageFiltersNode = nullptr;
#endif
};

}  // namespace node
}  // namespace dai
