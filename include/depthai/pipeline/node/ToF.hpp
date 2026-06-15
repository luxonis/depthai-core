#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/ImageFilters.hpp>

// standard
#include <memory>
#include <optional>
#include <utility>

// shared
#include <depthai/properties/ToFProperties.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ToFPreset.hpp"
#include "depthai/common/ToFSensorMode.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {

struct ToFBuildOptions {
    CameraBoardSocket boardSocket = CameraBoardSocket::AUTO;
    std::optional<ToFSensorMode> sensorMode;
    /// Integer FPS stored as float (matches ToFProperties/CameraProperties wire format).
    std::optional<float> fps;
    /// RVC4 IPP preset. Applied only when set; tune initialConfig after build() for further overrides.
    std::optional<ToFPreset> preset;
};

namespace node {

class Camera;

/**
 * @brief ToFBase node.
 * Device-side ToF depth processing (decoder on RVC2, IPP on RVC4).
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
     * Initial ToF config (IPP on RVC4, decoder on RVC2).
     */
    std::shared_ptr<ToFConfig> initialConfig = std::make_shared<ToFConfig>();

    /**
     * Input ToFConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this,
                      {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ToFConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for raw sensor frames (e.g. RAW12 superframes from VD55H1 ToF sensor).
     * When connected, the node processes incoming raw frames instead of self-capturing.
     * Default queue is blocking with size 8.
     */
    Input rawInput{*this, {"rawInput", DEFAULT_GROUP, true, 8, {{{DatatypeEnum::ImgFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    Output depth{*this, {"depth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    Output amplitude{*this, {"amplitude", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output intensity{*this, {"intensity", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output confidence{*this, {"confidence", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output phase{*this, {"phase", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output raw{*this, {"raw", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Build with a specific board socket (RVC2 path with host ImageFilters preset).
     */
    std::shared_ptr<ToFBase> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO,
                                   dai::ImageFiltersPresetMode presetMode = dai::ImageFiltersPresetMode::TOF_MID_RANGE,
                                   std::optional<float> fps = std::nullopt);

    /**
     * Build with board socket only (RVC4 path; IPP preset applied separately).
     */
    std::shared_ptr<ToFBase> build(dai::CameraBoardSocket boardSocket, std::optional<float> fps);

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
    std::shared_ptr<ToFBase> finalizeBuild(dai::CameraBoardSocket boardSocket, std::optional<float> fps);

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
    void buildStage1() override;
    void buildStage2() override;

    std::shared_ptr<ToF> build(const ToFBuildOptions& options);
    std::shared_ptr<ToF> build(dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::AUTO,
                               dai::ImageFiltersPresetMode presetMode = dai::ImageFiltersPresetMode::TOF_MID_RANGE,
                               std::optional<float> fps = std::nullopt);

    /**
     * Returns auto-created Camera on RVC4, or nullptr when rawInput was user-connected.
     */
    std::shared_ptr<Camera> getCamera() const;

    CameraBoardSocket getBoardSocket() const;

    /**
     * Depth / amplitude / confidence output size (width, height) after IPP and landscape transpose.
     * RVC4 only — derived from the selected ToFSensorMode. On RVC2, read dimensions from depth frames.
     */
    std::pair<uint32_t, uint32_t> getOutputResolution() const;

    /**
     * Raw VD55H1 superframe size (width, height) for Camera.build(sensorResolution=...).
     * RVC4 only — matches the ToFSensorMode passed to build().
     */
    std::pair<uint32_t, uint32_t> getRawResolution() const;

    /**
     * @deprecated Use getOutputResolution() — this name referred to sensor size but returns IPP output size.
     */
    std::pair<uint32_t, uint32_t> getSensorResolution() const;

    Subnode<ToFBase> tofBase{*this, "tofBase"};
    Subnode<ImageFilters> imageFilters{*this, "imageFilters"};

    /**
     * Initial ToF config (IPP on RVC4, decoder on RVC2). Unified accessor over the base node;
     * prefer this over tofBaseNode.initialConfig in application code.
     */
    std::shared_ptr<ToFConfig>& initialConfig;

    /**
     * Raw depth output from ToF sensor
     */
    Output& rawDepth;

    /**
     * Filtered depth output (RVC2) or IPP depth output (RVC4)
     */
    Output& depth;

    /**
     * Amplitude output
     */
    Output& amplitude;

    /**
     * Intensity output
     */
    Output& intensity;

    /**
     * Confidence output (RVC4). On RVC2, aliases amplitude as a per-pixel quality map.
     */
    Output& confidence;

    /**
     * Phase output
     */
    Output& phase;

    /**
     * Input for raw sensor frames
     */
    Input& rawInput;

    /**
     * Raw data coming from the sensor
     */
    Output& raw;

    /**
     * Runtime ToFConfig input for the ToF base node (decoder on RVC2, IPP on RVC4).
     *
     * RVC4: directly reconfigures the IPP that produces `depth`.
     * RVC2: reconfigures the decoder that feeds the host ImageFilters. The host-filter
     *       stage that actually produces `depth` is NOT controlled here — to retune the
     *       host filters at runtime send an ImageFiltersConfig to `imageFiltersInputConfig`.
     */
    Input& inputConfig;

    /**
     * Alias of `inputConfig` (the ToF base / decoder config input), kept for naming
     * symmetry with `imageFiltersInputConfig`.
     */
    Input& tofBaseInputConfig;

    /**
     * Runtime ImageFiltersConfig input for the host ImageFilters node (RVC2 only).
     *
     * On RVC2 this is the stage that produces `depth`; use it (not `inputConfig`) to
     * change host-side filtering at runtime. Unused on RVC4 (no host ImageFilters).
     */
    Input& imageFiltersInputConfig;

    /**
     * ToF base node
     */
    ToFBase& tofBaseNode;

    /**
     * Image filters node
     */
    ImageFilters& imageFiltersNode;

   private:
    void logWarnOnce(bool& warned, const char* message) const;
    void validateBuildFps(const std::optional<float>& fps);
    void warnIfMisconfiguredInitialConfig() const;
    void warnIfPhaseOrRawConnected() const;

    bool isRvc4Platform;
    bool isGroupBuilt = false;
    ToFBuildOptions buildOptions;
    std::shared_ptr<Camera> autoCamera;

    mutable bool warnedDecoderFieldsRvc4 = false;
    mutable bool warnedIppFieldsRvc2 = false;
    mutable bool warnedPhaseRawRvc4 = false;
    mutable bool warnedFpsRangeRvc4 = false;
    mutable bool warnedSensorResolutionDeprecated = false;
};

}  // namespace node
}  // namespace dai
