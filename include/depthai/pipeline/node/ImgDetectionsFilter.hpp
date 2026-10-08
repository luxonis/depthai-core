#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/properties/ImgDetectionsFilterProperties.hpp"

namespace dai {
namespace node {

/**
 * Filter detections or combine already synchronized camera detections into a reference image.
 * Linked keys are consumed once per round in lexicographic order. Use Sync and MessageDemux upstream.
 * Multiple inputs run on the host. One input runs on RVC4, or on the host for RVC2 and device-free pipelines.
 * Remapping uses calibration and rotation without depth or camera translation. For panorama alignment,
 * use Stitching PANORAMA with setUseInputCalibration(true); full 360 degree seam-crossing boxes are unsupported.
 */
class ImgDetectionsFilter : public DeviceNodeCRTP<DeviceNode, ImgDetectionsFilter, ImgDetectionsFilterProperties>, public HostRunnable {
   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "ImgDetectionsFilter";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    /** Construct from serialized properties (also used by the device runtime). */
    explicit ImgDetectionsFilter(std::unique_ptr<Properties> props);
    ~ImgDetectionsFilter() override;

    /** Initial configuration; runtime configs replace all filter options. */
    std::shared_ptr<ImgDetectionsFilterConfig> initialConfig = std::make_shared<ImgDetectionsFilterConfig>();
    /** Synchronized ImgDetections inputs. Only linked keys participate; keys are fixed at pipeline start. */
    InputMap inputs{*this, "inputs", {"", DEFAULT_GROUP, true, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgDetections, false}}}, true}};
    /** Optional reference frame. Pixels are ignored; the newest valid transformation is latched. */
    Input inputReference{*this, {"inputReference", DEFAULT_GROUP, false, 1, {{{DatatypeEnum::ImgFrame, false}}}, false}};
    /** Optional runtime config. The last queued message is checked once a round has been collected. */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgDetectionsFilterConfig, false}}}, false}};
    /** One ImgDetections per round, with metadata from the newest host timestamp (earlier key wins ties). */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /** Override automatic placement. Device execution with multiple linked keys is rejected. */
    ImgDetectionsFilter& setRunOnHost(bool runOnHost);
    /** Return the selected execution placement. RVC2 and device-free pipelines always run on the host. */
    bool runOnHost() const override;
    /** Validate configuration, linked inputs and reference availability before starting. */
    void buildStage1() override;
    /** Process rounds with the current configuration and latched reference. */
    void run() override;

   private:
    std::optional<bool> runOnHostVar;
    std::vector<std::pair<std::string, Input*>> linkedInputs;
};
}  // namespace node
}  // namespace dai
