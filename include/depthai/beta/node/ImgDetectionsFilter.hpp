#pragma once

#include <memory>

#include "depthai/beta/BetaNode.hpp"
#include "depthai/beta/properties/ImgDetectionsFilterProperties.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {
namespace beta {
namespace node {

/**
 * @brief Experimental node for filtering image detections.
 */
class ImgDetectionsFilter : public DeviceNodeCRTP<BetaNode, ImgDetectionsFilter, ImgDetectionsFilterProperties> {
   protected:
    Properties& getProperties() override {
        properties.initialConfig = *initialConfig;
        return properties;
    }

   public:
    constexpr static const char* NAME = "ImgDetectionsFilter";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    ImgDetectionsFilter() = default;
    ImgDetectionsFilter(std::unique_ptr<Properties> props);
    ~ImgDetectionsFilter() override;

    /**
     * Configuration used until a message is received on inputConfig.
     *
     * The default configuration forwards detections unchanged.
     */
    std::shared_ptr<ImgDetectionsFilterConfig> initialConfig = std::make_shared<ImgDetectionsFilterConfig>();

    /**
     * Image detections to filter.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgDetections, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Runtime filter configuration. The most recently received configuration
     * is reused for subsequent detection messages.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgDetectionsFilterConfig, false}}}, false}};

    /**
     * Filtered image detections.
     */
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /**
     * Select whether the node runs on the host or device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Returns true when this node runs on the host.
     *
     * Host-only pipelines always run the node on the host.
     */
    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace beta
}  // namespace dai
