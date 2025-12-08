#pragma once
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageFiltersConfig.hpp>
#include <depthai/properties/ImageFiltersProperties.hpp>
#include <memory>
#include <vector>

namespace dai {
namespace node {

class ImageFilters : public DeviceNodeCRTP<DeviceNode, ImageFilters, ImageFiltersProperties>, public HostRunnable {
   protected:
    Properties& getProperties() override {
        properties.initialConfig = *initialConfig;
        return properties;
    }

   public:
    constexpr static const char* NAME = "ImageFilters";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Initial config for image filters.
     */
    std::shared_ptr<ImageFiltersConfig> initialConfig = std::make_shared<ImageFiltersConfig>();

    /**
     * Build the node.
     * @param input Input for image frames to be filtered
     * @param presetMode Preset mode for image filters
     * @return Shared pointer to the node
     */
    std::shared_ptr<ImageFilters> build(Node::Output& input, ImageFiltersPresetMode presetMode = ImageFiltersPresetMode::TOF_MID_RANGE);

    /**
     * Build the node.
     * @param presetMode Preset mode for image filters
     * @return Shared pointer to the node
     */
    std::shared_ptr<ImageFilters> build(ImageFiltersPresetMode presetMode = ImageFiltersPresetMode::TOF_MID_RANGE);

    /**
     * Input for image frames to be filtered
     */
    Node::Input input{
        *this,
        {"input", Node::DEFAULT_GROUP, Node::DEFAULT_BLOCKING, Node::DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, Node::DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Filtered frame
     */
    Node::Output output{*this, {"output", Node::DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Config to be set for a specific filter
     */
    Node::Input inputConfig{*this,
                            {"inputConfig",
                             Node::DEFAULT_GROUP,
                             Node::DEFAULT_BLOCKING,
                             Node::DEFAULT_QUEUE_SIZE,
                             {{{DatatypeEnum::ImageFiltersConfig, true}}},
                             Node::DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    /**
     * Set default profile preset for ImageFilters.
     * @param mode Preset mode for ImageFilters.
     */
    void setDefaultProfilePreset(ImageFiltersPresetMode mode);

   private:
    bool runOnHostVar = true;
};

/**
 * Node for depth confidence filter, designed to be used with the `ToF` node.
 */
class ToFDepthConfidenceFilter : public DeviceNodeCRTP<DeviceNode, ToFDepthConfidenceFilter, ToFDepthConfidenceFilterProperties>, public HostRunnable {
   protected:
    Properties& getProperties() override {
        properties.initialConfig = *initialConfig;
        return properties;
    }

   public:
    constexpr static const char* NAME = "DepthConfidenceFilter";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Initial config for ToF depth confidence filter.
     */
    std::shared_ptr<ToFDepthConfidenceFilterConfig> initialConfig = std::make_shared<ToFDepthConfidenceFilterConfig>();

    /**
     * Build the node.
     * @param depth Depth frame image, expected ImgFrame type is RAW8 or RAW16.
     * @param amplitude Amplitude frame image, expected ImgFrame type is RAW8 or RAW16.
     * @param presetMode Preset mode for ToF depth confidence filter
     * @return Shared pointer to the node
     */
    std::shared_ptr<ToFDepthConfidenceFilter> build(Node::Output& depth,
                                                    Node::Output& amplitude,
                                                    ImageFiltersPresetMode presetMode = ImageFiltersPresetMode::TOF_MID_RANGE);

    /**
     * Build the node.
     * @param presetMode Preset mode for ToF depth confidence filter
     * @return Shared pointer to the node
     */
    std::shared_ptr<ToFDepthConfidenceFilter> build(ImageFiltersPresetMode presetMode = ImageFiltersPresetMode::TOF_MID_RANGE);

    /**
     * Depth frame image, expected ImgFrame type is RAW8 or RAW16.
     */
    Node::Input depth{
        *this,
        {"depth", Node::DEFAULT_GROUP, Node::DEFAULT_BLOCKING, Node::DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, Node::DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Amplitude frame image, expected ImgFrame type is RAW8 or RAW16.
     */
    Node::Input amplitude{*this,
                          {"amplitude",
                           Node::DEFAULT_GROUP,
                           Node::DEFAULT_BLOCKING,
                           Node::DEFAULT_QUEUE_SIZE,
                           {{{DatatypeEnum::ImgFrame, true}}},
                           Node::DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * RAW16 encoded filtered depth frame
     */
    Node::Output filteredDepth{*this, {"filteredDepth", Node::DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * RAW16 encoded confidence frame
     */
    Node::Output confidence{*this, {"confidence", Node::DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Config message for runtime filter configuration
     */
    Node::Input inputConfig{*this,
                            {"inputConfig",
                             Node::DEFAULT_GROUP,
                             Node::DEFAULT_BLOCKING,
                             Node::DEFAULT_QUEUE_SIZE,
                             {{{DatatypeEnum::ToFDepthConfidenceFilterConfig, true}}},
                             Node::DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

   private:
    void applyDepthConfidenceFilter(std::shared_ptr<ImgFrame> depthFrame,
                                    std::shared_ptr<ImgFrame> amplitudeFrame,
                                    std::shared_ptr<ImgFrame> filteredDepthFrame,
                                    std::shared_ptr<ImgFrame> confidenceFrame,
                                    float threshold);

    void setDefaultProfilePreset(ImageFiltersPresetMode mode);

    bool runOnHostVar = true;
};

}  // namespace node
}  // namespace dai