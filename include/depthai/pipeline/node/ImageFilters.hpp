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
     * Preset modes for image filters.
     */
    enum class PresetMode : std::uint32_t {
        DEFAULT,
        BIN_PICKING,
        LONG_RANGE,
        NORM,
    };

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
    std::shared_ptr<ImageFilters> build(Node::Output& input, PresetMode presetMode = PresetMode::DEFAULT);

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
    Node::Input config{*this,
                       {"config",
                        Node::DEFAULT_GROUP,
                        Node::DEFAULT_BLOCKING,
                        Node::DEFAULT_QUEUE_SIZE,
                        {{{DatatypeEnum::ImageFiltersConfig, true}}},
                        Node::DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    class Filter {
       public:
        virtual void process(std::shared_ptr<dai::ImgFrame>& frame) = 0;
        virtual void setParams(const FilterParams& params) = 0;
        virtual ~Filter() = default;
    };

    /**
     * Add a filter to the node pipeline. The type of filter is determined based on the type of the filter parameter.
     * @param filter The filter to add
     */
    void addFilter(const FilterParams& filter);

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
    void setDefaultProfilePreset(PresetMode mode);

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
     * Preset modes for ToF depth confidence filter.
     */
    enum class PresetMode : std::uint32_t {
        DEFAULT,
        BIN_PICKING,
        LONG_RANGE,
        NORM,
    };

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
    std::shared_ptr<ToFDepthConfidenceFilter> build(Node::Output& depth, Node::Output& amplitude, PresetMode presetMode = PresetMode::DEFAULT);

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
    Node::Input config{*this,
                       {"config",
                        Node::DEFAULT_GROUP,
                        Node::DEFAULT_BLOCKING,
                        Node::DEFAULT_QUEUE_SIZE,
                        {{{DatatypeEnum::ToFDepthConfidenceFilterConfig, true}}},
                        Node::DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    /**
     * Get the confidence threshold
     */
    float getConfidenceThreshold() const;

    /**
     * Set the confidence threshold
     */
    void setConfidenceThreshold(float threshold);

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

    void setDefaultProfilePreset(PresetMode mode);

    bool runOnHostVar = true;
};

}  // namespace node
}  // namespace dai