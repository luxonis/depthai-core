#pragma once
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageFiltersConfig.hpp>
#include <depthai/properties/ImageFiltersProperties.hpp>
#include <memory>
#include <vector>

namespace dai {
namespace node {

class ImageFilters : public DeviceNodeCRTP<DeviceNode, ImageFilters, ImageFiltersProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "ImageFilters";
    using DeviceNodeCRTP::DeviceNodeCRTP;

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
    bool runOnHostVar = true;
};

/**
 * Node for depth confidence filter, designed to be used with the `ToF` node.
 */
class ToFDepthConfidenceFilter : public DeviceNodeCRTP<DeviceNode, ToFDepthConfidenceFilter, ToFDepthConfidenceFilterProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "DepthConfidenceFilter";
    using DeviceNodeCRTP::DeviceNodeCRTP;

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
    bool runOnHostVar = true;
};

}  // namespace node
}  // namespace dai