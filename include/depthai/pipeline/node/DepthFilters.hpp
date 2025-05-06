#pragma once
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/DepthFiltersConfig.hpp>
#include <depthai/properties/DepthFiltersProperties.hpp>
#include <memory>
#include <vector>

namespace dai {
namespace node {

class SequentialDepthFilters : public DeviceNodeCRTP<DeviceNode, SequentialDepthFilters, SequentialDepthFiltersProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "SequentialDepthFilters";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    Node::Input input{
        *this,
        {"input", Node::DEFAULT_GROUP, Node::DEFAULT_BLOCKING, Node::DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, Node::DEFAULT_WAIT_FOR_MESSAGE}};
    Node::Output output{*this, {"output", Node::DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Config to be set for a specific filter
     */
    Node::Input config{*this,
                       {"config",
                        Node::DEFAULT_GROUP,
                        Node::DEFAULT_BLOCKING,
                        Node::DEFAULT_QUEUE_SIZE,
                        {{{DatatypeEnum::SequentialDepthFiltersConfig, true}}},
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

class DepthConfidenceFilter : public DeviceNodeCRTP<DeviceNode, DepthConfidenceFilter, DepthConfidenceFilterProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "DepthConfidenceFilter";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    Node::Input depth{
        *this,
        {"depth", Node::DEFAULT_GROUP, Node::DEFAULT_BLOCKING, Node::DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, Node::DEFAULT_WAIT_FOR_MESSAGE}};
    Node::Input amplitude{*this,
                          {"amplitude",
                           Node::DEFAULT_GROUP,
                           Node::DEFAULT_BLOCKING,
                           Node::DEFAULT_QUEUE_SIZE,
                           {{{DatatypeEnum::ImgFrame, true}}},
                           Node::DEFAULT_WAIT_FOR_MESSAGE}};

    Node::Output filteredDepth{*this, {"filteredDepth", Node::DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Node::Output confidence{*this, {"confidence", Node::DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    Node::Input config{*this,
                       {"config",
                        Node::DEFAULT_GROUP,
                        Node::DEFAULT_BLOCKING,
                        Node::DEFAULT_QUEUE_SIZE,
                        {{{DatatypeEnum::DepthConfidenceFilterConfig, true}}},
                        Node::DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    float getConfidenceThreshold() const;
    void setConfidenceThreshold(float threshold);
    void setRunOnHost(bool runOnHost);
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