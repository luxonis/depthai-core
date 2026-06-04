#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/GatherDataProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief GatherData node skeleton.
 *
 * The collection logic is intentionally left unimplemented.
 */
class GatherData : public DeviceNodeCRTP<DeviceNode, GatherData, GatherDataProperties> {
   public:
    constexpr static const char* NAME = "GatherData";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    GatherData() = default;
    GatherData(std::unique_ptr<Properties> props);

    /**
     *
     *   PROPERTIES:
     *      - referenceInput override and just collect a fixed number of MESSAGES
     *
     *
     *
     *
     */
    /**
     * Reference messages that define collection boundaries.
     */
    Input referenceInput{*this,
                         {"referenceInput", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Iterable, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Messages that are candidates for collection.
     */
    Input collectingInput{*this,
                          {"collectingInput", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Gathered output messages.
     */
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::MessageGroup, false}}}}};
};

}  // namespace node
}  // namespace dai
