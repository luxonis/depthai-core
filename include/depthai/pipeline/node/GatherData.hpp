#pragma once

#include <cstdint>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/GatherDataProperties.hpp>
#include <memory>
#include <optional>
#include <string>

#include "depthai/pipeline/datatype/MessageGroup.hpp"

namespace dai {
namespace node {

/**
 * @brief GatherData node skeleton.
 *
 * The collection logic is intentionally left unimplemented.
 */
class GatherData : public DeviceNodeCRTP<DeviceNode, GatherData, GatherDataProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "GatherData";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    GatherData() = default;
    ~GatherData() override;

    /**
     *
     *   PROPERTIES:
     *     - referenceInput override and just collect a fixed number of MESSAGES
     *     - is auto added to all nodes. Only instantiated if the syncInput is linked to
     *
     *
     *
     *
     */
    /**
     * Reference messages that define collection boundaries.
     */
    Input referenceInput{*this, {"referenceInput", DEFAULT_GROUP, DEFAULT_BLOCKING, 4, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Messages that are candidates for collection.
     */
    Input collectingInput{*this, {"collectingInput", DEFAULT_GROUP, DEFAULT_BLOCKING, 40, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Gathered output messages.
     */
    Output output{*this, {"output", DEFAULT_GROUP, {{{DatatypeEnum::MessageGroup, false}}}}};

    Output passthroughCollectingInput{*this, {"passthroughCollectingInput", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, false}}}}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = false;

    std::shared_ptr<MessageGroup> createCollectionMessageGroup(const std::shared_ptr<Buffer>& referenceBuffer);

    std::vector<uint32_t> getMessageIndicesToCollect(std::shared_ptr<MessageGroup>& referenceGroup);

    int getNumIterationsToCollect(uint32_t messageIndex);

    std::vector<uint32_t> getMessageSizes(std::vector<uint32_t>& messagesInLayer, std::shared_ptr<MessageGroup>& referenceGroup);

    // this should create a link that from the leaf of the subtree of root parentIndex to the childIndex.
    // this will look like:
    //                  parentNode
    //                /   /  |  \
    //               a1  a2  a3  a4
    //              /   /    |    \
    //             b1  b2    b3    b4
    //            /    |     |      \
    //     childIndex  ...  (missing)
    //
    // so parentItemIndex select which one of the 4 branches to go down. childIndex is the child it refers to
    bool attachLinkToBranchAsLeaf(std::shared_ptr<MessageGroup>&, uint32_t parentIndex, uint32_t childIndex, uint32_t parentItemIndex);
};

}  // namespace node
}  // namespace dai
