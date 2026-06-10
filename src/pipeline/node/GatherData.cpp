#include "depthai/pipeline/node/GatherData.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>

#include "pipeline/MessageQueue.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/ADatatype.hpp"
#include "pipeline/datatype/Buffer.hpp"
#include "pipeline/datatype/DatatypeEnum.hpp"
#include "pipeline/datatype/ImgDetections.hpp"
#include "pipeline/datatype/MessageGroup.hpp"
#include "pipeline/datatype/MissingDataMessage.hpp"
#include "pipeline/datatype/SpatialImgDetections.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

namespace {

std::shared_ptr<MessageGroup> cloneMessageGroupShallow(const std::shared_ptr<MessageGroup>& sourceMessageGroup) {
    if(sourceMessageGroup == nullptr) {
        throw std::runtime_error("Source MessageGroup is null, cannot clone.");
    }

    auto clonedMessageGroup = std::make_shared<MessageGroup>();
    clonedMessageGroup->group = sourceMessageGroup->group;
    clonedMessageGroup->links = sourceMessageGroup->links;
    clonedMessageGroup->keyToIndex = sourceMessageGroup->keyToIndex;
    clonedMessageGroup->setTimestamp(sourceMessageGroup->getTimestamp());
    clonedMessageGroup->setSequenceNum(sourceMessageGroup->getSequenceNum());
    clonedMessageGroup->setTimestampDevice(sourceMessageGroup->getTimestampDevice());
    return clonedMessageGroup;
}

}  // namespace

bool GatherData::runOnHost() const {
    return runOnHostVar;
}

void GatherData::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

GatherData::~GatherData() = default;

std::shared_ptr<MessageGroup> GatherData::createCollectionMessageGroup(const std::shared_ptr<Buffer>& referenceBuffer) {
    if(referenceBuffer->getDatatype() == DatatypeEnum::MessageGroup) {
        auto referenceMessageGroup = std::dynamic_pointer_cast<MessageGroup>(referenceBuffer);
        if(referenceMessageGroup == nullptr) {
            throw std::runtime_error("Reference buffer reports MessageGroup datatype but cannot be cast to MessageGroup.");
        }
        return cloneMessageGroupShallow(referenceMessageGroup);
    }

    // This is means its the first GatherData node so need to create the MessageGroup
    // logger->warn("Reference buffer is not a MessageGroup, creating a new MessageGroup with the reference buffer as the first message.");
    auto messageGroup = std::make_shared<MessageGroup>();

    messageGroup->addMessage(0, referenceBuffer);

    messageGroup->setTimestamp(referenceBuffer->getTimestamp());
    messageGroup->setSequenceNum(referenceBuffer->getSequenceNum());
    messageGroup->setTimestampDevice(referenceBuffer->getTimestampDevice());

    return messageGroup;
}

uint32_t getIterableSize(std::shared_ptr<ADatatype>& adtatatypeMsg) {
    auto buffer = std::dynamic_pointer_cast<Buffer>(adtatatypeMsg);
    if(!buffer) {
        throw std::runtime_error("Message is not a Buffer, cannot determine iterable size.");
    }
    auto datatype = buffer->getDatatype();

    if(isDatatypeSubclassOf(DatatypeEnum::Iterable, datatype)) {
        if(datatype == DatatypeEnum::ImgDetections) {
            return std::dynamic_pointer_cast<ImgDetections>(buffer)->getSize();
            // } else if(datatype == DatatypeEnum::SpatialImgDetections) {
            //     return std::dynamic_pointer_cast<SpatialImgDetections>(buffer)->getSize();
        } else {
            throw std::runtime_error("Iterable datatype " + std::to_string(static_cast<int>(datatype)) + " is not supported in getIterableSize.");
        }
    } else {
        return 1;
    }
}

std::vector<uint32_t> GatherData::getMessageSizes(std::vector<uint32_t>& messagesInLayer, std::shared_ptr<MessageGroup>& referenceGroup) {
    std::vector<uint32_t> messageSizes;
    for(uint32_t messageIndex : messagesInLayer) {
        auto message = referenceGroup->getNode(messageIndex);
        if(!message) {
            throw std::runtime_error("Message index " + std::to_string(messageIndex) + " does not exist in the message group.");
        }

        messageSizes.push_back(getIterableSize(message));
    }

    return messageSizes;
}

std::vector<uint32_t> GatherData::getMessageIndicesToCollect(std::shared_ptr<MessageGroup>& referenceGroup) {
    // options:
    // 1) only one node: return its iterable size or 1 if not iterable
    // 2) multiple nodes:
    //    2.1) Find the last layer of iterable messages
    //    2.2) getSiblings()
    //    2.3) vector of iterable sizes of the children (if not iterable, size is 1)

    // auto& logger = ThreadedNode::pimpl->logger;

    if(referenceGroup->getChildren(referenceGroup->getRootMessageNodes()[0]).empty()) {
        return {0};
    }

    auto depthFirstOrder = referenceGroup->depthFirstOrder(0);

    uint32_t currentCandidateToIterableLayer = 0;
    bool foundIterableLayer = false;
    for(uint32_t nodeIndex : depthFirstOrder) {
        if(referenceGroup->isLeaf(nodeIndex) && !foundIterableLayer) {
            // we came down to leaf before finding any iterable message, so we need to link to all leaves
            currentCandidateToIterableLayer = nodeIndex;
            break;
        }

        auto currentLayerDatatype = referenceGroup->getNode(nodeIndex)->getDatatype();
        // logger->warn("Checking node index {} with datatype {} for iterability", nodeIndex, currentLayerDatatype);
        if(isDatatypeSubclassOf(DatatypeEnum::Iterable, currentLayerDatatype)) {
            // logger->warn("is ITERABLE");
            // Found a potential candidate layer, but there might be another one deeper in the tree
            currentCandidateToIterableLayer = nodeIndex;
            foundIterableLayer = true;
        }
    }
    // for(const auto& [nodeIndex, node] : referenceGroup->group) {
    //     if(node == nullptr) {
    //         logger->warn("MessageGroup node {} has null data", nodeIndex);
    //         continue;
    //     }
    //     logger->warn("MessageGroup node {} has datatype {}", nodeIndex, node->getDatatype());
    // }

    // const auto candidateNode = referenceGroup->getNode(currentCandidateToIterableLayer);
    // if(candidateNode == nullptr) {
    //     logger->warn("currentCandidateToIterableLayer {} has null data", currentCandidateToIterableLayer);
    // } else {
    //     logger->warn("currentCandidateToIterableLayer {} has datatype {}", currentCandidateToIterableLayer, candidateNode->getDatatype());
    // }

    auto messageIndicesToCollect = referenceGroup->getMessageSiblings(currentCandidateToIterableLayer);
    if(messageIndicesToCollect.empty()) {
        throw std::runtime_error("No message siblings found for node index " + std::to_string(currentCandidateToIterableLayer)
                                 + ". At least one message should be collected.");
    }

    // indicesToCollect = getMessageSizes(messageIndicesToCollect, referenceGroup);

    return messageIndicesToCollect;
}

std::shared_ptr<Buffer> GatherData::createMissingDataMessage(const std::shared_ptr<ADatatype>& sourceMessage) {
    auto missingDataMessage = std::make_shared<MissingDataMessage>();
    auto sourceBuffer = std::dynamic_pointer_cast<Buffer>(sourceMessage);
    if(sourceBuffer != nullptr) {
        missingDataMessage->setTimestamp(sourceBuffer->getTimestamp());
        missingDataMessage->setTimestampDevice(sourceBuffer->getTimestampDevice());
        missingDataMessage->setSequenceNum(sourceBuffer->getSequenceNum());
    }
    return missingDataMessage;
}

bool GatherData::attachLinkToBranchAsLeaf(std::shared_ptr<MessageGroup>& msgGroup, uint32_t parentIndex, uint32_t childIndex, uint32_t parentItemIndex) {
    if(!msgGroup) {
        throw std::runtime_error("Message group is null, cannot attach child to branch leaf.");
    }
    if(msgGroup->getNode(parentIndex) == nullptr) {
        throw std::runtime_error("Parent node index " + std::to_string(parentIndex) + " does not exist in the message group.");
    }
    if(msgGroup->getNode(childIndex) == nullptr) {
        throw std::runtime_error("Child node index " + std::to_string(childIndex) + " does not exist in the message group.");
    }

    constexpr uint32_t INVALID_LINK_INDEX = std::numeric_limits<uint32_t>::max();

    auto selectedBranchLinks = msgGroup->getLinksFromParent(parentIndex, parentItemIndex);
    if(selectedBranchLinks.size() > 1) {
        throw std::runtime_error("Expected a single child on branch " + std::to_string(parentItemIndex) + " from parent node " + std::to_string(parentIndex)
                                 + ", but found " + std::to_string(selectedBranchLinks.size()) + ".");
    }

    // If this branch has not been expanded yet, attach directly to the branching parent using the selected item index.
    if(selectedBranchLinks.empty()) {
        return msgGroup->addLink(parentIndex, childIndex, parentItemIndex) != INVALID_LINK_INDEX;
    }

    uint32_t currentNodeIndex = selectedBranchLinks.front().childNodeIndex;
    while(true) {
        if(msgGroup->isLeaf(currentNodeIndex)) {
            // We have reached the end of the branch, we can attach the new child here
            return msgGroup->addLink(currentNodeIndex, childIndex) != INVALID_LINK_INDEX;
        }

        auto descendantLinks = msgGroup->getLinksFromParent(currentNodeIndex);
        // if(descendantLinks.empty()) {
        //     return msgGroup->addLink(currentNodeIndex, childIndex) != INVALID_LINK_INDEX;
        // }

        if(descendantLinks.size() > 1) {
            throw std::runtime_error("Expected a linear descendant chain below node " + std::to_string(currentNodeIndex) + ", but found "
                                     + std::to_string(descendantLinks.size()) + " children.");
        }

        currentNodeIndex = descendantLinks.front().childNodeIndex;
    }
}

void GatherData::run() {
    /*
    * LEMMAS:
    1) All work is done on the last layer that has iterable messages. If no iterable => work on last layer
    2) Append / expand only
    3) The only node to MAKE a CollectionTree (messageGroup)
    4) Messages are "collected" by the assumption that they arrive in order. No checking is made
    */

    while(mainLoop()) {
        std::shared_ptr<Buffer> referenceInputBuffer;
        auto& logger = ThreadedNode::pimpl->logger;

        {
            auto inputBlockEvent = this->inputBlockEvent();
            bool timedOut = false;
            // auto rStart = std::chrono::steady_clock::now();
            // logger->warn("referenceInput fullness: {} / {}", referenceInput.getSize(), referenceInput.getMaxSize());
            referenceInputBuffer = referenceInput.get<Buffer>(std::chrono::seconds(10), timedOut);
            if(timedOut) {
                logger->error("Timed out waiting for reference input buffer. No data received on reference input for 10 seconds. Restarting wait.");
                continue;
            }
            // logger->warn("Received reference input buffer after {} ms",
            //              std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - rStart).count());
        }

        // auto detMsg = std::dynamic_pointer_cast<ImgDetections>(referenceInputBuffer);
        // logger->warn("GatherData received reference message with {} detections. Its getSize() iterable function specifies: {}",
        //              detMsg->detections.size(),
        //              detMsg->getSize());

        // logger->warn("GatherData creating collectionMessageGroup");
        // if(referenceInputBuffer->getDatatype() == DatatypeEnum::MessageGroup) {
        //     logger->warn("Reference buffer is already a MessageGroup");
        // }

        std::shared_ptr<MessageGroup> outputMessageGroup = createCollectionMessageGroup(referenceInputBuffer);

        // logger->warn("Getting message indices to collect on");
        std::vector<uint32_t> messageIndicesToCollectOn = getMessageIndicesToCollect(outputMessageGroup);

        // logger->warn("got {} message indices to collect on", messageIndicesToCollectOn.size());

        for(uint32_t collectingParentIndex : messageIndicesToCollectOn) {
            auto collectingMsg = outputMessageGroup->getNode(collectingParentIndex);
            if(!collectingMsg) {
                throw std::runtime_error("Message index " + std::to_string(collectingParentIndex) + " does not exist in the message group.");
            }
            // logger->warn("Collecting on message index {} with datatype {} and iterable size {}",
            //              collectingParentIndex,
            //              collectingMsg->getDatatype(),
            //              getIterableSize(collectingMsg));

            // auto buffer_msg = std::dynamic_pointer_cast<Buffer>(collectingMsg);
            // if(buffer_msg) {
            //     const auto timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(buffer_msg->getTimestamp().time_since_epoch()).count();
            //     const auto timestampDeviceNs =
            //         std::chrono::duration_cast<std::chrono::nanoseconds>(buffer_msg->getTimestampDevice().time_since_epoch()).count();
            //     logger->warn("Message has seqNum {}, timestamp {} ns, and timestampDevice {} ns", buffer_msg->getSequenceNum(), timestampNs,
            //     timestampDeviceNs);
            // } else {
            //     logger->warn("Message index {} is not a Buffer, skipping sequence/timestamp debug logging.", collectingParentIndex);
            // }
            // std::chrono::steady_clock::time_point previousCollectMessageTime = buffer_msg->getTimestamp();
            // logger->warn("on datatype {} of message index {}", buffer_msg->getDatatype(), collectingParentIndex);

            auto numIterations = getIterableSize(collectingMsg);
            // logger->warn("getIterableSize returned {}", numIterations);
            uint32_t currentNewNodeIndex = outputMessageGroup->getLastMessageIndex() + 1;

            if(numIterations == 0) {
                outputMessageGroup->addMessage(currentNewNodeIndex, createMissingDataMessage(collectingMsg));
                if(!attachLinkToBranchAsLeaf(outputMessageGroup, collectingParentIndex, currentNewNodeIndex, 0)) {
                    throw std::runtime_error("Failed to attach missing child node " + std::to_string(currentNewNodeIndex) + " under parent node "
                                             + std::to_string(collectingParentIndex) + " on branch 0.");
                }
                continue;
            }
            auto tStart = std::chrono::steady_clock::now();
            for(uint32_t parentItemIndex = 0; parentItemIndex < numIterations; parentItemIndex++) {
                // logger->warn("Colecting on item {} of message index {}", parentItemIndex, collectingParentIndex);
                // logger->warn("collectingInput fullness: {} / {}", collectingInput.getSize(), collectingInput.getMaxSize());
                bool timedOutInput = false;
                auto collectingInputMsg = collectingInput.get<Buffer>(std::chrono::seconds(10), timedOutInput);
                if(timedOutInput) {
                    logger->error(
                        "Timed out waiting for collecting input buffer. No data received on collecting input for 10 seconds while collecting item {} of "
                        "message index {}. Restarting wait.",
                        parentItemIndex,
                        collectingParentIndex);
                    break;
                }
                passthroughCollectingInput.send(collectingInputMsg);

                outputMessageGroup->addMessage(currentNewNodeIndex, collectingInputMsg);
                if(!attachLinkToBranchAsLeaf(outputMessageGroup, collectingParentIndex, currentNewNodeIndex, parentItemIndex)) {
                    throw std::runtime_error("Failed to attach child node " + std::to_string(currentNewNodeIndex) + " under parent node "
                                             + std::to_string(collectingParentIndex) + " on branch " + std::to_string(parentItemIndex) + ".");
                }
                currentNewNodeIndex += 1;
            }
            // auto tEnd = std::chrono::steady_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count();
            // logger->warn("Finished collecting {} messages in {} ms", numIterations, duration);
        }

        {
            // auto oStart = std::chrono::steady_clock::now();
            auto blockEvent = this->outputBlockEvent();

            output.send(outputMessageGroup);
            // logger->warn("Sending output message with seqNum {} took {} ms",
            //              outputMessageGroup->getSequenceNum(),
            //              std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - oStart).count());
        }
    }
    // DatatypeEnum inputDatatype = classifyInputDatatype(firstInput);
    // DatatypeEnum alignToDatatype = classifyInputDatatype(firstAlignTo);

    // const auto isImgFrameOrTransformable = [](DatatypeEnum datatype) { return datatype == DatatypeEnum::ImgFrame ||
    // isTransformableDatatype(datatype); };
}

}  // namespace node
}  // namespace dai
