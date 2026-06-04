#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "depthai/common/ADatatypeSharedPtrSerialization.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"
namespace dai {

struct Link {
    uint32_t parentMessageIndex;
    uint32_t itemIndex = 0;  // default. Parent has one child (eg. ImgFrame does not have multiple items)
    uint32_t childNodeIndex;

    DEPTHAI_SERIALIZE(Link, parentMessageIndex, itemIndex, childNodeIndex);
};

/**
 * MessageGroup message. Carries multiple messages in one.
 */
class MessageGroup : public Buffer {
   public:
    std::map<uint32_t, std::shared_ptr<ADatatype>> group;
    std::vector<Link> links;
    std::map<std::string, uint32_t> keyToIndex;

    virtual ~MessageGroup();

    /// Group
    std::shared_ptr<ADatatype> operator[](uint32_t index);
    std::shared_ptr<ADatatype> operator[](const std::string& index);
    template <typename T>
    std::shared_ptr<T> get(uint32_t index) {
        return std::dynamic_pointer_cast<T>(get(index));
    }
    template <typename T>
    std::shared_ptr<T> get(const std::string& index) {
        return std::dynamic_pointer_cast<T>(get(index));
    }

    std::shared_ptr<ADatatype> get(uint32_t index) {
        auto iterator = group.find(index);
        if(iterator == group.end()) {
            return nullptr;
        }
        return iterator->second;
    }

    std::shared_ptr<ADatatype> get(const std::string& index) {
        uint32_t resolvedIndex = 0;
        if(!findIndex(index, resolvedIndex)) {
            return nullptr;
        }
        return get(resolvedIndex);
    }

    // TODO(Morato) - this API is dangerous, when T is a base reference to a derived class
    // template <typename T>
    // void add(uint32_t index, const T& value) {
    //     static_assert(std::is_base_of<ADatatype, T>::value, "T must derive from ADatatype");
    //     group[index] = std::make_shared<T>(value);
    // }
    void add(uint32_t index, const std::shared_ptr<ADatatype>& value);
    void add(const std::string& index, const std::shared_ptr<ADatatype>& value);

    /// Replaces a node at the given index; returns false if the index is invalid.
    bool setNode(int nodeIndex, std::shared_ptr<ADatatype> node);

    /// Returns the node at the given index, or nullptr if the index is invalid.
    std::shared_ptr<ADatatype> getNode(int nodeIndex) const;

    /// Adds a link from a parent node/item to a child node and returns the link index.
    int addLink(int parentNodeIndex, int childNodeIndex, int parentItemIndex = 0);

    /// Adds a link structure directly and returns the link index.
    int addLink(const dai::Link& link);

    /// Returns true if a specific parent-item-child link exists.
    bool hasLink(int parentNodeIndex, int parentItemIndex, int childNodeIndex) const;

    /// Removes a link by index; returns false if the index is invalid.
    bool removeLink(int linkIndex);

    /// Removes a node and all links referencing it; reindexes links to keep indices consistent.
    bool removeMessageNode(int nodeIndex);

    /// Returns all links originating from a parent node.
    std::vector<Link> getLinksFromParent(int parentNodeIndex) const;

    /// Returns all links originating from a parent node for a specific item index.
    std::vector<Link> getLinksFromParent(int parentNodeIndex, int parentItemIndex) const;

    /// Returns all links that point to the given child node.
    std::vector<Link> getLinksToChild(int childNodeIndex) const;

    /// Returns child node indices for a parent node across all parent items.
    std::vector<int> getChildren(int parentNodeIndex) const;

    /// Returns child node indices for a parent node and a specific item index.
    std::vector<int> getChildren(int parentNodeIndex, int parentItemIndex) const;

    /// Returns parent node indices for a given child node.
    std::vector<int> getParents(int childNodeIndex) const;

    /// Returns true if the node has no children.
    bool isLeaf(int nodeIndex) const;

    /// Returns true if the node has no parents.
    bool isRoot(int nodeIndex) const;

    /// Returns all root node indices.
    std::vector<int> getRootMessageNodes() const;

    /// Returns a depth-first traversal order starting from a given root node.
    std::vector<int> depthFirstOrder(int rootNodeIndex) const;

    /// Returns a breadth-first traversal order starting from a given root node.
    std::vector<int> breadthFirstOrder(int rootNodeIndex) const;

    // Iterators
    std::map<uint32_t, std::shared_ptr<ADatatype>>::iterator begin();
    std::map<uint32_t, std::shared_ptr<ADatatype>>::iterator end();

    /**
     * True if all messages in the group are in the interval
     * @param thresholdNs Maximal interval between messages
     */
    bool isSynced(int64_t thresholdNs) const;

    /**
     * Retrieves interval between the first and the last message in the group.
     */
    int64_t getIntervalNs() const;

    int64_t getNumMessages() const;

    /**
     * Gets the names of messages in the group
     */
    std::vector<std::string> getMessageNames() const;

    /**
     * Gets the indices of messages in the group
     */
    std::vector<uint32_t> getMessageIndices() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MessageGroup;
    }
    DEPTHAI_SERIALIZE(MessageGroup, group, links, keyToIndex, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);

   private:
    bool findIndex(const std::string& index, uint32_t& resolvedIndex) const;
    uint32_t nextAvailableIndex() const;
    static bool tryParseIndex(const std::string& index, uint32_t& resolvedIndex);
};

}  // namespace dai
