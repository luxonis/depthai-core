#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

namespace dai {

namespace {

bool tryConvertIndex(int index, uint32_t& convertedIndex) {
    if(index < 0) {
        return false;
    }
    convertedIndex = static_cast<uint32_t>(index);
    return true;
}

void appendUniqueIndex(std::vector<int>& indices, int index) {
    for(int existingIndex : indices) {
        if(existingIndex == index) {
            return;
        }
    }
    indices.push_back(index);
}

bool linkReferencesNode(const Link& link, uint32_t nodeIndex) {
    return link.parentMessageIndex == nodeIndex || link.childNodeIndex == nodeIndex;
}

}  // namespace

MessageGroup::~MessageGroup() = default;

void MessageGroup::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::MessageGroup;
}

bool MessageGroup::tryParseIndex(const std::string& index, uint32_t& resolvedIndex) {
    std::size_t parsedCharacters = 0;
    try {
        auto parsed = std::stoull(index, &parsedCharacters, 10);
        if(parsedCharacters != index.size() || parsed > std::numeric_limits<uint32_t>::max()) {
            return false;
        }
        resolvedIndex = static_cast<uint32_t>(parsed);
        return true;
    } catch(const std::exception&) {
        return false;
    }
}

bool MessageGroup::findIndex(const std::string& index, uint32_t& resolvedIndex) const {
    auto keyIterator = keyToIndex.find(index);
    if(keyIterator != keyToIndex.end()) {
        resolvedIndex = keyIterator->second;
        return group.find(resolvedIndex) != group.end();
    }

    if(!tryParseIndex(index, resolvedIndex)) {
        return false;
    }

    return group.find(resolvedIndex) != group.end();
}

uint32_t MessageGroup::nextAvailableIndex() const {
    uint32_t candidate = 0;
    for(const auto& [index, _] : group) {
        if(index != candidate) {
            break;
        }
        ++candidate;
    }
    return candidate;
}

std::shared_ptr<ADatatype> MessageGroup::operator[](uint32_t index) {
    return group.at(index);
}

std::shared_ptr<ADatatype> MessageGroup::operator[](const std::string& index) {
    uint32_t resolvedIndex = 0;
    if(!findIndex(index, resolvedIndex)) {
        throw std::out_of_range("MessageGroup key not found");
    }
    return (*this)[resolvedIndex];
}

void MessageGroup::add(uint32_t index, const std::shared_ptr<ADatatype>& value) {
    group[index] = value;
}

void MessageGroup::add(const std::string& index, const std::shared_ptr<ADatatype>& value) {
    auto keyIterator = keyToIndex.find(index);
    if(keyIterator != keyToIndex.end()) {
        add(keyIterator->second, value);
        return;
    }

    auto nextIndex = nextAvailableIndex();
    keyToIndex[index] = nextIndex;
    add(nextIndex, value);
}

bool MessageGroup::setNode(int nodeIndex, std::shared_ptr<ADatatype> node) {
    uint32_t convertedNodeIndex = 0;
    if(!tryConvertIndex(nodeIndex, convertedNodeIndex)) {
        return false;
    }

    auto iterator = group.find(convertedNodeIndex);
    if(iterator == group.end()) {
        return false;
    }

    iterator->second = std::move(node);
    return true;
}

std::shared_ptr<ADatatype> MessageGroup::getNode(int nodeIndex) const {
    uint32_t convertedNodeIndex = 0;
    if(!tryConvertIndex(nodeIndex, convertedNodeIndex)) {
        return nullptr;
    }

    auto iterator = group.find(convertedNodeIndex);
    if(iterator == group.end()) {
        return nullptr;
    }

    return iterator->second;
}

int MessageGroup::addLink(int parentNodeIndex, int childNodeIndex, int parentItemIndex) {
    uint32_t convertedParentNodeIndex = 0;
    uint32_t convertedChildNodeIndex = 0;
    uint32_t convertedParentItemIndex = 0;
    if(!tryConvertIndex(parentNodeIndex, convertedParentNodeIndex) || !tryConvertIndex(childNodeIndex, convertedChildNodeIndex)
       || !tryConvertIndex(parentItemIndex, convertedParentItemIndex)) {
        return -1;
    }

    if(group.find(convertedParentNodeIndex) == group.end() || group.find(convertedChildNodeIndex) == group.end()) {
        return -1;
    }

    links.push_back(Link{convertedParentNodeIndex, convertedParentItemIndex, convertedChildNodeIndex});
    return static_cast<int>(links.size() - 1);
}

int MessageGroup::addLink(const dai::Link& link) {
    if(group.find(link.parentMessageIndex) == group.end() || group.find(link.childNodeIndex) == group.end()) {
        return -1;
    }

    links.push_back(link);
    return static_cast<int>(links.size() - 1);
}

bool MessageGroup::hasLink(int parentNodeIndex, int parentItemIndex, int childNodeIndex) const {
    uint32_t convertedParentNodeIndex = 0;
    uint32_t convertedParentItemIndex = 0;
    uint32_t convertedChildNodeIndex = 0;
    if(!tryConvertIndex(parentNodeIndex, convertedParentNodeIndex) || !tryConvertIndex(parentItemIndex, convertedParentItemIndex)
       || !tryConvertIndex(childNodeIndex, convertedChildNodeIndex)) {
        return false;
    }

    for(const auto& link : links) {
        if(link.parentMessageIndex == convertedParentNodeIndex && link.itemIndex == convertedParentItemIndex
           && link.childNodeIndex == convertedChildNodeIndex) {
            return true;
        }
    }

    return false;
}

bool MessageGroup::removeLink(int linkIndex) {
    if(linkIndex < 0 || static_cast<std::size_t>(linkIndex) >= links.size()) {
        return false;
    }

    links.erase(links.begin() + linkIndex);
    return true;
}

bool MessageGroup::removeMessageNode(int nodeIndex) {
    uint32_t convertedNodeIndex = 0;
    if(!tryConvertIndex(nodeIndex, convertedNodeIndex)) {
        return false;
    }

    if(group.find(convertedNodeIndex) == group.end()) {
        return false;
    }

    std::map<uint32_t, std::shared_ptr<ADatatype>> reindexedGroup;
    for(auto& entry : group) {
        if(entry.first == convertedNodeIndex) {
            continue;
        }

        uint32_t reindexedNodeIndex = entry.first;
        if(entry.first > convertedNodeIndex) {
            reindexedNodeIndex--;
        }
        reindexedGroup[reindexedNodeIndex] = entry.second;
    }
    group = std::move(reindexedGroup);

    for(auto iterator = keyToIndex.begin(); iterator != keyToIndex.end();) {
        if(iterator->second == convertedNodeIndex) {
            iterator = keyToIndex.erase(iterator);
            continue;
        }
        if(iterator->second > convertedNodeIndex) {
            iterator->second--;
        }
        ++iterator;
    }

    std::vector<Link> reindexedLinks;
    reindexedLinks.reserve(links.size());
    for(auto link : links) {
        if(linkReferencesNode(link, convertedNodeIndex)) {
            continue;
        }

        if(link.parentMessageIndex > convertedNodeIndex) {
            link.parentMessageIndex--;
        }
        if(link.childNodeIndex > convertedNodeIndex) {
            link.childNodeIndex--;
        }
        reindexedLinks.push_back(link);
    }
    links = std::move(reindexedLinks);

    return true;
}

std::vector<Link> MessageGroup::getLinksFromParent(int parentNodeIndex) const {
    uint32_t convertedParentNodeIndex = 0;
    if(!tryConvertIndex(parentNodeIndex, convertedParentNodeIndex)) {
        return {};
    }

    std::vector<Link> matchingLinks;
    for(const auto& link : links) {
        if(link.parentMessageIndex == convertedParentNodeIndex) {
            matchingLinks.push_back(link);
        }
    }

    return matchingLinks;
}

std::vector<Link> MessageGroup::getLinksFromParent(int parentNodeIndex, int parentItemIndex) const {
    uint32_t convertedParentNodeIndex = 0;
    uint32_t convertedParentItemIndex = 0;
    if(!tryConvertIndex(parentNodeIndex, convertedParentNodeIndex) || !tryConvertIndex(parentItemIndex, convertedParentItemIndex)) {
        return {};
    }

    std::vector<Link> matchingLinks;
    for(const auto& link : links) {
        if(link.parentMessageIndex == convertedParentNodeIndex && link.itemIndex == convertedParentItemIndex) {
            matchingLinks.push_back(link);
        }
    }

    return matchingLinks;
}

std::vector<Link> MessageGroup::getLinksToChild(int childNodeIndex) const {
    uint32_t convertedChildNodeIndex = 0;
    if(!tryConvertIndex(childNodeIndex, convertedChildNodeIndex)) {
        return {};
    }

    std::vector<Link> matchingLinks;
    for(const auto& link : links) {
        if(link.childNodeIndex == convertedChildNodeIndex) {
            matchingLinks.push_back(link);
        }
    }

    return matchingLinks;
}

std::vector<int> MessageGroup::getChildren(int parentNodeIndex) const {
    std::vector<int> children;
    for(const auto& link : getLinksFromParent(parentNodeIndex)) {
        appendUniqueIndex(children, static_cast<int>(link.childNodeIndex));
    }
    return children;
}

std::vector<int> MessageGroup::getChildren(int parentNodeIndex, int parentItemIndex) const {
    std::vector<int> children;
    for(const auto& link : getLinksFromParent(parentNodeIndex, parentItemIndex)) {
        appendUniqueIndex(children, static_cast<int>(link.childNodeIndex));
    }
    return children;
}

std::vector<int> MessageGroup::getParents(int childNodeIndex) const {
    std::vector<int> parents;
    for(const auto& link : getLinksToChild(childNodeIndex)) {
        appendUniqueIndex(parents, static_cast<int>(link.parentMessageIndex));
    }
    return parents;
}

bool MessageGroup::isLeaf(int nodeIndex) const {
    if(getNode(nodeIndex) == nullptr) {
        return false;
    }

    return getChildren(nodeIndex).empty();
}

bool MessageGroup::isRoot(int nodeIndex) const {
    if(getNode(nodeIndex) == nullptr) {
        return false;
    }

    return getParents(nodeIndex).empty();
}

std::vector<int> MessageGroup::getRootMessageNodes() const {
    std::vector<int> rootNodes;
    rootNodes.reserve(group.size());
    for(const auto& entry : group) {
        if(getLinksToChild(static_cast<int>(entry.first)).empty()) {
            rootNodes.push_back(static_cast<int>(entry.first));
        }
    }
    return rootNodes;
}

std::vector<int> MessageGroup::depthFirstOrder(int rootNodeIndex) const {
    if(getNode(rootNodeIndex) == nullptr) {
        return {};
    }

    std::vector<int> orderedNodes;
    std::vector<int> stack{rootNodeIndex};
    std::unordered_set<int> visitedNodes;

    while(!stack.empty()) {
        int nodeIndex = stack.back();
        stack.pop_back();

        if(!visitedNodes.insert(nodeIndex).second) {
            continue;
        }

        orderedNodes.push_back(nodeIndex);
        auto children = getChildren(nodeIndex);
        for(auto iterator = children.rbegin(); iterator != children.rend(); ++iterator) {
            if(visitedNodes.find(*iterator) == visitedNodes.end()) {
                stack.push_back(*iterator);
            }
        }
    }

    return orderedNodes;
}

std::vector<int> MessageGroup::breadthFirstOrder(int rootNodeIndex) const {
    if(getNode(rootNodeIndex) == nullptr) {
        return {};
    }

    std::vector<int> orderedNodes;
    std::vector<int> queue{rootNodeIndex};
    std::unordered_set<int> visitedNodes;
    visitedNodes.insert(rootNodeIndex);

    for(std::size_t queueIndex = 0; queueIndex < queue.size(); ++queueIndex) {
        int nodeIndex = queue[queueIndex];
        orderedNodes.push_back(nodeIndex);

        for(int childIndex : getChildren(nodeIndex)) {
            if(visitedNodes.insert(childIndex).second) {
                queue.push_back(childIndex);
            }
        }
    }

    return orderedNodes;
}

std::map<uint32_t, std::shared_ptr<ADatatype>>::iterator MessageGroup::begin() {
    return group.begin();
}
std::map<uint32_t, std::shared_ptr<ADatatype>>::iterator MessageGroup::end() {
    return group.end();
}

int64_t MessageGroup::getIntervalNs() const {
    if(!group.empty()) {
        auto oldest = std::dynamic_pointer_cast<Buffer>(group.begin()->second)->getTimestampDevice();
        auto latest = oldest;
        for(const auto& entry : group) {
            auto ts = std::dynamic_pointer_cast<Buffer>(entry.second)->getTimestampDevice();
            if(ts < oldest) oldest = ts;
            if(ts > latest) latest = ts;
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(latest - oldest).count();
    }
    return {};
}

int64_t MessageGroup::getNumMessages() const {
    return group.size();
}

std::vector<std::string> MessageGroup::getMessageNames() const {
    std::vector<std::pair<uint32_t, std::string>> orderedNames;
    orderedNames.reserve(keyToIndex.size());
    for(const auto& [key, index] : keyToIndex) {
        if(group.find(index) != group.end()) {
            orderedNames.emplace_back(index, key);
        }
    }

    std::sort(orderedNames.begin(), orderedNames.end(), [](const auto& lhs, const auto& rhs) {
        if(lhs.first != rhs.first) {
            return lhs.first < rhs.first;
        }
        return lhs.second < rhs.second;
    });

    std::vector<std::string> names;
    names.reserve(orderedNames.size());
    for(const auto& [_, key] : orderedNames) {
        names.push_back(key);
    }
    return names;
}

std::vector<uint32_t> MessageGroup::getMessageIndices() const {
    std::vector<uint32_t> indices;
    indices.reserve(group.size());
    for(const auto& entry : group) {
        indices.push_back(entry.first);
    }
    return indices;
}

bool MessageGroup::isSynced(int64_t thresholdNs) const {
    return getIntervalNs() <= thresholdNs;
}

}  // namespace dai
