#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

namespace dai {

namespace {
constexpr uint32_t INVALID_LINK_INDEX = std::numeric_limits<uint32_t>::max();

void appendUniqueIndex(std::vector<uint32_t>& indices, uint32_t index) {
    for(uint32_t existingIndex : indices) {
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

uint32_t MessageGroup::getLastMessageIndex() const {
    if(group.empty()) {
        throw std::runtime_error("MessageGroup is empty");
    }

    return group.rbegin()->first;
}

void MessageGroup::addMessage(uint32_t index, const std::shared_ptr<ADatatype>& value) {
    // possible  issue of overriding. Need to tighten this
    group[index] = value;
}

void MessageGroup::addMessage(const std::string& index, const std::shared_ptr<ADatatype>& value) {
    auto keyIterator = keyToIndex.find(index);
    if(keyIterator != keyToIndex.end()) {
        addMessage(keyIterator->second, value);
        return;
    }

    const auto nextIndex = nextAvailableIndex();
    keyToIndex[index] = nextIndex;
    addMessage(nextIndex, value);
}

bool MessageGroup::setNode(uint32_t nodeIndex, std::shared_ptr<ADatatype> node) {
    auto iterator = group.find(nodeIndex);
    if(iterator == group.end()) {
        return false;
    }

    iterator->second = std::move(node);
    return true;
}

std::shared_ptr<ADatatype> MessageGroup::getNode(uint32_t nodeIndex) const {
    auto iterator = group.find(nodeIndex);
    if(iterator == group.end()) {
        return nullptr;
    }

    return iterator->second;
}

uint32_t MessageGroup::addLink(uint32_t parentNodeIndex, uint32_t childNodeIndex, uint32_t parentItemIndex) {
    if(group.find(parentNodeIndex) == group.end() || group.find(childNodeIndex) == group.end()) {
        return INVALID_LINK_INDEX;
    }

    links.push_back(Link{parentNodeIndex, parentItemIndex, childNodeIndex});
    return static_cast<uint32_t>(links.size() - 1);
}

uint32_t MessageGroup::addLink(const dai::Link& link) {
    if(group.find(link.parentMessageIndex) == group.end() || group.find(link.childNodeIndex) == group.end()) {
        return INVALID_LINK_INDEX;
    }

    links.push_back(link);
    return static_cast<uint32_t>(links.size() - 1);
}

bool MessageGroup::hasLink(uint32_t parentNodeIndex, uint32_t parentItemIndex, uint32_t childNodeIndex) const {
    for(const auto& link : links) {
        if(link.parentMessageIndex == parentNodeIndex && link.itemIndex == parentItemIndex && link.childNodeIndex == childNodeIndex) {
            return true;
        }
    }

    return false;
}

bool MessageGroup::removeLink(uint32_t linkIndex) {
    if(linkIndex >= links.size()) {
        return false;
    }

    links.erase(links.begin() + linkIndex);
    return true;
}

bool MessageGroup::removeMessageNode(uint32_t nodeIndex) {
    if(group.find(nodeIndex) == group.end()) {
        return false;
    }

    std::map<uint32_t, std::shared_ptr<ADatatype>> reindexedGroup;
    for(auto& entry : group) {
        if(entry.first == nodeIndex) {
            continue;
        }

        uint32_t reindexedNodeIndex = entry.first;
        if(entry.first > nodeIndex) {
            reindexedNodeIndex--;
        }
        reindexedGroup[reindexedNodeIndex] = entry.second;
    }
    group = std::move(reindexedGroup);

    for(auto iterator = keyToIndex.begin(); iterator != keyToIndex.end();) {
        if(iterator->second == nodeIndex) {
            iterator = keyToIndex.erase(iterator);
            continue;
        }
        if(iterator->second > nodeIndex) {
            iterator->second--;
        }
        ++iterator;
    }

    std::vector<Link> reindexedLinks;
    reindexedLinks.reserve(links.size());
    for(auto link : links) {
        if(linkReferencesNode(link, nodeIndex)) {
            continue;
        }

        if(link.parentMessageIndex > nodeIndex) {
            link.parentMessageIndex--;
        }
        if(link.childNodeIndex > nodeIndex) {
            link.childNodeIndex--;
        }
        reindexedLinks.push_back(link);
    }
    links = std::move(reindexedLinks);

    return true;
}

std::vector<Link> MessageGroup::getLinksFromParent(uint32_t parentNodeIndex) const {
    std::vector<Link> matchingLinks;
    for(const auto& link : links) {
        if(link.parentMessageIndex == parentNodeIndex) {
            matchingLinks.push_back(link);
        }
    }

    return matchingLinks;
}

std::vector<Link> MessageGroup::getLinksFromParent(uint32_t parentNodeIndex, uint32_t parentItemIndex) const {
    std::vector<Link> matchingLinks;
    for(const auto& link : links) {
        if(link.parentMessageIndex == parentNodeIndex && link.itemIndex == parentItemIndex) {
            matchingLinks.push_back(link);
        }
    }

    return matchingLinks;
}

std::vector<Link> MessageGroup::getLinksToChild(uint32_t childNodeIndex) const {
    std::vector<Link> matchingLinks;
    for(const auto& link : links) {
        if(link.childNodeIndex == childNodeIndex) {
            matchingLinks.push_back(link);
        }
    }

    return matchingLinks;
}

std::vector<uint32_t> MessageGroup::getChildren(uint32_t parentNodeIndex) const {
    std::vector<uint32_t> children;
    for(const auto& link : getLinksFromParent(parentNodeIndex)) {
        appendUniqueIndex(children, link.childNodeIndex);
    }
    return children;
}

std::vector<uint32_t> MessageGroup::getChildren(uint32_t parentNodeIndex, uint32_t parentItemIndex) const {
    std::vector<uint32_t> children;
    for(const auto& link : getLinksFromParent(parentNodeIndex, parentItemIndex)) {
        appendUniqueIndex(children, link.childNodeIndex);
    }
    return children;
}

std::vector<uint32_t> MessageGroup::getParents(uint32_t childNodeIndex) const {
    std::vector<uint32_t> parents;
    for(const auto& link : getLinksToChild(childNodeIndex)) {
        appendUniqueIndex(parents, link.parentMessageIndex);
    }
    return parents;
}

bool MessageGroup::isLeaf(uint32_t nodeIndex) const {
    if(getNode(nodeIndex) == nullptr) {
        return false;
    }

    return getChildren(nodeIndex).empty();
}

bool MessageGroup::isRoot(uint32_t nodeIndex) const {
    if(getNode(nodeIndex) == nullptr) {
        return false;
    }

    return getParents(nodeIndex).empty();
}

std::vector<uint32_t> MessageGroup::getRootMessageNodes() const {
    std::vector<uint32_t> rootNodes;
    rootNodes.reserve(group.size());
    for(const auto& entry : group) {
        if(getLinksToChild(entry.first).empty()) {
            rootNodes.push_back(entry.first);
        }
    }
    return rootNodes;
}

std::vector<uint32_t> MessageGroup::getMessageSiblings(uint32_t nodeIndex) const {
    if(getNode(nodeIndex) == nullptr) {
        return {};
    }

    const auto parentLinks = getLinksToChild(nodeIndex);
    if(parentLinks.empty()) {
        return {};
    }

    std::vector<uint32_t> siblingIndices;
    siblingIndices.reserve(links.size());

    for(const auto& parentLink : parentLinks) {
        for(const auto& link : links) {
            if(link.parentMessageIndex == parentLink.parentMessageIndex && link.itemIndex == parentLink.itemIndex) {
                appendUniqueIndex(siblingIndices, link.childNodeIndex);
            }
        }
    }

    return siblingIndices;
}

std::vector<uint32_t> MessageGroup::depthFirstOrder(uint32_t rootNodeIndex) const {
    if(getNode(rootNodeIndex) == nullptr) {
        return {};
    }

    std::vector<uint32_t> orderedNodes;
    std::vector<uint32_t> stack{rootNodeIndex};
    std::unordered_set<uint32_t> visitedNodes;

    while(!stack.empty()) {
        uint32_t nodeIndex = stack.back();
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

std::vector<uint32_t> MessageGroup::breadthFirstOrder(uint32_t rootNodeIndex) const {
    if(getNode(rootNodeIndex) == nullptr) {
        return {};
    }

    std::vector<uint32_t> orderedNodes;
    std::vector<uint32_t> queue{rootNodeIndex};
    std::unordered_set<uint32_t> visitedNodes;
    visitedNodes.insert(rootNodeIndex);

    for(std::size_t queueIndex = 0; queueIndex < queue.size(); ++queueIndex) {
        uint32_t nodeIndex = queue[queueIndex];
        orderedNodes.push_back(nodeIndex);

        for(uint32_t childIndex : getChildren(nodeIndex)) {
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
