#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"

namespace gather_data_examples {

namespace detail {

struct MessageGroupRenderNode {
    uint32_t nodeIndex = 0;
    int depth = 0;
    int row = 0;
    bool isRoot = false;
    bool isUnreachable = false;
    std::string label;
    std::string annotation;
};

struct MessageGroupRenderEdge {
    uint32_t parentNodeIndex = 0;
    uint32_t childNodeIndex = 0;
    uint32_t itemIndex = 0;
    bool isShared = false;
    bool isCycle = false;
};

struct MessageGroupRenderGraph {
    std::vector<MessageGroupRenderNode> nodes;
    std::vector<MessageGroupRenderEdge> edges;
    std::string treeText;
};

inline MessageGroupRenderGraph buildMessageGroupRenderGraph(const dai::MessageGroup& messageGroup);

}  // namespace detail

inline const char* datatypeEnumName(dai::DatatypeEnum datatype) {
#define DAI_DATATYPE_ENUM_CASE(name) \
    case dai::DatatypeEnum::name:    \
        return #name

    switch(datatype) {
        DAI_DATATYPE_ENUM_CASE(ADatatype);
        DAI_DATATYPE_ENUM_CASE(Buffer);
        DAI_DATATYPE_ENUM_CASE(ImgFrame);
        DAI_DATATYPE_ENUM_CASE(EncodedFrame);
        DAI_DATATYPE_ENUM_CASE(SegmentationMask);
        DAI_DATATYPE_ENUM_CASE(GateControl);
        DAI_DATATYPE_ENUM_CASE(NNData);
        DAI_DATATYPE_ENUM_CASE(ImageManipConfig);
        DAI_DATATYPE_ENUM_CASE(CameraControl);
        DAI_DATATYPE_ENUM_CASE(ImgDetections);
        DAI_DATATYPE_ENUM_CASE(SpatialImgDetections);
        DAI_DATATYPE_ENUM_CASE(SystemInformation);
        DAI_DATATYPE_ENUM_CASE(SystemInformationRVC4);
        DAI_DATATYPE_ENUM_CASE(SpatialLocationCalculatorConfig);
        DAI_DATATYPE_ENUM_CASE(SpatialLocationCalculatorData);
        DAI_DATATYPE_ENUM_CASE(EdgeDetectorConfig);
        DAI_DATATYPE_ENUM_CASE(AprilTagConfig);
        DAI_DATATYPE_ENUM_CASE(AprilTags);
        DAI_DATATYPE_ENUM_CASE(Tracklets);
        DAI_DATATYPE_ENUM_CASE(IMUData);
        DAI_DATATYPE_ENUM_CASE(StereoDepthConfig);
        DAI_DATATYPE_ENUM_CASE(NeuralDepthConfig);
        DAI_DATATYPE_ENUM_CASE(GPUStereoConfig);
        DAI_DATATYPE_ENUM_CASE(FeatureTrackerConfig);
        DAI_DATATYPE_ENUM_CASE(ThermalConfig);
        DAI_DATATYPE_ENUM_CASE(ToFConfig);
        DAI_DATATYPE_ENUM_CASE(TrackedFeatures);
        DAI_DATATYPE_ENUM_CASE(BenchmarkReport);
        DAI_DATATYPE_ENUM_CASE(MessageGroup);
        DAI_DATATYPE_ENUM_CASE(MapData);
        DAI_DATATYPE_ENUM_CASE(TransformData);
        DAI_DATATYPE_ENUM_CASE(PointCloudConfig);
        DAI_DATATYPE_ENUM_CASE(PointCloudData);
        DAI_DATATYPE_ENUM_CASE(RGBDData);
        DAI_DATATYPE_ENUM_CASE(ImageAlignConfig);
        DAI_DATATYPE_ENUM_CASE(ImgAnnotations);
        DAI_DATATYPE_ENUM_CASE(ImageFiltersConfig);
        DAI_DATATYPE_ENUM_CASE(ToFDepthConfidenceFilterConfig);
        DAI_DATATYPE_ENUM_CASE(ObjectTrackerConfig);
        DAI_DATATYPE_ENUM_CASE(DynamicCalibrationControl);
        DAI_DATATYPE_ENUM_CASE(DynamicCalibrationResult);
        DAI_DATATYPE_ENUM_CASE(AutoCalibrationConfig);
        DAI_DATATYPE_ENUM_CASE(AutoCalibrationResult);
        DAI_DATATYPE_ENUM_CASE(CalibrationQuality);
        DAI_DATATYPE_ENUM_CASE(CalibrationMetrics);
        DAI_DATATYPE_ENUM_CASE(CoverageData);
        DAI_DATATYPE_ENUM_CASE(SegmentationParserConfig);
        DAI_DATATYPE_ENUM_CASE(PipelineEvent);
        DAI_DATATYPE_ENUM_CASE(PipelineState);
        DAI_DATATYPE_ENUM_CASE(PipelineEventAggregationConfig);
        DAI_DATATYPE_ENUM_CASE(VppConfig);
        DAI_DATATYPE_ENUM_CASE(PacketizedData);
        DAI_DATATYPE_ENUM_CASE(Transformable);
        DAI_DATATYPE_ENUM_CASE(Iterable);
        DAI_DATATYPE_ENUM_CASE(IterableTransformableBuffer);
        DAI_DATATYPE_ENUM_CASE(COUNT);
    }

#undef DAI_DATATYPE_ENUM_CASE

    return "UnknownDatatypeEnum";
}

inline std::string formatMessageNodeLabel(const dai::MessageGroup& messageGroup, uint32_t nodeIndex) {
    const auto node = messageGroup.getNode(nodeIndex);
    std::ostringstream label;
    label << "[" << nodeIndex << "] ";

    if(node == nullptr) {
        label << "<missing>";
        return label.str();
    }

    label << datatypeEnumName(node->getDatatype());
    return label.str();
}

inline std::string visualizeMessageGroupTree(const dai::MessageGroup& messageGroup) {
    return detail::buildMessageGroupRenderGraph(messageGroup).treeText;
}

inline cv::Mat renderMessageGroupTreeFrame(const dai::MessageGroup& messageGroup, const std::string& title = "MessageGroup tree") {
    const auto graph = detail::buildMessageGroupRenderGraph(messageGroup);

    constexpr int kMarginX = 24;
    constexpr int kMarginY = 24;
    constexpr int kTopPadding = 18;
    constexpr int kBottomPadding = 24;
    constexpr int kNodePaddingX = 14;
    constexpr int kNodePaddingY = 10;
    constexpr int kColumnGap = 80;
    constexpr int kRowGap = 22;
    constexpr int kMinFrameWidth = 720;
    constexpr int kMinFrameHeight = 180;
    constexpr int kTitleGap = 24;

    constexpr int kFontFace = cv::FONT_HERSHEY_SIMPLEX;
    constexpr double kTitleScale = 0.7;
    constexpr double kLabelScale = 0.5;
    constexpr double kAnnotationScale = 0.42;
    constexpr double kEdgeScale = 0.42;
    constexpr int kTitleThickness = 2;
    constexpr int kTextThickness = 1;

    int baseline = 0;
    const cv::Size titleSize = cv::getTextSize(title, kFontFace, kTitleScale, kTitleThickness, &baseline);

    if(graph.nodes.empty()) {
        const int width = std::max(kMinFrameWidth, titleSize.width + (2 * kMarginX));
        const int height = std::max(kMinFrameHeight, titleSize.height + 140);
        cv::Mat frame(height, width, CV_8UC3, cv::Scalar(247, 247, 244));

        cv::putText(frame, title, cv::Point(kMarginX, kMarginY + titleSize.height), kFontFace, kTitleScale, cv::Scalar(35, 35, 35), kTitleThickness);
        cv::putText(frame,
                    "<empty>",
                    cv::Point(kMarginX, kMarginY + titleSize.height + kTitleGap + 32),
                    kFontFace,
                    0.6,
                    cv::Scalar(90, 90, 90),
                    kTextThickness);
        return frame;
    }

    int maxNodeLabelWidth = 0;
    int maxNodeLabelHeight = 0;
    int maxNodeAnnotationWidth = 0;
    int maxNodeAnnotationHeight = 0;
    int maxEdgeLabelWidth = 0;
    int maxDepth = 0;
    int maxRow = 0;

    for(const auto& node : graph.nodes) {
        const cv::Size labelSize = cv::getTextSize(node.label, kFontFace, kLabelScale, kTextThickness, &baseline);
        maxNodeLabelWidth = std::max(maxNodeLabelWidth, labelSize.width);
        maxNodeLabelHeight = std::max(maxNodeLabelHeight, labelSize.height);

        if(!node.annotation.empty()) {
            const cv::Size annotationSize = cv::getTextSize(node.annotation, kFontFace, kAnnotationScale, kTextThickness, &baseline);
            maxNodeAnnotationWidth = std::max(maxNodeAnnotationWidth, annotationSize.width);
            maxNodeAnnotationHeight = std::max(maxNodeAnnotationHeight, annotationSize.height);
        }

        maxDepth = std::max(maxDepth, node.depth);
        maxRow = std::max(maxRow, node.row);
    }

    for(const auto& edge : graph.edges) {
        std::string edgeLabel = std::to_string(edge.itemIndex);
        if(edge.isCycle) {
            edgeLabel += " cycle";
        } else if(edge.isShared) {
            edgeLabel += " shared";
        }
        const cv::Size edgeLabelSize = cv::getTextSize(edgeLabel, kFontFace, kEdgeScale, kTextThickness, &baseline);
        maxEdgeLabelWidth = std::max(maxEdgeLabelWidth, edgeLabelSize.width);
    }

    const int nodeWidth = std::max(220, std::max(maxNodeLabelWidth, maxNodeAnnotationWidth) + (2 * kNodePaddingX));
    const int nodeHeight = std::max(50, maxNodeLabelHeight + maxNodeAnnotationHeight + (maxNodeAnnotationHeight > 0 ? 8 : 0) + (2 * kNodePaddingY));
    const int contentTop = kMarginY + titleSize.height + kTitleGap + kTopPadding;
    const int width = std::max(kMinFrameWidth,
                               (2 * kMarginX) + ((maxDepth + 1) * nodeWidth) + (maxDepth * kColumnGap) + maxEdgeLabelWidth + 32);
    const int height = std::max(kMinFrameHeight,
                                contentTop + ((maxRow + 1) * nodeHeight) + (maxRow * kRowGap) + kBottomPadding);

    cv::Mat frame(height, width, CV_8UC3, cv::Scalar(247, 247, 244));

    cv::putText(frame, title, cv::Point(kMarginX, kMarginY + titleSize.height), kFontFace, kTitleScale, cv::Scalar(35, 35, 35), kTitleThickness);

    std::unordered_map<uint32_t, cv::Rect> nodeRects;
    nodeRects.reserve(graph.nodes.size());

    for(const auto& node : graph.nodes) {
        const int x = kMarginX + (node.depth * (nodeWidth + kColumnGap));
        const int y = contentTop + (node.row * (nodeHeight + kRowGap));
        nodeRects.emplace(node.nodeIndex, cv::Rect(x, y, nodeWidth, nodeHeight));
    }

    for(const auto& edge : graph.edges) {
        const auto parentIt = nodeRects.find(edge.parentNodeIndex);
        const auto childIt = nodeRects.find(edge.childNodeIndex);
        if(parentIt == nodeRects.end() || childIt == nodeRects.end()) {
            continue;
        }

        const cv::Rect& parentRect = parentIt->second;
        const cv::Rect& childRect = childIt->second;

        const cv::Point start(parentRect.x + parentRect.width, parentRect.y + (parentRect.height / 2));
        const cv::Point end(childRect.x, childRect.y + (childRect.height / 2));
        const int elbowX = start.x + std::max(24, (end.x - start.x) / 2);

        cv::Scalar edgeColor(115, 115, 115);
        if(edge.isCycle) {
            edgeColor = cv::Scalar(45, 65, 210);
        } else if(edge.isShared) {
            edgeColor = cv::Scalar(0, 140, 220);
        }

        cv::line(frame, start, cv::Point(elbowX, start.y), edgeColor, 2, cv::LINE_AA);
        cv::line(frame, cv::Point(elbowX, start.y), cv::Point(elbowX, end.y), edgeColor, 2, cv::LINE_AA);
        cv::line(frame, cv::Point(elbowX, end.y), end, edgeColor, 2, cv::LINE_AA);

        std::string edgeLabel = std::to_string(edge.itemIndex);
        if(edge.isCycle) {
            edgeLabel += " cycle";
        } else if(edge.isShared) {
            edgeLabel += " shared";
        }

        const int labelX = std::min(elbowX + 6, frame.cols - 4);
        const int labelY = std::max(std::min((start.y + end.y) / 2 - 4, frame.rows - 10), 18);
        cv::putText(frame, edgeLabel, cv::Point(labelX, labelY), kFontFace, kEdgeScale, edgeColor, kTextThickness);
    }

    for(const auto& node : graph.nodes) {
        const cv::Rect& rect = nodeRects.at(node.nodeIndex);

        cv::Scalar fillColor(229, 236, 245);
        cv::Scalar borderColor(79, 100, 136);
        if(node.isUnreachable) {
            fillColor = cv::Scalar(220, 232, 245);
            borderColor = cv::Scalar(52, 102, 168);
        } else if(node.isRoot) {
            fillColor = cv::Scalar(214, 238, 219);
            borderColor = cv::Scalar(58, 128, 71);
        }

        cv::rectangle(frame, rect, fillColor, cv::FILLED, cv::LINE_AA);
        cv::rectangle(frame, rect, borderColor, 2, cv::LINE_AA);

        int currentY = rect.y + kNodePaddingY + maxNodeLabelHeight;
        cv::putText(frame,
                    node.label,
                    cv::Point(rect.x + kNodePaddingX, currentY),
                    kFontFace,
                    kLabelScale,
                    cv::Scalar(30, 30, 30),
                    kTextThickness);

        if(!node.annotation.empty()) {
            currentY += maxNodeAnnotationHeight + 8;
            cv::putText(frame,
                        node.annotation,
                        cv::Point(rect.x + kNodePaddingX, currentY),
                        kFontFace,
                        kAnnotationScale,
                        cv::Scalar(75, 75, 75),
                        kTextThickness);
        }
    }

    return frame;
}

inline bool showMessageGroupTreeIfChanged(const dai::MessageGroup& messageGroup,
                                          std::string& previousVisualization,
                                          const std::string& windowName = "MessageGroup tree") {
    const auto graph = detail::buildMessageGroupRenderGraph(messageGroup);
    if(graph.treeText == previousVisualization) {
        return false;
    }

    previousVisualization = graph.treeText;
    cv::imshow(windowName, renderMessageGroupTreeFrame(messageGroup, windowName));
    return true;
}

inline bool printMessageGroupTreeIfChanged(const dai::MessageGroup& messageGroup,
                                           std::string& previousVisualization,
                                           std::ostream& out,
                                           const std::string& title = "MessageGroup tree") {
    const auto visualization = visualizeMessageGroupTree(messageGroup);
    if(visualization == previousVisualization) {
        return false;
    }

    previousVisualization = visualization;
    out << title << ":\n" << visualization;
    if(visualization.empty() || visualization.back() != '\n') {
        out << '\n';
    }
    return true;
}

namespace detail {

inline MessageGroupRenderGraph buildMessageGroupRenderGraph(const dai::MessageGroup& messageGroup) {
    MessageGroupRenderGraph graph;
    const auto allMessageIndices = messageGroup.getMessageIndices();
    if(allMessageIndices.empty()) {
        graph.treeText = "<empty>\n";
        return graph;
    }

    auto sortedRoots = messageGroup.getRootMessageNodes();
    std::sort(sortedRoots.begin(), sortedRoots.end());
    if(sortedRoots.empty()) {
        sortedRoots = allMessageIndices;
    }

    std::ostringstream output;
    std::unordered_set<uint32_t> renderedNodes;
    std::unordered_set<uint32_t> activePath;
    std::unordered_map<uint32_t, std::size_t> nodePositions;

    auto addGraphNode = [&](uint32_t nodeIndex, int depth, bool isRoot, bool isUnreachable, const std::string& annotation = std::string()) {
        const auto existing = nodePositions.find(nodeIndex);
        if(existing != nodePositions.end()) {
            auto& node = graph.nodes[existing->second];
            node.depth = std::min(node.depth, depth);
            node.isRoot = node.isRoot || isRoot;
            node.isUnreachable = node.isUnreachable || isUnreachable;
            if(node.annotation.empty()) {
                node.annotation = annotation;
            }
            return;
        }

        MessageGroupRenderNode node;
        node.nodeIndex = nodeIndex;
        node.depth = depth;
        node.row = static_cast<int>(graph.nodes.size());
        node.isRoot = isRoot;
        node.isUnreachable = isUnreachable;
        node.label = formatMessageNodeLabel(messageGroup, nodeIndex);
        node.annotation = annotation;

        nodePositions.emplace(nodeIndex, graph.nodes.size());
        graph.nodes.push_back(std::move(node));
    };

    std::function<void(uint32_t, const std::string&, int)> appendChildren = [&](uint32_t nodeIndex, const std::string& prefix, int depth) {
        auto childLinks = messageGroup.getLinksFromParent(nodeIndex);
        std::sort(childLinks.begin(), childLinks.end(), [](const dai::Link& lhs, const dai::Link& rhs) {
            if(lhs.itemIndex != rhs.itemIndex) {
                return lhs.itemIndex < rhs.itemIndex;
            }
            return lhs.childNodeIndex < rhs.childNodeIndex;
        });

        for(size_t childIndex = 0; childIndex < childLinks.size(); ++childIndex) {
            const auto& link = childLinks[childIndex];
            const bool isLastChild = childIndex + 1 == childLinks.size();
            const bool isCycle = activePath.count(link.childNodeIndex) != 0U;
            const bool isFirstRender = !isCycle && renderedNodes.insert(link.childNodeIndex).second;
            const bool isShared = !isCycle && !isFirstRender;

            graph.edges.push_back({nodeIndex, link.childNodeIndex, link.itemIndex, isShared, isCycle});

            output << prefix << (isLastChild ? "`-- " : "|-- ") << "ItemIndex=" << link.itemIndex << " -> "
                   << formatMessageNodeLabel(messageGroup, link.childNodeIndex);

            if(isCycle) {
                output << " (cycle)\n";
                continue;
            }

            if(!isFirstRender) {
                output << " (shared)\n";
                continue;
            }

            output << "\n";
            addGraphNode(link.childNodeIndex, depth + 1, false, false);
            activePath.insert(link.childNodeIndex);
            appendChildren(link.childNodeIndex, prefix + (isLastChild ? "    " : "|   "), depth + 1);
            activePath.erase(link.childNodeIndex);
        }
    };

    bool wroteAnyRoot = false;
    for(uint32_t rootIndex : sortedRoots) {
        if(renderedNodes.count(rootIndex) != 0U) {
            continue;
        }

        if(wroteAnyRoot) {
            output << "\n";
        }

        wroteAnyRoot = true;
        renderedNodes.insert(rootIndex);
        activePath.insert(rootIndex);
        addGraphNode(rootIndex, 0, true, false);
        output << formatMessageNodeLabel(messageGroup, rootIndex) << "\n";
        appendChildren(rootIndex, "", 0);
        activePath.erase(rootIndex);
    }

    auto sortedAllIndices = allMessageIndices;
    std::sort(sortedAllIndices.begin(), sortedAllIndices.end());
    for(uint32_t nodeIndex : sortedAllIndices) {
        if(renderedNodes.count(nodeIndex) != 0U) {
            continue;
        }

        if(wroteAnyRoot) {
            output << "\n";
        }

        wroteAnyRoot = true;
        renderedNodes.insert(nodeIndex);
        activePath.insert(nodeIndex);
        addGraphNode(nodeIndex, 0, true, true, "unreachable-from-roots");
        output << formatMessageNodeLabel(messageGroup, nodeIndex) << " (unreachable-from-roots)\n";
        appendChildren(nodeIndex, "", 0);
        activePath.erase(nodeIndex);
    }

    graph.treeText = output.str();
    return graph;
}

}  // namespace detail

}  // namespace gather_data_examples
