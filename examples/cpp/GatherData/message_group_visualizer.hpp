#pragma once

#include <algorithm>
#include <cstdint>
#include <functional>
#include <ostream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"

namespace gather_data_examples {

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
    const auto allMessageIndices = messageGroup.getMessageIndices();
    if(allMessageIndices.empty()) {
        return "<empty>\n";
    }

    auto sortedRoots = messageGroup.getRootMessageNodes();
    std::sort(sortedRoots.begin(), sortedRoots.end());
    if(sortedRoots.empty()) {
        sortedRoots = allMessageIndices;
    }

    std::ostringstream output;
    std::unordered_set<uint32_t> renderedNodes;
    std::unordered_set<uint32_t> activePath;

    std::function<void(uint32_t, const std::string&)> appendChildren = [&](uint32_t nodeIndex, const std::string& prefix) {
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

            output << prefix << (isLastChild ? "`-- " : "|-- ") << "ItemIndex=" << link.itemIndex << " -> "
                   << formatMessageNodeLabel(messageGroup, link.childNodeIndex);

            if(activePath.count(link.childNodeIndex) != 0U) {
                output << " (cycle)\n";
                continue;
            }

            const bool isFirstRender = renderedNodes.insert(link.childNodeIndex).second;
            if(!isFirstRender) {
                output << " (shared)\n";
                continue;
            }

            output << "\n";
            activePath.insert(link.childNodeIndex);
            appendChildren(link.childNodeIndex, prefix + (isLastChild ? "    " : "|   "));
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
        output << formatMessageNodeLabel(messageGroup, rootIndex) << "\n";
        appendChildren(rootIndex, "");
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
        output << formatMessageNodeLabel(messageGroup, nodeIndex) << " (unreachable-from-roots)\n";
        appendChildren(nodeIndex, "");
        activePath.erase(nodeIndex);
    }

    return output.str();
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

}  // namespace gather_data_examples
