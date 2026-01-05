
#pragma once
#include "KeypointParser.hpp"
#include "parsers/BaseParser.hpp"

namespace dai::node {


class SimCCKeypointParser final : public NodeCRTP<KeypointParser, SimCCKeypointParser> {
   public:
    constexpr static const char* NAME = "SimCCKeypointParser";

   protected:
    void buildImpl(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model) override;
    void run() override;
    void foggyGuessesForOneDim(const nn_archive::v1::Head& head,
                               const nn_archive::v1::Model& model,
                               const nn_archive::v1::Input& imgInput,
                               const std::pair<std::optional<int64_t>, std::optional<int64_t>>& imgWHMaybe);
    void inferConfigFromMultipleOutputs(const nn_archive::v1::Head& head,
                                        const nn_archive::v1::Model& model,
                                        const nn_archive::v1::Input& imgInput,
                                        std::pair<std::optional<int64_t>, std::optional<int64_t>>& imgWidthHeight);

    uint8_t pixelSubdivisions = 2;
    // Populated if the keypoint # dim and the XY(Z) dimensionality are collapsed(like yolo). Stores whether collapsed dim is interleaved(x1, x2 .. y1, y2 .. z1, z2)
    //  or planar(x1, y1, z1, x2, y2, z2)
    std::optional<bool> collapsedDimsAreInterleaved = std::nullopt;
    bool replicateXDimToZDim = true;
    std::vector<uint16_t> simCCDimLengths;
};
}  // namespace dai::node
