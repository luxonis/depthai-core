#include "beta/utilities/LaneDetection/LaneDetectionUtils.hpp"

#include <fmt/ranges.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace LaneDetectionUtils {

std::vector<std::vector<Point2f>> decodeUfld(const std::vector<float>& values,
                                             const std::vector<std::size_t>& dims,
                                             const std::vector<std::int64_t>& rowAnchors,
                                             std::int64_t gridingNum,
                                             std::int64_t clsNumPerLane,
                                             std::uint32_t inputWidth,
                                             std::uint32_t inputHeight) {
    DAI_CHECK(gridingNum > 1, "Griding number must be greater than 1.");
    DAI_CHECK(clsNumPerLane > 0, "Number of points per lane must be greater than 0.");
    DAI_CHECK(inputWidth > 0 && inputHeight > 0, "Input size must be greater than 0.");

    const auto gridingRows = static_cast<std::size_t>(gridingNum) + 1;
    const auto numRowAnchors = static_cast<std::size_t>(clsNumPerLane);
    DAI_CHECK_V(dims.size() == 4 && dims[0] >= 1 && dims[1] == gridingRows && dims[2] == numRowAnchors,
                "Expected a 4D output tensor of shape (batch, gridingNum + 1, clsNumPerLane, numLanes) = (N, {}, {}, L), got shape ({}).",
                gridingRows,
                numRowAnchors,
                fmt::join(dims, ", "));
    DAI_CHECK_V(rowAnchors.size() >= numRowAnchors, "Expected at least clsNumPerLane = {} row anchors, got {} row anchors.", numRowAnchors, rowAnchors.size());

    const std::size_t numLanes = dims[3];
    const auto y = [&](std::size_t g, std::size_t c, std::size_t l) { return values[(g * numRowAnchors + c) * numLanes + l]; };  // Decodes only batch entry 0
    const auto outJ = [&](std::size_t g, std::size_t c, std::size_t l) { return y(g, numRowAnchors - 1 - c, l); };

    std::vector<double> loc(numRowAnchors * numLanes, 0.0);
    std::vector<float> expValues(static_cast<std::size_t>(gridingNum));
    for(std::size_t c = 0; c < numRowAnchors; c++) {
        for(std::size_t l = 0; l < numLanes; l++) {
            float expSum = 0.0f;  // No max-subtraction and single precision, as the source softmax()
            for(std::size_t g = 0; g < static_cast<std::size_t>(gridingNum); g++) {
                expValues[g] = std::exp(outJ(g, c, l));
                expSum += expValues[g];
            }
            double location = 0.0;
            for(std::size_t g = 0; g < static_cast<std::size_t>(gridingNum); g++) {
                const float prob = expValues[g] / expSum;
                location += static_cast<double>(prob) * static_cast<double>(g + 1);
            }

            std::size_t argMax = 0;
            float maxValue = outJ(0, c, l);
            for(std::size_t g = 1; g < gridingRows; g++) {
                const float value = outJ(g, c, l);
                if(value > maxValue) {
                    maxValue = value;
                    argMax = g;
                }
            }
            if(argMax == static_cast<std::size_t>(gridingNum)) {
                location = 0.0;
            }
            loc[c * numLanes + l] = location;
        }
    }

    const double colSampleW = static_cast<double>(inputWidth - 1) / static_cast<double>(gridingNum - 1);

    std::vector<std::vector<Point2f>> points(numLanes);
    for(std::size_t l = 0; l < numLanes; l++) {
        std::size_t nonZeroCount = 0;
        for(std::size_t c = 0; c < numRowAnchors; c++) {
            if(loc[c * numLanes + l] != 0.0) {
                nonZeroCount++;
            }
        }
        if(nonZeroCount <= 2) {
            continue;
        }
        for(std::size_t c = 0; c < numRowAnchors; c++) {
            const double location = loc[c * numLanes + l];
            if(location > 0.0) {
                const auto x = static_cast<std::int64_t>(location * colSampleW) - 1;  // Keeps the source double round-trip through the normalized anchor
                const auto anchor = static_cast<double>(rowAnchors[numRowAnchors - 1 - c]);
                const auto yCoord = static_cast<std::int64_t>(static_cast<double>(inputHeight) * (anchor / static_cast<double>(inputHeight))) - 1;
                points[l].emplace_back(static_cast<float>(static_cast<double>(x) / static_cast<double>(inputWidth)),
                                       static_cast<float>(static_cast<double>(yCoord) / static_cast<double>(inputHeight)));
            }
        }
    }

    return points;
}

std::shared_ptr<Clusters> createClustersMessage(const std::vector<std::vector<Point2f>>& points) {
    auto message = std::make_shared<Clusters>();
    message->clusters.reserve(points.size());
    for(std::size_t i = 0; i < points.size(); i++) {
        Cluster cluster;
        cluster.label = static_cast<std::int32_t>(i);
        cluster.points = points[i];
        message->clusters.push_back(std::move(cluster));
    }
    return message;
}

}  // namespace LaneDetectionUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
