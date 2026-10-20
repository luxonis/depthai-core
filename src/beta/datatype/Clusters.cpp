#include "depthai/beta/datatype/Clusters.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

namespace dai {
namespace beta {

namespace {

constexpr float CLUSTER_THICKNESS = 2.0f;

// Rainbow colormap base samples, taken from OpenCV's COLORMAP_RAINBOW
// (opencv/modules/imgproc/src/colormap.cpp, Apache License 2.0), which itself equals the GNU
// Octave colormap "rainbow". The 64 base samples are linearly interpolated to a 256-entry
// 8-bit lookup table exactly like OpenCV's ColorMap::linear_colormap, so the visualization
// colors match the source implementation in depthai-nodes, which uses
// cv2.applyColorMap(..., cv2.COLORMAP_RAINBOW).
constexpr std::size_t RAINBOW_BASE_SAMPLES = 64;
// clang-format off
constexpr std::array<float, RAINBOW_BASE_SAMPLES> RAINBOW_R = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.9365079365079367f, 0.8571428571428572f,
    0.7777777777777777f, 0.6984126984126986f, 0.6190476190476191f, 0.53968253968254f, 0.4603174603174605f,
    0.3809523809523814f, 0.3015873015873018f, 0.2222222222222223f, 0.1428571428571432f, 0.06349206349206415f, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0.03174603174603208f, 0.08465608465608465f, 0.1375661375661377f, 0.1904761904761907f,
    0.2433862433862437f, 0.2962962962962963f, 0.3492063492063493f, 0.4021164021164023f, 0.4550264550264553f,
    0.5079365079365079f, 0.5608465608465609f, 0.6137566137566139f, 0.666666666666667f};
constexpr std::array<float, RAINBOW_BASE_SAMPLES> RAINBOW_G = {
    0, 0.03968253968253968f, 0.07936507936507936f, 0.119047619047619f, 0.1587301587301587f, 0.1984126984126984f,
    0.2380952380952381f, 0.2777777777777778f, 0.3174603174603174f, 0.3571428571428571f, 0.3968253968253968f,
    0.4365079365079365f, 0.4761904761904762f, 0.5158730158730158f, 0.5555555555555556f, 0.5952380952380952f,
    0.6349206349206349f, 0.6746031746031745f, 0.7142857142857142f, 0.753968253968254f, 0.7936507936507936f,
    0.8333333333333333f, 0.873015873015873f, 0.9126984126984127f, 0.9523809523809523f, 0.992063492063492f, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 0.9841269841269842f, 0.9047619047619047f, 0.8253968253968256f, 0.7460317460317465f,
    0.666666666666667f, 0.587301587301587f, 0.5079365079365079f, 0.4285714285714288f, 0.3492063492063493f,
    0.2698412698412698f, 0.1904761904761907f, 0.1111111111111116f, 0.03174603174603208f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0};
constexpr std::array<float, RAINBOW_BASE_SAMPLES> RAINBOW_B = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0.01587301587301582f, 0.09523809523809534f, 0.1746031746031744f, 0.2539682539682535f, 0.333333333333333f,
    0.412698412698413f, 0.4920634920634921f, 0.5714285714285712f, 0.6507936507936507f, 0.7301587301587302f,
    0.8095238095238093f, 0.8888888888888884f, 0.9682539682539679f, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
// clang-format on

/// A rainbow colormap color as 8-bit (blue, green, red), matching the OpenCV BGR lookup table.
struct RainbowBgr {
    std::uint8_t b, g, r;
};

/// Linear interpolation of the 64 base samples at xi, replicating OpenCV's interp1 in
/// single-precision arithmetic.
float interpolateRainbowChannel(const std::array<float, RAINBOW_BASE_SAMPLES>& samples, float xi) {
    constexpr float step = 1.0f / static_cast<float>(RAINBOW_BASE_SAMPLES - 1);
    std::size_t low = 0;
    std::size_t high = RAINBOW_BASE_SAMPLES - 1;
    if(xi < 0.0f) {
        high = 1;
    }
    if(xi > static_cast<float>(RAINBOW_BASE_SAMPLES - 1) * step) {
        low = high - 1;
    }
    while((high - low) > 1) {
        const std::size_t c = low + ((high - low) >> 1);
        if(xi > static_cast<float>(c) * step) {
            low = c;
        } else {
            high = c;
        }
    }
    const float xLow = static_cast<float>(low) * step;
    const float xHigh = static_cast<float>(high) * step;
    return samples[low] + (xi - xLow) * (samples[high] - samples[low]) / (xHigh - xLow);
}

/// Convert a [0, 1] float channel value to 8-bit exactly like OpenCV's convertTo(CV_8U, 255.0):
/// scale in single precision and round to nearest, ties to even.
std::uint8_t toLutEntry(float value) {
    const float scaled = value * 255.0f;
    long rounded = std::lrintf(scaled);  // Default FE_TONEAREST rounding: half to even.
    if(rounded < 0) rounded = 0;
    if(rounded > 255) rounded = 255;
    return static_cast<std::uint8_t>(rounded);
}

/// 256-entry rainbow BGR lookup table, computed once.
const std::array<RainbowBgr, 256>& getRainbowLut() {
    static const std::array<RainbowBgr, 256> lut = [] {
        std::array<RainbowBgr, 256> table{};
        constexpr float xiStep = 1.0f / 255.0f;
        for(std::size_t i = 0; i < table.size(); i++) {
            const float xi = static_cast<float>(i) * xiStep;
            table[i].b = toLutEntry(interpolateRainbowChannel(RAINBOW_B, xi));
            table[i].g = toLutEntry(interpolateRainbowChannel(RAINBOW_G, xi));
            table[i].r = toLutEntry(interpolateRainbowChannel(RAINBOW_R, xi));
        }
        return table;
    }();
    return lut;
}

}  // namespace

Clusters::~Clusters() = default;

void Clusters::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

void Clusters::transformToInternal(const ImgTransformation& target) {
    if(!getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform clusters.");
    }
    ImgTransformation source = *getTransformation();
    for(auto& cluster : clusters) {
        for(auto& point : cluster.points) {
            point = source.remapPointTo(target, point);
        }
    }
    setTransformation(target);
}

Clusters Clusters::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<Clusters>::transformTo(target);
}

dai::VisualizeType Clusters::getVisualizationMessage() const {
    ImgAnnotation annotation;

    // Each cluster is drawn as one POINTS annotation. Cluster i samples the rainbow lookup
    // table at index i * (255 / numClusters), replicating the source implementation's
    // np.array(range(0, 255, 255 // num_clusters)) color mask. The source assigns the BGR
    // colormap triple directly to the (r, g, b) color channels; that assignment is reproduced
    // here to keep the emitted colors identical.
    const std::size_t numClusters = clusters.size();
    if(numClusters > 0) {
        const std::size_t step = 255 / numClusters;
        if(step == 0) {
            throw std::runtime_error("Cannot visualize more than 255 clusters.");
        }
        const auto& lut = getRainbowLut();
        for(std::size_t i = 0; i < numClusters; i++) {
            const auto& bgr = lut[i * step];
            const Color color(static_cast<float>(bgr.b / 255.0), static_cast<float>(bgr.g / 255.0), static_cast<float>(bgr.r / 255.0), 1.0f);
            PointsAnnotation pointsAnnotation;
            pointsAnnotation.type = PointsAnnotationType::POINTS;
            pointsAnnotation.points = clusters[i].points;
            pointsAnnotation.outlineColor = color;
            pointsAnnotation.fillColor = color;
            pointsAnnotation.thickness = CLUSTER_THICKNESS;
            annotation.points.push_back(std::move(pointsAnnotation));
        }
    }

    auto annotations = std::make_shared<ImgAnnotations>();
    annotations->annotations.push_back(std::move(annotation));
    annotations->setTimestamp(getTimestamp());
    annotations->setSequenceNum(getSequenceNum());
    annotations->setTimestampDevice(getTimestampDevice());
    return annotations;
}

}  // namespace beta
}  // namespace dai
