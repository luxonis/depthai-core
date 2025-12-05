//
// Created by thwdpc on 7/28/25.
//

#pragma once
#include <depthai/common/ImgTransformations.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>

namespace dai {

struct ValueWithConfidence {
    float_t value;
    float_t confidence;
};

// Per-dimension confidence, strictly [v0, c0, v1, c1, ...]
template <std::size_t D>
struct KeypointPerDimConfidence {
    ValueWithConfidence data[D];                 // value, confidence, value, confidence, ...
    static constexpr std::size_t value = 2 * D;  // value,conf xD
};

// Per-keypoint confidence, strictly [v0, v1, v2 ... confidence]
template <std::size_t D>
struct KeypointPerKeypointConfidence {
    float_t values[D];
    float_t confidence;
    static constexpr std::size_t value = D + 1;  // D values + 1 confidence
};

using Keypoint2D2C = KeypointPerDimConfidence<2>;
using Keypoint2D1C = KeypointPerKeypointConfidence<2>;
using Keypoint3D3C = KeypointPerDimConfidence<3>;
using Keypoint3D1C = KeypointPerKeypointConfidence<3>;

template <typename KP>
class Keypoints : public Buffer {
   public:
    std::optional<ImgTransformation> transformation;

    std::vector<KP> kpVec;

    Keypoints(std::shared_ptr<NNData>&& other, xt::xarray<float>&& planarStackedKeypoints);
};

template class Keypoints<Keypoint2D1C>;
typedef Keypoints<Keypoint2D1C> Keypoints2D;
template class Keypoints<Keypoint2D2C>;
typedef Keypoints<Keypoint2D2C> Keypoints2D2C;

template class Keypoints<Keypoint3D1C>;
typedef Keypoints<Keypoint3D1C> Keypoints3D;
template class Keypoints<Keypoint3D3C>;
typedef Keypoints<Keypoint3D3C> Keypoints3D3C;
}  // namespace dai