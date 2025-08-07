//
// Created by thwdpc on 7/28/25.
//

#include "messages/Keypoints.hpp"

#include <spdlog/spdlog.h>

#include "utility/ErrorMacros.hpp"

namespace dai {
template <typename KP>
Keypoints<KP>::Keypoints(std::shared_ptr<NNData>&& other, xt::xarray<float>&& planarStackedKeypoints){
    // KP#, dim
    const size_t numKeypoints = planarStackedKeypoints.shape()[0], numDimsFound = planarStackedKeypoints.shape()[1];

    DAI_CHECK_V(numDimsFound == KP::value,
                "Trying to build {} dimensional keypoints, got {} sets of keypoints/confidence values",
                KP::value,
                numDimsFound);

    kpVec = std::vector<KP>(numKeypoints);
    // Direct copy into the vec
    assert(sizeof(KP) == sizeof(float) * KP::value);
    assert(planarStackedKeypoints.size() == numKeypoints * KP::value);
    std::memcpy(kpVec.data(), planarStackedKeypoints.data(), planarStackedKeypoints.size() * sizeof(float));

    transformation = other->transformation;
    setTimestamp(other->getTimestamp());
    setSequenceNum(other->sequenceNum);
}



}  // namespace dai