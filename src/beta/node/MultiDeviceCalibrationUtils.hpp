#pragma once

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "depthai/utility/matrixOps.hpp"

namespace dai {
namespace beta {
namespace node {
namespace detail {

using MultiDeviceTransform = std::vector<std::vector<float>>;

struct PairwiseDeviceTransform {
    std::string fromDeviceId;
    std::string toDeviceId;
    MultiDeviceTransform fromToTo;
};

inline MultiDeviceTransform identityTransform() {
    return {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
}

inline MultiDeviceTransform inverseRigidTransform(MultiDeviceTransform transform) {
    matrix::invertSe3Matrix4x4InPlace(transform);
    return transform;
}

inline MultiDeviceTransform composeRigidTransforms(const MultiDeviceTransform& lhs, const MultiDeviceTransform& rhs) {
    return matrix::matMul(lhs, rhs);
}

inline std::map<std::string, MultiDeviceTransform> makeReferenceRelativeTransforms(const std::string& referenceDeviceId,
                                                                                   const std::vector<PairwiseDeviceTransform>& pairwiseTransforms) {
    std::map<std::string, MultiDeviceTransform> referenceToDevice{{referenceDeviceId, identityTransform()}};
    while(true) {
        bool madeProgress = false;
        for(const auto& pairwise : pairwiseTransforms) {
            const bool hasFrom = referenceToDevice.count(pairwise.fromDeviceId) != 0;
            const bool hasTo = referenceToDevice.count(pairwise.toDeviceId) != 0;
            if(hasFrom && !hasTo) {
                referenceToDevice[pairwise.toDeviceId] = composeRigidTransforms(pairwise.fromToTo, referenceToDevice.at(pairwise.fromDeviceId));
                madeProgress = true;
            } else if(!hasFrom && hasTo) {
                referenceToDevice[pairwise.fromDeviceId] =
                    composeRigidTransforms(inverseRigidTransform(pairwise.fromToTo), referenceToDevice.at(pairwise.toDeviceId));
                madeProgress = true;
            }
        }
        if(madeProgress) continue;

        const auto disconnected = std::find_if(pairwiseTransforms.begin(), pairwiseTransforms.end(), [&](const PairwiseDeviceTransform& pairwise) {
            return referenceToDevice.count(pairwise.fromDeviceId) == 0 && referenceToDevice.count(pairwise.toDeviceId) == 0;
        });
        if(disconnected == pairwiseTransforms.end()) break;

        // Initial guesses are seeds, not constraints. Give each disconnected
        // guess component an arbitrary gauge so all of its relative poses are retained.
        referenceToDevice.emplace(disconnected->fromDeviceId, identityTransform());
    }
    return referenceToDevice;
}

}  // namespace detail
}  // namespace node
}  // namespace beta
}  // namespace dai
