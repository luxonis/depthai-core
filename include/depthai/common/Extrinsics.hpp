#pragma once

#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/utility/Serialization.hpp"
#include <fmt/format.h>
namespace dai {

/// Extrinsics structure
struct Extrinsics {
    std::vector<std::vector<float>> rotationMatrix;
    /**
     *  (x, y, z) pose of destCameraSocket w.r.t currentCameraSocket obtained through calibration
     */
    Point3f translation;
    /**
     *  (x, y, z) pose of destCameraSocket w.r.t currentCameraSocket measured through CAD design
     */
    Point3f specTranslation;
    CameraBoardSocket toCameraSocket = CameraBoardSocket::AUTO;

    /**
    * Get the 4x4 transformation matrix from this extrinsics
    * @return 4x4 transformation matrix
    */
    std::vector<std::vector<float>> getTransformationMatrix(bool useSpecTranslation = false) const {
        std::vector<std::vector<float>> transformMatrix(4, std::vector<float>(4, 0.0f));
        // Fill in the rotation part
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                transformMatrix[i][j] = rotationMatrix[i][j];
            }
        }
        if(useSpecTranslation) {
            transformMatrix[0][3] = specTranslation.x;
            transformMatrix[1][3] = specTranslation.y;
            transformMatrix[2][3] = specTranslation.z;
        } else {
            // Fill in the translation part
            transformMatrix[0][3] = translation.x;
            transformMatrix[1][3] = translation.y;
            transformMatrix[2][3] = translation.z;
        }
        transformMatrix[3][3] = 1.0f;
        return transformMatrix;
    }

    void setTransformationMatrix(std::vector<std::vector<float>> matrix) {
        if(matrix.size() != 4 || matrix[0].size() != 4) {
            throw std::invalid_argument(fmt::format("Invalid transformation matrix size: {}x{}", matrix.size(), matrix[0].size()));
        }
        rotationMatrix = {
            {matrix[0][0], matrix[0][1], matrix[0][2]},
            {matrix[1][0], matrix[1][1], matrix[1][2]},
            {matrix[2][0], matrix[2][1], matrix[2][2]}
        };
        translation = {matrix[0][3], matrix[1][3], matrix[2][3]};
        specTranslation = translation;
    }
    DEPTHAI_SERIALIZE(Extrinsics, rotationMatrix, translation, specTranslation, toCameraSocket);
};

}  // namespace dai