#pragma once

#include <array>
#include <vector>

#include "depthai/common/Point3d.hpp"
#include "depthai/common/Quaterniond.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

// optional
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core/mat.hpp>
    #include <opencv2/opencv.hpp>
#endif



// #include "utility/Serialization.hpp"

namespace dai {
struct Landmark {
    /**
     * The ID of the Landmark
     */
    int id;

    /**
     * The size of the landmark in meters
     */
    double size;

    /**
     * The translation of the landmark reletive to base_frame (the camera)
     */
    dai::Point3d translation;

    /**
     * The orientation of the landmark relative to base_frame (the camera)
     */
    dai::Quaterniond quaternion;

    /**
     * The covariance of the landmark. If you do not know what this means, you can probably just fill with 0.01.
     */
    std::array<std::array<double, 6>, 6> covariance;
};

DEPTHAI_SERIALIZE_EXT(Landmark, id, size, translation, quaternion, covariance);

class Landmarks : public Buffer {
   public:
    Landmarks() = default;
    virtual ~Landmarks() = default;

    #ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    cv::Mat getCovarianceAsCVMat(Landmark landmark);
    #endif


   public:
    std::vector<Landmark> landmarks;

    DEPTHAI_SERIALIZE(Landmarks, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, landmarks);
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::Landmarks;
    }
};

}  // namespace dai
