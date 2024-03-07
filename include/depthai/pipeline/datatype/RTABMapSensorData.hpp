#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/common/Point3f.hpp"


namespace dai {

struct CameraModel{
    std::string name;
    int imageWidth;
    int imageHeight;
    float cx;
    float cy;
    float fx;
    float fy;
    float baseline;
    Transform localTransform;
};
DEPTHAI_SERIALIZE_EXT(CameraModel, name, imageWidth, imageHeight, cx, cy, fx, fy, baseline, localTransform);

struct SensorData {
    int id;
    double stamp;
    std::vector<CameraModel> cameraModels;
    ImgFrame imageRaw;
    ImgFrame depthImage;
    TrackedFeatures trackedFeatures;
    std::vector<Point3f> keypoints3D;
    std::vector<uint8_t> descriptors;
    Transform globalTransform;
    Transform groundTruth;
    IMUData imuData;
};

DEPTHAI_SERIALIZE_EXT(SensorData, id, stamp, cameraModels, imageRaw, depthImage, trackedFeatures, keypoints3D, descriptors, globalTransform, groundTruth, imuData);



/**
 * RTABMapSensorData message. Carries RTABMap sensor data
 */
class RTABMapSensorData : public Buffer {
   public:
    /**
     * Construct TransformData message.
     */
    RTABMapSensorData() = default;
    virtual ~RTABMapSensorData() = default;

    /// Data
    SensorData data;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::RTABMapSensorData;
    };

    DEPTHAI_SERIALIZE(RTABMapSensorData, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, data);
};

}  // namespace dai
