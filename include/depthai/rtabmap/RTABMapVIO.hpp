#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"

namespace dai {
namespace node {

/**
 * @brief RTABMap Visual Inertial Odometry node. Performs VIO on rectified frame, depth frame and IMU data.

*/
class RTABMapVIO : public NodeCRTP<ThreadedHostNode, RTABMapVIO> {
   public:
    constexpr static const char* NAME = "RTABMapVIO";

    std::shared_ptr<RTABMapVIO> build();

    std::string rectInputName = "rect";
    std::string depthInputName = "depth";
    std::string featuresInputName = "features";

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    /**
     * Input rectified image on which VIO is performed.
     */
    Input& rect = inputs[rectInputName];
    /**
     * Input depth image on which VIO is performed.
     */
    Input& depth = inputs[depthInputName];
    /**
     * Input tracked features on which VIO is performed (optional).
     */
    Input features{*this, {.name = featuresInputName, .types = {{DatatypeEnum::TrackedFeatures, true}}}};
    /**
     * Input IMU data.
     */
    Input imu{*this, {.name = "imu", .types = {{DatatypeEnum::IMUData, true}}}};
    /**
     * Output transform.
     */
    Output transform{*this, {.name = "transform", .types = {{DatatypeEnum::TransformData, true}}}};
    /**
     * Passthrough rectified frame.
     */
    Output passthroughRect{*this, {.name = "passthroughRect", .types = {{DatatypeEnum::ImgFrame, true}}}};
    /**
     * Passthrough depth frame.
     */
    Output passthroughDepth{*this, {.name = "passthroughDepth", .types = {{DatatypeEnum::ImgFrame, true}}}};
    /**
     * Passthrough features.
     */
    Output passthroughFeatures{*this, {.name = "passthroughFeatures", .types = {{DatatypeEnum::TrackedFeatures, true}}}};

    /**
     * Set RTABMap parameters.
     */
    void setParams(const std::map<std::string, std::string>& params);
    /**
     * Whether to use input features or calculate them internally.
     */
    void setUseFeatures(bool use);

    void setLocalTransform(std::shared_ptr<TransformData> transform) {
        localTransform = transform->getRTABMapTransform();
    }

    /**
     * Reset Odometry.
     */
    void reset(std::shared_ptr<TransformData> transform = nullptr);

   private:
    void run() override;
    void syncCB(std::shared_ptr<dai::ADatatype> data);
    Input inSync{*this, {.name = "inSync", .types = {{DatatypeEnum::MessageGroup, true}}}};
    void imuCB(std::shared_ptr<ADatatype> msg);
    void initialize(Pipeline& pipeline, int instanceNum, int width, int height);
    rtabmap::StereoCameraModel model;
    std::unique_ptr<rtabmap::Odometry> odom;
    rtabmap::Transform localTransform;
    rtabmap::Transform imuLocalTransform;
    std::map<std::string, std::string> rtabParams;
    std::map<double, cv::Vec3f> accBuffer;
    std::map<double, cv::Vec3f> gyroBuffer;
    std::mutex imuMtx;
    float alphaScaling = -1.0;
    bool initialized = false;
    bool useFeatures = true;
};
}  // namespace node
}  // namespace dai