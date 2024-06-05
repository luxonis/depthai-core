#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
namespace dai {
namespace node {
class RTABMapVIO : public NodeCRTP<HostNode, RTABMapVIO> {
   public:
    constexpr static const char* NAME = "RTABMapVIO";

   public:
    std::shared_ptr<RTABMapVIO> build();

    std::string rectInputName = "rect";
    std::string depthInputName = "depth";
    std::string featuresInputName = "features";

    Input& inputRect = inputs[rectInputName];
    Input& inputDepth = inputs[depthInputName];
    Input inputFeatures{*this, {.name = featuresInputName, .types = {{DatatypeEnum::TrackedFeatures, true}}}};
    Input inputIMU{*this, {.name = "imu", .types = {{DatatypeEnum::IMUData, true}}}};
    Input input{*this, {.name = "in", .types = {{DatatypeEnum::MessageGroup, true}}}};
    Output transform{*this, {.name = "transform", .types = {{DatatypeEnum::TransformData, true}}}};
    Output passthroughRect{*this, {.name = "passthrough_rect", .types = {{DatatypeEnum::ImgFrame, true}}}};
    Output passthroughDepth{*this, {.name = "passthrough_depth", .types = {{DatatypeEnum::ImgFrame, true}}}};
    Output passthroughFeatures{*this, {.name = "passthrough_features", .types = {{DatatypeEnum::TrackedFeatures, true}}}};

    void setParams(const rtabmap::ParametersMap& params);
    void setUseFeatures(bool use) { useFeatures = use; }
    void setLocalTransform(std::shared_ptr<TransformData> transform) { transform->getRTABMapTransform(localTransform); }
   private:
    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override;
    void imuCB(std::shared_ptr<ADatatype> msg);
    void getCalib(Pipeline& pipeline, int instanceNum, int width, int height);
    rtabmap::StereoCameraModel model;
    std::unique_ptr<rtabmap::Odometry> odom;
    rtabmap::Transform localTransform;
    rtabmap::Transform imuLocalTransform;
    std::map<double, cv::Vec3f> accBuffer_;
    std::map<double, cv::Vec3f> gyroBuffer_;
    std::map<double, cv::Vec4f> rotBuffer_;
    float alphaScaling;
    bool modelSet = false;
    bool useFeatures = true;

};
}  // namespace node
}  // namespace dai