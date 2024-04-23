#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
namespace dai {
namespace node {
class RTABMapVIO : public dai::NodeCRTP<dai::node::ThreadedHostNode, RTABMapVIO> {
   public:
    constexpr static const char* NAME = "RTABMapVIO";

   public:
    void build();

    /**
     * Input for any ImgFrame messages to be displayed
     * Default queue is non-blocking with size 8
     */
    Input inputRect{*this, {.name="img_rect", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Input inputDepth{*this, {.name="depth", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Input inputIMU{*this, {.name="imu", .types={{dai::DatatypeEnum::IMUData, true}}}};
    Input inputFeatures{*this, {.name="features", .types={{dai::DatatypeEnum::TrackedFeatures, true}}}};
    Input inputReset{*this, {.name="reset", .types={{dai::DatatypeEnum::CameraControl, true}}}};

    Output transform{*this, {.name="transform", .types={{dai::DatatypeEnum::TransformData, true}}}};
    Output passthroughRect{*this, {.name="passthrough_rect", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Output passthroughDepth{*this, {.name="passthrough_depth", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Output passthroughFeatures{*this, {.name="passthrough_features", .types={{dai::DatatypeEnum::TrackedFeatures, true}}}};
    
    void run() override;
    void stop() override;
    void setParams(const rtabmap::ParametersMap& params);
   private:
    void imuCB(std::shared_ptr<dai::ADatatype> msg);
    void getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height);
    rtabmap::StereoCameraModel model;
    rtabmap::Odometry* odom;
    rtabmap::OdometryInfo info;
    rtabmap::Transform imuLocalTransform;
    std::map<double, cv::Vec3f> accBuffer_;
	std::map<double, cv::Vec3f> gyroBuffer_;
    std::map<double, cv::Vec4f> rotBuffer_;
    float alphaScaling;
    bool modelSet = false;
};
}  // namespace node
}  // namespace dai