#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
namespace dai {
namespace node {
class RTABMapSLAM : public dai::NodeCRTP<dai::ThreadedNode, RTABMapSLAM> {
   public:
    constexpr static const char* NAME = "RTABMapSLAM";

   public:
    void build();


    Input inputRect{true, *this, "img_rect", Input::Type::SReceiver, false, 8, false, {{dai::DatatypeEnum::ImgFrame, true}}};
    Input inputDepth{true, *this, "depth", Input::Type::SReceiver, false, 8, false, {{dai::DatatypeEnum::ImgFrame, true}}};
    Input inputIMU{true, *this, "imu", Input::Type::SReceiver, false, 8, false, {{dai::DatatypeEnum::IMUData, true}}};
    Input inputFeatures{true, *this, "features", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::TrackedFeatures, true}}};
    Input inputOdomPose{true, *this, "odom_pose", Input::Type::SReceiver, false, 8, false, {{dai::DatatypeEnum::TransformData, true}}};
    Output transform{true, *this, "transform", Output::Type::MSender, {{dai::DatatypeEnum::TransformData, true}}};
    Output passthroughRect{true, *this, "passthrough_rect", Output::Type::MSender, {{dai::DatatypeEnum::ImgFrame, true}}};
    void run() override;
    void stop() override;
    void setParams(const rtabmap::ParametersMap& params);
   private:
    void imuCB(std::shared_ptr<dai::ADatatype> msg);
    void getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height);
    rtabmap::StereoCameraModel model;
    rtabmap::Rtabmap rtabmap;
    rtabmap::Transform odomCorrection;
    bool reuseFeatures;
    std::chrono::steady_clock::time_point lastProcessTime; 
    rtabmap::Transform imuLocalTransform;
    std::map<double, cv::Vec3f> accBuffer_;
	std::map<double, cv::Vec3f> gyroBuffer_;
    std::map<double, cv::Vec4f> rotBuffer_;
    float alphaScaling;
    bool modelSet = false;
};
}  // namespace node
}  // namespace dai