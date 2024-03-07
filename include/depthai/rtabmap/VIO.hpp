#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
namespace dai {
namespace node {
class VisualOdometry : public dai::NodeCRTP<dai::ThreadedNode, VisualOdometry> {
   public:
    constexpr static const char* NAME = "VisualOdometry";

   public:
    void build();

    /**
     * Input for any ImgFrame messages to be displayed
     * Default queue is non-blocking with size 8
     */
    Input inputRect{true, *this, "img_rect", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::ImgFrame, true}}};
    Input inputDepth{true, *this, "depth", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::ImgFrame, true}}};
    Input inputIMU{true, *this, "imu", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::IMUData, true}}};
    Input inputFeatures{true, *this, "features", Input::Type::SReceiver, false, 8, true, {{dai::DatatypeEnum::TrackedFeatures, true}}};

    Output transform{true, *this, "transform", Output::Type::MSender, {{dai::DatatypeEnum::TransformData, true}}};

    void run() override;
    void getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height);

   private:
    rtabmap::StereoCameraModel model;
    rtabmap::Odometry* odom;
    rtabmap::OdometryInfo info;
    rtabmap::Transform imuLocalTransform;
    bool modelSet = false;
};
}  // namespace node
}  // namespace dai