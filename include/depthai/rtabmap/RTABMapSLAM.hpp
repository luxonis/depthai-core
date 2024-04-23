#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/LocalGrid.h"
#include "rtabmap/core/global_map/OccupancyGrid.h"

namespace dai {
namespace node {
class RTABMapSLAM : public dai::NodeCRTP<dai::node::ThreadedHostNode, RTABMapSLAM> {
   public:
    constexpr static const char* NAME = "RTABMapSLAM";

   public:
    void build();

    Input inputRect{*this, {.name="img_rect", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Input inputDepth{*this, {.name="depth", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Input inputIMU{*this, {.name="imu", .types={{dai::DatatypeEnum::IMUData, true}}}};
    Input inputFeatures{*this, {.name="features", .types={{dai::DatatypeEnum::TrackedFeatures, true}}}};
    Input inputOdomPose{*this, {.name="odom_pose", .types={{dai::DatatypeEnum::TransformData, true}}}};

    Output transform{*this, {.name="transform", .types={{dai::DatatypeEnum::TransformData, true}}}};
    Output passthroughRect{*this, {.name="passthrough_rect", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Output pointCloud{*this, {.name="point_cloud", .types={{dai::DatatypeEnum::PointCloudData, true}}}};
    Output occupancyMap{*this, {.name="map", .types={{dai::DatatypeEnum::ImgFrame, true}}}};

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
    std::chrono::steady_clock::time_point startTime;
    rtabmap::Transform imuLocalTransform;
    rtabmap::Transform localTransform;
    std::map<double, cv::Vec3f> accBuffer_;
	std::map<double, cv::Vec3f> gyroBuffer_;
    std::map<double, cv::Vec4f> rotBuffer_;
    rtabmap::LocalGridCache localMaps_;
    rtabmap::OccupancyGrid* grid;
    float alphaScaling;
    bool modelSet = false;
    rtabmap::ParametersMap rtabParams;
};
}  // namespace node
}  // namespace dai