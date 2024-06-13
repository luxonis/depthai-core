#pragma once

#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/LocalGrid.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/global_map/CloudMap.h"
#include "rtabmap/core/global_map/OccupancyGrid.h"

namespace dai {
namespace node {

/**
 * @brief RTABMap SLAM node. Performs SLAM on given odometry pose, rectified frame and depth frame.

*/
class RTABMapSLAM : public dai::NodeCRTP<dai::node::ThreadedHostNode, RTABMapSLAM> {
   public:
    constexpr static const char* NAME = "RTABMapSLAM";
    ~RTABMapSLAM() override;
    std::shared_ptr<RTABMapSLAM> build();
    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;
    std::string rectInputName = "rect";
    std::string depthInputName = "depth";
    std::string featuresInputName = "features";

    /**
     * Input rectified image on which SLAM is performed.
     */
    Input& rect = inputs[rectInputName];
    /**
     * Input depth image on which SLAM is performed.
     */
    Input& depth = inputs[depthInputName];
    /**
     * Input tracked features on which SLAM is performed (optional).
     */
    Input features{*this, {.name = featuresInputName, .types = {{DatatypeEnum::TrackedFeatures, true}}}};
    /**
     * Input odometry pose.
     */
    Input odom{*this, {.name = "inputOdomPose", .types = {{dai::DatatypeEnum::TransformData, true}}}};

    /**
     * Output transform.
     */
    Output transform{*this, {.name = "transform", .types = {{dai::DatatypeEnum::TransformData, true}}}};
    /**
     * Output odometry correction (map to odom).
     */
    Output odomCorrection{*this, {.name = "odomCorrection", .types = {{dai::DatatypeEnum::TransformData, true}}}};
    /**
     * Output obstacle point cloud.
     */
    Output obstaclePCL{*this, {.name = "obstaclePCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
    /**
     * Output ground point cloud.
     */
    Output groundPCL{*this, {.name = "groundPCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
    /**
     * Output occupancy grid map.
     */
    Output occupancyGridMap{*this, {.name = "occupancyGridMap", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};

    /**
     * Output passthrough rectified image.
     */
    Output passthroughRect{*this, {.name = "passthroughRect", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};
    /**
     * Output passthrough depth image.
     */
    Output passthroughDepth{*this, {.name = "passthroughDepth", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};
    /**
     * Output passthrough features.
     */
    Output passthroughFeatures{*this, {.name = "passthroughFeatures", .types = {{dai::DatatypeEnum::TrackedFeatures, true}}}};
    /**
     * Output passthrough odometry pose.
     */
    Output passthroughOdom{*this, {.name = "passthroughOdom", .types = {{dai::DatatypeEnum::TransformData, true}}}};

    /**
     * Set RTABMap parameters.
     */
    void setParams(const std::map<std::string, std::string>& params);

    /**
     * Set RTABMap database path. "/tmp/rtabmap.tmp.db" by default.
     */
    void setDatabasePath(const std::string& path) {
        databasePath = path;
    }

    /**
     * Whether to load the database on start. False by default.
     */
    void setLoadDatabaseOnStart(bool load) {
        loadDatabaseOnStart = load;
    }

    /**
     * Whether to save the database on close. False by default.
     */
    void setSaveDatabaseOnClose(bool save) {
        saveDatabaseOnClose = save;
    }

    void saveDatabase();
    /**
     * Whether to save the database periodically. False by default.
     */
    void setSaveDatabasePeriodically(bool save) {
        saveDatabasePeriodically = save;
    }
    /**
     * Set the interval at which the database is saved. 30.0s by default.
     */
    void setSaveDatabasePeriod(double interval) {
        databaseSaveInterval = interval;
    }
    /**
     * Whether to publish the obstacle point cloud. True by default.
     */
    void setPublishObstacleCloud(bool publish) {
        publishObstacleCloud = publish;
    }
    /**
     * Whether to publish the ground point cloud. True by default.
     */
    void setPublishGrid(bool publish) {
        publishGrid = publish;
    }
    /**
     * Set the frequency at which the node processes data. 1Hz by default.
     */
    void setFreq(float f) {
        freq = f;
    }
    /**
     * Set the alpha scaling factor for the camera model.
     */
    void setAlphaScaling(float alpha) {
        alphaScaling = alpha;
    }
    /**
     * Whether to use input features for SLAM. False by default.
     */
    void setUseFeatures(bool reuse);
    void setLocalTransform(std::shared_ptr<TransformData> transform) {
        localTransform = transform->getRTABMapTransform();
    }
    std::shared_ptr<TransformData> getLocalTransform() {
        return std::make_shared<TransformData>(localTransform);
    }
    /**
     * Trigger a new map.
     */
    void triggerNewMap();

   private:
    void run() override;
    Input inSync{*this, {.name = "inSync", .types = {{dai::DatatypeEnum::MessageGroup, true}}}};
    void syncCB(std::shared_ptr<dai::ADatatype> data);
    void odomPoseCB(std::shared_ptr<dai::ADatatype> data);
    void imuCB(std::shared_ptr<dai::ADatatype> msg);
    void initialize(dai::Pipeline& pipeline, int instanceNum, int width, int height);
    rtabmap::StereoCameraModel model;
    rtabmap::Rtabmap rtabmap;
    rtabmap::Transform currPose, odomCorr;
    std::chrono::steady_clock::time_point lastProcessTime;
    std::chrono::steady_clock::time_point startTime;
    rtabmap::Transform imuLocalTransform;
    rtabmap::Transform localTransform;
    std::shared_ptr<rtabmap::LocalGridCache> localMaps;
    std::unique_ptr<rtabmap::OccupancyGrid> occupancyGrid;
    std::unique_ptr<rtabmap::CloudMap> cloudMap;
    rtabmap::SensorData sensorData;
    float alphaScaling = -1.0;
    bool useFeatures = false;
    bool initialized = false;
    std::map<std::string, std::string> rtabParams;
    std::string databasePath = "/tmp/rtabmap.tmp.db";
    double databaseSaveInterval = 30.0;
    bool loadDatabaseOnStart = false;
    bool saveDatabaseOnClose = false;
    bool saveDatabasePeriodically = false;
    bool publishObstacleCloud = true;
    bool publishGroundCloud = true;
    bool publishGrid = true;
    float freq = 1.0f;
};
}  // namespace node
}  // namespace dai