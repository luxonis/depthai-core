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
#include "rtabmap/core/global_map/OccupancyGrid.h"

namespace dai {
namespace node {
class RTABMapSLAM : public dai::NodeCRTP<dai::node::ThreadedHostNode, RTABMapSLAM> {
   public:
    constexpr static const char* NAME = "RTABMapSLAM";

    std::shared_ptr<RTABMapSLAM> build();
    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;
    std::string rectInputName = "rect";
    std::string depthInputName = "depth";
    std::string featuresInputName = "features";

    Input& inputRect = inputs[rectInputName];
    Input& inputDepth = inputs[depthInputName];
    Input inputFeatures{*this, {.name = featuresInputName, .types = {{DatatypeEnum::TrackedFeatures, true}}}};

    Input inputSync{*this, {.name = "inSync", .types = {{dai::DatatypeEnum::MessageGroup, true}}}};
    Input inputOdomPose{*this, {.name = "inputOdomPose", .types = {{dai::DatatypeEnum::TransformData, true}}}};

    Output transform{*this, {.name = "transform", .types = {{dai::DatatypeEnum::TransformData, true}}}};
    Output passthroughRect{*this, {.name = "passthroughRect", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};
    Output pointCloud{*this, {.name = "pointCloud", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
    Output occupancyMap{*this, {.name = "map", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};

    void run() override;
    void stop() override;
    void setParams(const rtabmap::ParametersMap& params);
    void syncCB(std::shared_ptr<dai::ADatatype> data);
    void odomPoseCB(std::shared_ptr<dai::ADatatype> data);
    void setDatabasePath(const std::string& path) {
        databasePath = path;
    }
    void saveDatabase();
    void setSaveDatabasePeriodically(bool save) {
        saveDatabasePeriodically = save;
    }
    void setPublishPCL(bool publish) {
        publishPCL = publish;
    }
    void setPublishGrid(bool publish) {
        publishGrid = publish;
    }
    void setFreq(float f) {
        freq = f;
    }
    void setAlphaScaling(float alpha) {
        alphaScaling = alpha;
    }
    void setReuseFeatures(bool reuse);
    void setLocalTransform(std::shared_ptr<TransformData> transform) {
        transform->getRTABMapTransform(localTransform);
    }
    std::shared_ptr<TransformData> getLocalTransform() {
        return std::make_shared<TransformData>(localTransform);
    }
    void triggerNewMap();

   private:
    void imuCB(std::shared_ptr<dai::ADatatype> msg);
    void getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height);
    rtabmap::StereoCameraModel model;
    rtabmap::Rtabmap rtabmap;
    rtabmap::Transform currPose, odomCorrection;
    std::chrono::steady_clock::time_point lastProcessTime;
    std::chrono::steady_clock::time_point startTime;
    rtabmap::Transform imuLocalTransform;
    rtabmap::Transform localTransform;
    rtabmap::LocalGridCache localMaps;
    rtabmap::OccupancyGrid* grid;
    float alphaScaling = -1.0;
    bool useFeatures = false;
    bool modelSet = false;
    rtabmap::ParametersMap rtabParams;
    rtabmap::SensorData sensorData;
    std::string databasePath = "/tmp/rtabmap.tmp.db";
    float databaseSaveInterval = 5.0f;
    bool saveDatabasePeriodically = false;
    bool publishPCL = false;
    bool publishGrid = true;
    float freq = 1.0f;
};
}  // namespace node
}  // namespace dai