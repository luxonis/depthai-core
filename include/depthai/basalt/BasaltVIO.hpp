#pragma once
#define SOPHUS_USE_BASIC_LOGGING

#include "basalt/calibration/calibration.hpp"
#include "basalt/serialization/headers_serialization.h"
#include "basalt/spline/se3_spline.h"
#include "basalt/utils/vio_config.h"
#include "basalt/vi_estimator/vio_estimator.h"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {
namespace node {

/**
 * @brief Basalt Visual Inertial Odometry node. Performs VIO on stereo images and IMU data.

*/
class BasaltVIO : public NodeCRTP<ThreadedHostNode, BasaltVIO> {
   public:
    constexpr static const char* NAME = "BasaltVIO";
    BasaltVIO();
    ~BasaltVIO();
    std::shared_ptr<BasaltVIO> build();

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    std::string leftInputName = "left";
    std::string rightInputName = "right";

    /**
     * Input left image on which VIO is performed.
     */
    Input& left = inputs[leftInputName];
    /**
     * Input right image on which VIO is performed.
     */
    Input& right = inputs[rightInputName];
    /**
     * Input IMU data.
     */
    Input imu{*this, {"inIMU", DEFAULT_GROUP, false, 0, {{DatatypeEnum::IMUData, true}}}};

    /**
     * Output transform data.
     */
    Output transform{*this, {"transform", DEFAULT_GROUP, {{DatatypeEnum::TransformData, true}}}};
    /**
     * Output passthrough of left image.
     */
    Output passthrough{*this, {"imgPassthrough", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * VIO configuration file.
     */
    basalt::VioConfig vioConfig;
    void setImuUpdateRate(int rate) {
        imuUpdateRate = rate;
    }
    void setConfigPath(const std::string& path) {
        configPath = path;
    }
    void setUseSpecTranslation(bool use) {
        useSpecTranslation = use;
    }
    void setLocalTransform(const std::shared_ptr<TransformData>& transform);
    void setDefaultVIOConfig();

   private:
    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    void run() override;
    void initialize(std::vector<std::shared_ptr<ImgFrame>> frames);
    void stereoCB(std::shared_ptr<ADatatype> in);
    void imuCB(std::shared_ptr<ADatatype> imuData);
    void stop() override;
    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};
    std::shared_ptr<basalt::Calibration<double>> calib;

    basalt::OpticalFlowBase::Ptr optFlowPtr;
    basalt::VioEstimatorBase::Ptr vio;
    basalt::OpticalFlowInput::Ptr lastImgData;

    std::vector<int64_t> vioTNSec;
    std::shared_ptr<basalt::PoseState<double>::SE3> localTransform;
    std::shared_ptr<ImgFrame> leftImg;
    bool initialized = false;
    std::string configPath = "";
    int imuUpdateRate = 200;
    int threadNum = 1;
    bool useSpecTranslation = true;
};
}  // namespace node
}  // namespace dai