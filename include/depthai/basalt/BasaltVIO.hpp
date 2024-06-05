#pragma once
#define SOPHUS_USE_BASIC_LOGGING
#include <tbb/concurrent_queue.h>
#include <tbb/global_control.h>

#include <Eigen/Core>

#include "basalt/calibration/calibration.hpp"
#include "basalt/serialization/headers_serialization.h"
#include "basalt/spline/se3_spline.h"
#include "basalt/utils/vio_config.h"
#include "basalt/vi_estimator/vio_estimator.h"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/node/Sync.hpp"

namespace dai {
namespace node {
class BasaltVIO : public NodeCRTP<ThreadedHostNode, BasaltVIO> {
   public:
    constexpr static const char* NAME = "BasaltVIO";

    void stereoCB(std::shared_ptr<ADatatype> in);

    void imuCB(std::shared_ptr<ADatatype> imuData);

    std::shared_ptr<BasaltVIO> build();
    void run() override;

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    std::string leftInputName = "left";
    std::string rightInputName = "right";

    Input& inputLeft = inputs[leftInputName];
    Input& inputRight = inputs[rightInputName];

    Input inputSync{*this, {.name = "inSync", .types = {{DatatypeEnum::MessageGroup, true}}}};
    Input inputImu{*this, {.name = "inIMU", .types = {{DatatypeEnum::IMUData, true}}}};
    Output transform{*this, {.name = "transform", .types = {{DatatypeEnum::TransformData, true}}}};
    Output passthrough{*this, {.name = "imgPassthrough", .types = {{DatatypeEnum::ImgFrame, true}}}};

    basalt::VioConfig vioConfig;
    void setImuUpdateRate(int rate) { imuUpdateRate = rate; }
    void setConfigPath(const std::string& path) { configPath = path; }
    void setUseSpecTranslation(bool use) { useSpecTranslation = use; }

   private:
    std::shared_ptr<basalt::Calibration<double>> calib;

    basalt::OpticalFlowBase::Ptr optFlowPtr;
    basalt::VioEstimatorBase::Ptr vio;
    basalt::OpticalFlowInput::Ptr lastImgData;

    tbb::concurrent_bounded_queue<basalt::OpticalFlowInput::Ptr>* imageDataQueue;
    tbb::concurrent_bounded_queue<basalt::ImuData<double>::Ptr>* imuDataQueue;
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> outStateQueue;

    std::vector<int64_t> vioTNSec;
    Eigen::aligned_vector<Eigen::Vector3d> vioTwI;
    std::shared_ptr<ImgFrame> leftImg;
    bool calibrated = false;
    std::string configPath = VIO_CONFIG_PATH;
    int imuUpdateRate = 200;
    int threadNum = 1;
    std::unique_ptr<tbb::global_control> tbbGlobalControl;
    bool useSpecTranslation = true;


    void initialize(std::vector<std::shared_ptr<ImgFrame>> frames);
};
}  // namespace node
}  // namespace dai