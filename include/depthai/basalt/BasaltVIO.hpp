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
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

namespace dai {
namespace node {
class BasaltVIO : public dai::NodeCRTP<dai::node::ThreadedHostNode, BasaltVIO> {
   public:
    constexpr static const char* NAME = "BasaltVIO";

    void stereoCB(std::shared_ptr<dai::ADatatype> images);

    void imuCB(std::shared_ptr<dai::ADatatype> imuData);

    std::shared_ptr<BasaltVIO> build();
    void run() override;

    dai::Node::Input inputStereo{*this, {.name = "inStereo", .types = {{dai::DatatypeEnum::MessageGroup, true}}}};
    dai::Node::Input inputImu{*this, {.name = "inIMU", .types = {{dai::DatatypeEnum::IMUData, true}}}};
    dai::Node::Output transform{*this, {.name = "transform", .types = {{dai::DatatypeEnum::TransformData, true}}}};
    dai::Node::Output passthrough{*this, {.name = "imgPassthrough", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};

    basalt::VioConfig vioConfig;
    void setImuUpdateRate(int rate) { imuUpdateRate = rate; }
    void setConfigPath(const std::string& path) { configPath = path; }

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
    std::shared_ptr<dai::ImgFrame> leftImg;
    bool calibrated = false;
    std::string configPath = VIO_CONFIG_PATH;
    int imuUpdateRate = 200;
    int threadNum = 8;
    std::unique_ptr<tbb::global_control> tbb_global_control;


    void initialize(std::vector<std::shared_ptr<dai::ImgFrame>> frames);
};
}  // namespace node
}  // namespace dai