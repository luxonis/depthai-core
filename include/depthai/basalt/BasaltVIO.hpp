#pragma once
#define SOPHUS_USE_BASIC_LOGGING

#include "depthai/pipeline/Subnode.hpp"
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

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    std::string leftInputName = "left";
    std::string rightInputName = "right";

    void buildInternal() override;
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
    void setImuExtrinsics(const std::shared_ptr<TransformData>& imuExtr);
    void setAccelBias(const std::vector<double>& accelBias);
    void setAccelNoiseStd(const std::vector<double>& accelNoiseStd);
    void setGyroBias(const std::vector<double>& gyroBias);
    void setGyroNoiseStd(const std::vector<double>& gyroNoiseStd);
    void setDefaultVIOConfig();
    void runSyncOnHost(bool runOnHost);

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
    std::shared_ptr<ImgFrame> leftImg;
    bool initialized = false;
    std::string configPath = "";
    int imuUpdateRate = 200;
    int threadNum = 1;
    bool useSpecTranslation = true;
    std::optional<std::shared_ptr<TransformData>> imuExtrinsics;
    std::optional<std::vector<double>> accelBias;
    std::optional<std::vector<double>> accelNoiseStd;
    std::optional<std::vector<double>> gyroBias;
    std::optional<std::vector<double>> gyroNoiseStd;
};
}  // namespace node
}  // namespace dai
