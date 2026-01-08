#include "depthai/rtabmap/RTABMapVIO.hpp"

#include "../utility/PimplImpl.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/rtabmap/RTABMapConversions.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"

namespace dai {
namespace node {
class RTABMapVIO::Impl {
   public:
    Impl() = default;
    rtabmap::StereoCameraModel model;
    std::unique_ptr<rtabmap::Odometry> odom;
    rtabmap::Transform localTransform;
    rtabmap::Transform imuLocalTransform;
    std::map<std::string, std::string> rtabParams;
    std::map<double, cv::Vec3f> accBuffer;
    std::map<double, cv::Vec3f> gyroBuffer;
};

RTABMapVIO::RTABMapVIO() = default;
RTABMapVIO::~RTABMapVIO() = default;
void RTABMapVIO::buildInternal() {
    sync->out.link(inSync);
    sync->setRunOnHost(false);
    pimplRtabmap->localTransform = rtabmap::Transform::getIdentity();

    rtabmap::Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    pimplRtabmap->localTransform = pimplRtabmap->localTransform * opticalTransform.inverse();

    inSync.setBlocking(false);
    inSync.setMaxSize(0);
    imu.setMaxSize(1);
    imu.setBlocking(false);
    rect.setBlocking(false);
    rect.setMaxSize(1);
    depth.setBlocking(false);
    depth.setMaxSize(1);
    if(useFeatures) {
        features.setBlocking(false);
        features.setMaxSize(1);
        inputs[featuresInputName] = features;
    }
    inSync.setMaxSize(1);
    inSync.setBlocking(false);
    imu.addCallback(std::bind(&RTABMapVIO::imuCB, this, std::placeholders::_1));
}

void RTABMapVIO::setUseFeatures(bool use) {
    useFeatures = use;
    if(useFeatures) {
        features.setBlocking(false);
        features.setMaxSize(1);
        inputs[featuresInputName] = features;
    }
}

void RTABMapVIO::imuCB(std::shared_ptr<dai::ADatatype> msg) {
    auto imuData = std::static_pointer_cast<dai::IMUData>(msg);
    auto imuPackets = imuData->packets;
    std::lock_guard<std::mutex> lock(imuMtx);
    for(auto& imuPacket : imuPackets) {
        auto& acceleroValues = imuPacket.acceleroMeter;
        auto& gyroValues = imuPacket.gyroscope;
        double accStamp = std::chrono::duration<double>(acceleroValues.getTimestampDevice().time_since_epoch()).count();
        double gyroStamp = std::chrono::duration<double>(gyroValues.getTimestampDevice().time_since_epoch()).count();
        pimplRtabmap->accBuffer.emplace_hint(pimplRtabmap->accBuffer.end(), accStamp, cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z));
        pimplRtabmap->gyroBuffer.emplace_hint(pimplRtabmap->gyroBuffer.end(), gyroStamp, cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z));
    }
}

void RTABMapVIO::syncCB(std::shared_ptr<dai::ADatatype> data) {
    auto group = std::dynamic_pointer_cast<dai::MessageGroup>(data);
    if(group == nullptr) return;
    auto imgFrame = group->get<dai::ImgFrame>(rectInputName);
    auto depthFrame = group->get<dai::ImgFrame>(depthInputName);
    std::shared_ptr<dai::TrackedFeatures> featuresFrame = nullptr;
    if(useFeatures) {
        featuresFrame = group->get<dai::TrackedFeatures>(featuresInputName);
    }
    rtabmap::SensorData sensorData;
    if(imgFrame != nullptr && depthFrame != nullptr) {
        if(!initialized) {
            auto pipeline = getParentPipeline();
            initialize(pipeline, imgFrame->getInstanceNum(), imgFrame->getWidth(), imgFrame->getHeight());
        } else {
            double stamp = std::chrono::duration<double>(imgFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();

            sensorData = rtabmap::SensorData(imgFrame->getCvFrame(), depthFrame->getCvFrame(), pimplRtabmap->model.left(), imgFrame->getSequenceNum(), stamp);

            if(featuresFrame != nullptr) {
                std::vector<cv::KeyPoint> keypoints;
                for(auto& feature : featuresFrame->trackedFeatures) {
                    keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
                }
                sensorData.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
            }
            std::lock_guard<std::mutex> lock(imuMtx);
            cv::Vec3d acc, gyro;
            std::map<double, cv::Vec3f>::const_iterator iterA, iterB;

            if(!pimplRtabmap->accBuffer.empty() && !pimplRtabmap->gyroBuffer.empty() && pimplRtabmap->accBuffer.rbegin()->first >= stamp
               && pimplRtabmap->gyroBuffer.rbegin()->first >= stamp) {
                // acc
                iterB = pimplRtabmap->accBuffer.lower_bound(stamp);
                if(iterB != pimplRtabmap->accBuffer.end()) {
                    iterA = iterB;
                    if(iterA != pimplRtabmap->accBuffer.begin()) --iterA;
                    if(iterA == iterB || stamp == iterB->first) {
                        acc = iterB->second;
                    } else if(stamp > iterA->first && stamp < iterB->first) {
                        float t = (stamp - iterA->first) / (iterB->first - iterA->first);
                        acc = iterA->second + t * (iterB->second - iterA->second);
                    }
                    pimplRtabmap->accBuffer.erase(pimplRtabmap->accBuffer.begin(), iterB);
                }

                // gyro
                iterB = pimplRtabmap->gyroBuffer.lower_bound(stamp);
                if(iterB != pimplRtabmap->gyroBuffer.end()) {
                    iterA = iterB;
                    if(iterA != pimplRtabmap->gyroBuffer.begin()) --iterA;
                    if(iterA == iterB || stamp == iterB->first) {
                        gyro = iterB->second;
                    } else if(stamp > iterA->first && stamp < iterB->first) {
                        float t = (stamp - iterA->first) / (iterB->first - iterA->first);
                        gyro = iterA->second + t * (iterB->second - iterA->second);
                    }
                    pimplRtabmap->gyroBuffer.erase(pimplRtabmap->gyroBuffer.begin(), iterB);
                }
                sensorData.setIMU(rtabmap::IMU(gyro, cv::Mat::eye(3, 3, CV_64FC1), acc, cv::Mat::eye(3, 3, CV_64FC1), pimplRtabmap->imuLocalTransform));
            }
            rtabmap::OdometryInfo info;
            auto pose = pimplRtabmap->odom->process(sensorData, &info);
            pose = pimplRtabmap->localTransform * pose * pimplRtabmap->localTransform.inverse();
            auto out = rtabmapToTransformData(pose);
            transform.send(out);
            passthroughRect.send(imgFrame);
            passthroughDepth.send(depthFrame);
            if(useFeatures && featuresFrame != nullptr) {
                passthroughFeatures.send(featuresFrame);
            }
        }
    }
}

void RTABMapVIO::run() {
    while(mainLoop()) {
        auto msg = inSync.get<dai::ADatatype>();
        if(msg != nullptr) {
            syncCB(msg);
        }
    }
}

void RTABMapVIO::reset(std::shared_ptr<TransformData> transform) {
    if(transform != nullptr) {
        pimplRtabmap->odom->reset(getRTABMapTransform(transform->transform));
    } else {
        pimplRtabmap->odom->reset();
    }
}

void RTABMapVIO::setParams(const rtabmap::ParametersMap& params) {
    pimplRtabmap->rtabParams = params;
}
void RTABMapVIO::setLocalTransform(std::shared_ptr<TransformData> transform) {
    pimplRtabmap->localTransform = getRTABMapTransform(transform->transform);
}

void RTABMapVIO::initialize(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    pimplRtabmap->model = getRTABMapCameraModel(cameraId, width, height, pimplRtabmap->localTransform, alphaScaling, calibHandler);

    std::vector<std::vector<float>> imuExtr = calibHandler.getImuToCameraExtrinsics(cameraId, true);

    pimplRtabmap->imuLocalTransform = rtabmap::Transform(double(imuExtr[0][0]),
                                                         double(imuExtr[0][1]),
                                                         double(imuExtr[0][2]),
                                                         double(imuExtr[0][3]) * 0.01,
                                                         double(imuExtr[1][0]),
                                                         double(imuExtr[1][1]),
                                                         double(imuExtr[1][2]),
                                                         double(imuExtr[1][3]) * 0.01,
                                                         double(imuExtr[2][0]),
                                                         double(imuExtr[2][1]),
                                                         double(imuExtr[2][2]),
                                                         double(imuExtr[2][3]) * 0.01);

    pimplRtabmap->imuLocalTransform = pimplRtabmap->localTransform * pimplRtabmap->imuLocalTransform;
    pimplRtabmap->odom.reset(rtabmap::Odometry::create(pimplRtabmap->rtabParams));
    initialized = true;
}
}  // namespace node
}  // namespace dai
