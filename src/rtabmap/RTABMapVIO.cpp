#include "depthai/rtabmap/RTABMapVIO.hpp"

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {
namespace node {
std::shared_ptr<RTABMapVIO> RTABMapVIO::build() {
    sync->out.link(inSync);
    sync->setRunOnHost(false);
    odom.reset(rtabmap::Odometry::create());

    localTransform = rtabmap::Transform::getIdentity();

    rtabmap::Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    localTransform = localTransform * opticalTransform.inverse();

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
    // inSync.addCallback(std::bind(&RTABMapVIO::syncCB, this, std::placeholders::_1));
    imu.addCallback(std::bind(&RTABMapVIO::imuCB, this, std::placeholders::_1));
    return std::static_pointer_cast<RTABMapVIO>(shared_from_this());
}

void RTABMapVIO::setUseFeatures(bool reuse) {
    useFeatures = reuse;
    if(useFeatures) {
        features.setBlocking(false);
        features.setMaxSize(1);
        inputs[featuresInputName] = features;
    }
}

void RTABMapVIO::imuCB(std::shared_ptr<dai::ADatatype> msg) {
    auto imuData = std::static_pointer_cast<dai::IMUData>(msg);
    auto imuPackets = imuData->packets;
    for(auto& imuPacket : imuPackets) {
        auto& acceleroValues = imuPacket.acceleroMeter;
        auto& gyroValues = imuPacket.gyroscope;
        auto& rotValues = imuPacket.rotationVector;
        double accStamp = std::chrono::duration<double>(acceleroValues.getTimestampDevice().time_since_epoch()).count();
        double gyroStamp = std::chrono::duration<double>(gyroValues.getTimestampDevice().time_since_epoch()).count();
        accBuffer.emplace_hint(accBuffer.end(), accStamp, cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z));
        gyroBuffer.emplace_hint(gyroBuffer.end(), gyroStamp, cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z));
        rotBuffer.emplace_hint(rotBuffer.end(), gyroStamp, cv::Vec4f(rotValues.i, rotValues.j, rotValues.k, rotValues.real));
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
        if(!modelSet) {
            auto pipeline = getParentPipeline();
            getCalib(pipeline, imgFrame->getInstanceNum(), imgFrame->getWidth(), imgFrame->getHeight());
            modelSet = true;
        } else {
            double stamp = std::chrono::duration<double>(imgFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();

            sensorData = rtabmap::SensorData(imgFrame->getCvFrame(), depthFrame->getCvFrame(), model.left(), imgFrame->getSequenceNum(), stamp);

            std::vector<cv::KeyPoint> keypoints;
            if(featuresFrame != nullptr) {
                std::cout << "Features frame\n" << std::endl;
                for(auto& feature : featuresFrame->trackedFeatures) {
                    keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
                }
                sensorData.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
            }

            cv::Vec3d acc, gyro;
            cv::Vec4d rot;
            std::map<double, cv::Vec3f>::const_iterator iterA, iterB;
            std::map<double, cv::Vec4f>::const_iterator iterC, iterD;

            if(!accBuffer.empty() && !gyroBuffer.empty() && !rotBuffer.empty() && accBuffer.rbegin()->first >= stamp && gyroBuffer.rbegin()->first >= stamp
               && rotBuffer.rbegin()->first >= stamp) {
                // acc
                iterB = accBuffer.lower_bound(stamp);
                if(iterB != accBuffer.end()) {
                    iterA = iterB;
                    if(iterA != accBuffer.begin()) --iterA;
                    if(iterA == iterB || stamp == iterB->first) {
                        acc = iterB->second;
                    } else if(stamp > iterA->first && stamp < iterB->first) {
                        float t = (stamp - iterA->first) / (iterB->first - iterA->first);
                        acc = iterA->second + t * (iterB->second - iterA->second);
                    }
                    accBuffer.erase(accBuffer.begin(), iterB);
                }

                // gyro
                iterB = gyroBuffer.lower_bound(stamp);
                if(iterB != gyroBuffer.end()) {
                    iterA = iterB;
                    if(iterA != gyroBuffer.begin()) --iterA;
                    if(iterA == iterB || stamp == iterB->first) {
                        gyro = iterB->second;
                    } else if(stamp > iterA->first && stamp < iterB->first) {
                        float t = (stamp - iterA->first) / (iterB->first - iterA->first);
                        gyro = iterA->second + t * (iterB->second - iterA->second);
                    }
                    gyroBuffer.erase(gyroBuffer.begin(), iterB);
                }

                // rot
                iterD = rotBuffer.lower_bound(stamp);
                if(iterD != rotBuffer.end()) {
                    iterC = iterD;
                    if(iterC != rotBuffer.begin()) --iterC;
                    if(iterC == iterD || stamp == iterD->first) {
                        rot = iterD->second;
                    } else if(stamp > iterC->first && stamp < iterD->first) {
                        float t = (stamp - iterC->first) / (iterD->first - iterC->first);
                        rot = iterC->second + t * (iterD->second - iterC->second);
                    }
                    rotBuffer.erase(rotBuffer.begin(), iterD);
                }

                sensorData.setIMU(
                    rtabmap::IMU(rot, cv::Mat::eye(3, 3, CV_64FC1), gyro, cv::Mat::eye(3, 3, CV_64FC1), acc, cv::Mat::eye(3, 3, CV_64FC1), imuLocalTransform));
            }
            rtabmap::OdometryInfo info;
            auto pose = odom->process(sensorData, &info);
            pose = localTransform * pose * localTransform.inverse();
            auto out = std::make_shared<dai::TransformData>(pose);
            transform.send(out);
            passthroughRect.send(imgFrame);
            passthroughDepth.send(depthFrame);
            if(useFeatures&&featuresFrame!=nullptr) {
                passthroughFeatures.send(featuresFrame);
            }
        }
    }
}

void RTABMapVIO::run(){
    while(isRunning()) {
        auto msg = inSync.get<dai::ADatatype>();
        if(msg != nullptr) {
            syncCB(msg);
        }
    }
}

void RTABMapVIO::setParams(const rtabmap::ParametersMap& params) {
    odom.reset(rtabmap::Odometry::create(params));
}

void RTABMapVIO::getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    model = calibHandler.getRTABMapCameraModel(cameraId, width, height, localTransform, alphaScaling);

    std::vector<std::vector<float>> imuExtr = calibHandler.getImuToCameraExtrinsics(cameraId, true);

    imuLocalTransform = rtabmap::Transform(double(imuExtr[0][0]),
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

    imuLocalTransform = localTransform * imuLocalTransform;
}
}  // namespace node
}  // namespace dai