#include "depthai/rtabmap/RTABMapVIO.hpp"

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {
namespace node {
std::shared_ptr<RTABMapVIO> RTABMapVIO::build() {
    alphaScaling = -1.0;
    odom = rtabmap::Odometry::create();
    // inputIMU.queue.setMaxSize(0);
    // inputRect.queue.setMaxSize(0);
    // inputDepth.queue.setMaxSize(0);
    // inputFeatures.queue.setMaxSize(0);

    // inputIMU.queue.setBlocking(false);
    // inputRect.queue.setBlocking(false);
    // inputDepth.queue.setBlocking(false);
    // inputFeatures.queue.setBlocking(false);
    // inputRect.queue.setMaxSize(1);
    // inputDepth.queue.setMaxSize(1);
    // inputFeatures.queue.setMaxSize(1);
    // inputRect.queue.setBlocking(false);
    // inputDepth.queue.setBlocking(false);
    // inputFeatures.queue.setBlocking(false);
    inputIMU.addCallback(std::bind(&RTABMapVIO::imuCB, this, std::placeholders::_1));
    return std::static_pointer_cast<RTABMapVIO>(shared_from_this());
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
        accBuffer_.emplace_hint(accBuffer_.end(), accStamp, cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z));
        gyroBuffer_.emplace_hint(gyroBuffer_.end(), gyroStamp, cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z));
        rotBuffer_.emplace_hint(rotBuffer_.end(), gyroStamp, cv::Vec4f(rotValues.i, rotValues.j, rotValues.k, rotValues.real));
    }
}

void RTABMapVIO::stop() {
    dai::Node::stop();
    // cv::destroyAllWindows();
    if(odom != nullptr) {
        delete odom;
    }
}

void RTABMapVIO::setParams(const rtabmap::ParametersMap& params) {
    odom = rtabmap::Odometry::create(params);
}

void RTABMapVIO::run() {
    while(isRunning()) {
        auto imgFrame = inputRect.get<dai::ImgFrame>();
        auto depthFrame = inputDepth.get<dai::ImgFrame>();
        auto features = inputFeatures.get<dai::TrackedFeatures>();
        auto reset = inputReset.tryGet<dai::CameraControl>();
        rtabmap::SensorData data;
        if(reset != nullptr) {
            odom->reset();
        }

        if(imgFrame != nullptr && depthFrame != nullptr) {
            if(!modelSet) {
                auto pipeline = getParentPipeline();
                getCalib(pipeline, imgFrame->getInstanceNum(), imgFrame->getWidth(), imgFrame->getHeight());
                modelSet = true;
            } else {
                double stamp = std::chrono::duration<double>(imgFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();

                data = rtabmap::SensorData(imgFrame->getCvFrame(), depthFrame->getCvFrame(), model.left(), imgFrame->getSequenceNum(), stamp);

                std::vector<cv::KeyPoint> keypoints;
                if(features != nullptr) {
                    for(auto& feature : features->trackedFeatures) {
                        keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
                    }
                    data.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
                }

                cv::Vec3d acc, gyro;
                cv::Vec4d rot;
                std::map<double, cv::Vec3f>::const_iterator iterA, iterB;
                std::map<double, cv::Vec4f>::const_iterator iterC, iterD;
                if(accBuffer_.empty() || gyroBuffer_.empty() || rotBuffer_.empty() || accBuffer_.rbegin()->first < stamp || gyroBuffer_.rbegin()->first < stamp
                   || rotBuffer_.rbegin()->first < stamp) {
                } else {
                    // acc
                    iterB = accBuffer_.lower_bound(stamp);
                    iterA = iterB;
                    if(iterA != accBuffer_.begin()) iterA = --iterA;
                    if(iterA == iterB || stamp == iterB->first) {
                        acc = iterB->second;
                    } else if(stamp > iterA->first && stamp < iterB->first) {
                        float t = (stamp - iterA->first) / (iterB->first - iterA->first);
                        acc = iterA->second + t * (iterB->second - iterA->second);
                    }
                    accBuffer_.erase(accBuffer_.begin(), iterB);

                    // gyro
                    iterB = gyroBuffer_.lower_bound(stamp);
                    iterA = iterB;
                    if(iterA != gyroBuffer_.begin()) iterA = --iterA;
                    if(iterA == iterB || stamp == iterB->first) {
                        gyro = iterB->second;
                    } else if(stamp > iterA->first && stamp < iterB->first) {
                        float t = (stamp - iterA->first) / (iterB->first - iterA->first);
                        gyro = iterA->second + t * (iterB->second - iterA->second);
                    }
                    gyroBuffer_.erase(gyroBuffer_.begin(), iterB);

                    // rot
                    iterD = rotBuffer_.lower_bound(stamp);
                    iterC = iterD;
                    if(iterC != rotBuffer_.begin()) iterC = --iterC;
                    if(iterC == iterD || stamp == iterD->first) {
                        rot = iterD->second;
                    } else if(stamp > iterC->first && stamp < iterD->first) {
                        float t = (stamp - iterC->first) / (iterD->first - iterC->first);
                        rot = iterC->second + t * (iterD->second - iterC->second);
                    }
                    rotBuffer_.erase(rotBuffer_.begin(), iterD);

                    data.setIMU(rtabmap::IMU(
                        rot, cv::Mat::eye(3, 3, CV_64FC1), gyro, cv::Mat::eye(3, 3, CV_64FC1), acc, cv::Mat::eye(3, 3, CV_64FC1), imuLocalTransform));
                }
                auto pose = odom->process(data, &info);
                cv::Mat final_img;

                for(auto word : info.words) {
                    keypoints.push_back(word.second);
                }
                cv::drawKeypoints(imgFrame->getCvFrame(), keypoints, final_img);

                // add pose information to frame

                float x, y, z, roll, pitch, yaw;
                pose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
                auto out = std::make_shared<dai::TransformData>(pose);
                transform.send(out);
                passthroughRect.send(imgFrame);
                passthroughDepth.send(depthFrame);
                passthroughFeatures.send(features);
            }
        }
    }
    fmt::print("Display node stopped\n");
}

void RTABMapVIO::getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    calibHandler.getRTABMapCameraModel(model, cameraId, width, height, rtabmap::Transform::getIdentity(), alphaScaling);
    auto eeprom = calibHandler.getEepromData();
    if(eeprom.boardName == "OAK-D" || eeprom.boardName == "BW1098OBC") {
        imuLocalTransform = rtabmap::Transform(0, -1, 0, 0.0525, 1, 0, 0, 0.013662, 0, 0, 1, 0);
    } else if(eeprom.boardName == "DM9098") {
        imuLocalTransform = rtabmap::Transform(0, 1, 0, 0.037945, 1, 0, 0, 0.00079, 0, 0, -1, 0);
    } else if(eeprom.boardName == "NG2094") {
        imuLocalTransform = rtabmap::Transform(0, 1, 0, 0.0374, 1, 0, 0, 0.00176, 0, 0, -1, 0);
    } else if(eeprom.boardName == "NG9097") {
        imuLocalTransform = rtabmap::Transform(0, 1, 0, 0.04, 1, 0, 0, 0.020265, 0, 0, -1, 0);
    } else {
        std::cout << "Unknown IMU local transform for " << eeprom.boardName << std::endl;
        stop();
    }

    imuLocalTransform = rtabmap::Transform::getIdentity() * imuLocalTransform;
}
}  // namespace node
}  // namespace dai