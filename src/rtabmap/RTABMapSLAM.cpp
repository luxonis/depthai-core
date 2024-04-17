#include "depthai/rtabmap/RTABMapSLAM.hpp"

namespace dai {
namespace node {
void RTABMapSLAM::build() {
    hostNode = true;
    alphaScaling = -1.0;
    reuseFeatures = false;
    rtabmap.init();
    // inputRect.queue.setMaxSize(1);
    // inputDepth.queue.setMaxSize(1);
    // inputFeatures.queue.setMaxSize(1);
    // inputRect.queue.setBlocking(false);
    // inputDepth.queue.setBlocking(false);
    // inputFeatures.queue.setBlocking(false);
}

void RTABMapSLAM::stop() {
    dai::Node::stop();
    // cv::destroyAllWindows();
}

void RTABMapSLAM::setParams(const rtabmap::ParametersMap& params) {
    rtabmap.init(params);
}

void RTABMapSLAM::run() {
    while(isRunning()) {
        auto imgFrame = inputRect.queue.get<dai::ImgFrame>();
        auto depthFrame = inputDepth.queue.get<dai::ImgFrame>();
        std::shared_ptr<dai::TrackedFeatures> features = nullptr;
        if(reuseFeatures) {
            features = inputFeatures.queue.get<dai::TrackedFeatures>();
        }
        auto odomPose = inputOdomPose.queue.get<dai::TransformData>();
        rtabmap::SensorData data;

        if(imgFrame != nullptr && depthFrame != nullptr) {
            if(!modelSet) {
                auto pipeline = getParentPipeline();
                getCalib(pipeline, imgFrame->getInstanceNum(), imgFrame->getWidth(), imgFrame->getHeight());
                modelSet = true;
                lastProcessTime = std::chrono::steady_clock::now();
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

                // convert odom pose to rtabmap pose
                rtabmap::Transform pose;
                odomPose->getRTABMapTransform(pose);

                rtabmap::Statistics stats;

                // process at 1 Hz
                std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                if(now - lastProcessTime > std::chrono::milliseconds(1000)) {
                    lastProcessTime = now;
                    if(rtabmap.process(data, pose)) {
                        stats = rtabmap.getStatistics();
                        if(rtabmap.getLoopClosureId() > 0){
                            fmt::print("Loop closure detected! last loop closure id = {}\n", rtabmap.getLoopClosureId());
                        }
                        odomCorrection= stats.mapCorrection();

                    }
                } 
                auto out = std::make_shared<dai::TransformData>(odomCorrection*pose);
                transform.send(out);
                passthroughRect.send(imgFrame);

            }
        }
    }
    fmt::print("Display node stopped\n");
}

void RTABMapSLAM::getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto device = pipeline.getDevice();
    auto calibHandler = device->readCalibration2();

    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    calibHandler.getRTABMapCameraModel(model, cameraId, width, height, alphaScaling);
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