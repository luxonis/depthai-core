#include "depthai/rtabmap/VIO.hpp"

namespace dai {
namespace node {
void VisualOdometry::build() {
    hostNode = true;
    odom = rtabmap::Odometry::create();
}
void VisualOdometry::run() {
    while(isRunning()) {
        auto imgFrame = inputRect.queue.get<dai::ImgFrame>();
        auto depthFrame = inputDepth.queue.get<dai::ImgFrame>();
        auto imuData = inputIMU.queue.get<dai::IMUData>();
        auto features = inputFeatures.queue.tryGet<dai::TrackedFeatures>();
        if(!modelSet) {
            auto pipeline = getParentPipeline();
            getCalib(pipeline, imgFrame->getInstanceNum(), imgFrame->getWidth(), imgFrame->getHeight());
            modelSet = true;
            std::cout << "Model set" << std::endl;
        } else {
            double stamp = std::chrono::duration<double>(imgFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();

            auto data = rtabmap::SensorData(imgFrame->getCvFrame(), depthFrame->getCvFrame(), model.left(), imgFrame->getSequenceNum(), stamp);

            auto imuPackets = imuData->packets;

            for(auto& imuPacket : imuPackets) {
                auto& acceleroValues = imuPacket.acceleroMeter;
                auto& gyroValues = imuPacket.gyroscope;
                auto& rotValues = imuPacket.rotationVector;
                rtabmap::IMU imu(cv::Vec4f(rotValues.i, rotValues.j, rotValues.k, rotValues.real),
                                 cv::Mat::eye(3, 3, CV_64FC1),
                                 cv::Vec3f(gyroValues.x, gyroValues.y, gyroValues.z),
                                 cv::Mat::eye(3, 3, CV_64FC1),
                                 cv::Vec3f(acceleroValues.x, acceleroValues.y, acceleroValues.z),
                                 cv::Mat::eye(3, 3, CV_64FC1),
                                 imuLocalTransform);
                data.setIMU(imu);
            }

            std::vector<cv::KeyPoint> keypoints;
            if(features != nullptr) {
                for(auto& feature : features->trackedFeatures) {
                    keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
                }
                data.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
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

            std::stringstream xPos;
            xPos << "X: " << x << " m";
            cv::putText(final_img, xPos.str(), cv::Point(10, 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream yPos;
            yPos << "Y: " << y << " m";
            cv::putText(final_img, yPos.str(), cv::Point(10, 70), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream zPos;
            zPos << "Z: " << z << " m";
            cv::putText(final_img, zPos.str(), cv::Point(10, 90), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream rollPos;
            rollPos << "Roll: " << roll << " rad";
            cv::putText(final_img, rollPos.str(), cv::Point(10, 110), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream pitchPos;
            pitchPos << "Pitch: " << pitch << " rad";
            cv::putText(final_img, pitchPos.str(), cv::Point(10, 130), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream yawPos;
            yawPos << "Yaw: " << yaw << " rad";
            cv::putText(final_img, yawPos.str(), cv::Point(10, 150), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            cv::imshow("keypoints", final_img);
            auto key = cv::waitKey(1);
            if(key == 'q' || key == 'Q') {
                stop();
            }
            Eigen::Quaternionf q = pose.getQuaternionf();

            auto out = std::make_shared<dai::TransformData>();
            out->transform.x = x;
            out->transform.y = y;
            out->transform.z = z;
            out->transform.qx = q.x();
            out->transform.qy = q.y();
            out->transform.qz = q.z();
            out->transform.qw = q.w();
            transform.send(out);
        }
    }
    fmt::print("Display node stopped\n");
}

void VisualOdometry::getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    std::cout << "Getting calib" << std::endl;
    std::cout << "Instance num: " << instanceNum << std::endl;
    std::cout << "Width: " << width << std::endl;
    std::cout << "Height: " << height << std::endl;
    auto device = pipeline.getDevice();
    auto calibHandler = device->readCalibration2();

    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    cv::Mat cameraMatrix, newCameraMatrix;

    std::vector<std::vector<float> > matrix = calibHandler.getCameraIntrinsics(cameraId, width, height);
    cameraMatrix = (cv::Mat_<double>(3, 3) << matrix[0][0],
                    matrix[0][1],
                    matrix[0][2],
                    matrix[1][0],
                    matrix[1][1],
                    matrix[1][2],
                    matrix[2][0],
                    matrix[2][1],
                    matrix[2][2]);

    std::vector<float> coeffs = calibHandler.getDistortionCoefficients(cameraId);

    newCameraMatrix = cameraMatrix;

    double fx = newCameraMatrix.at<double>(0, 0);
    double fy = newCameraMatrix.at<double>(1, 1);
    double cx = newCameraMatrix.at<double>(0, 2);
    double cy = newCameraMatrix.at<double>(1, 2);
    double baseline = calibHandler.getBaselineDistance(dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_B) / 100.0;

    if(cameraId == dai::CameraBoardSocket::CAM_A)
        model = rtabmap::StereoCameraModel(device->getDeviceName(), fx, fy, cx, cy, baseline, rtabmap::Transform::getIdentity(), cv::Size(width, height));
    else
        model = rtabmap::StereoCameraModel(
            device->getDeviceName(),
            fx,
            fy,
            cx,
            cy,
            baseline,
            rtabmap::Transform::getIdentity() * rtabmap::Transform(-calibHandler.getBaselineDistance(dai::CameraBoardSocket::CAM_A) / 100.0, 0, 0),
            cv::Size(width, height));

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