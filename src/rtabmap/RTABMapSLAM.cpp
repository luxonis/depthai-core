#include "depthai/rtabmap/RTABMapSLAM.hpp"

#include <pcl/point_cloud.h>

#include "rtabmap/core/util3d.h"

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
    rtabParams = params;
    
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
                lastProcessTime = std::chrono::steady_clock::now();
                grid = new rtabmap::OccupancyGrid(&localMaps_, rtabParams);

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
                        if(rtabmap.getLoopClosureId() > 0) {
                            fmt::print("Loop closure detected! last loop closure id = {}\n", rtabmap.getLoopClosureId());
                        }
                        odomCorrection = stats.mapCorrection();
                        std::map<int, rtabmap::Signature> nodes;
                        std::map<int, rtabmap::Transform> optimizedPoses;
                        std::multimap<int, rtabmap::Link> links;
                        rtabmap.getGraph(optimizedPoses, links, true, true, &nodes, true, true, true, true);
                        for(std::map<int, rtabmap::Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter) {
                            rtabmap::Signature node = nodes.find(iter->first)->second;
                            if(node.sensorData().gridCellSize() == 0.0f) {
                                std::cout << "Grid cell size is 0, skipping node " << iter->first << std::endl;
                                continue;
                            }
                            // uncompress grid data
                            cv::Mat ground, obstacles, empty;
                            node.sensorData().uncompressData(0, 0, 0, 0, &ground, &obstacles, &empty);
                            localMaps_.add(iter->first, ground, obstacles, empty, node.sensorData().gridCellSize(), node.sensorData().gridViewPoint());
                        }
                        if(grid->addedNodes().size() || localMaps_.size() > 0) {
                            grid->update(optimizedPoses);
                        }
                        float xMin, yMin;
                        cv::Mat map = grid->getMap(xMin, yMin);
                        if(!map.empty()) {
                            cv::Mat map8U(map.rows, map.cols, CV_8U);
                            // convert to gray scaled map
                            for(int i = 0; i < map.rows; ++i) {
                                for(int j = 0; j < map.cols; ++j) {
                                    char v = map.at<char>(i, j);
                                    unsigned char gray;
                                    if(v == 0) {
                                        gray = 178;
                                    } else if(v == 100) {
                                        gray = 0;
                                    } else  // -1
                                    {
                                        gray = 89;
                                    }
                                    map8U.at<unsigned char>(i, j) = gray;
                                }
                            }
                            auto gridMap = std::make_shared<dai::ImgFrame>();
                            gridMap->setTimestamp(std::chrono::steady_clock::now());
                            // convert cv mat data to std::vector<uint8_t>
                            // std::cout << map8U << std::endl;
                            cv::Mat flat = map8U.reshape(1, map8U.total()*map8U.channels());
                            std::vector<uchar> vec = map8U.isContinuous()? flat : flat.clone();
                            gridMap->setData(vec);
                            gridMap->setType(dai::ImgFrame::Type::GRAY8);
                            gridMap->setHeight(map8U.rows);
                            gridMap->setWidth(map8U.cols);
                            occupancyMap.send(gridMap);
                        }
                    }
                }
                auto out = std::make_shared<dai::TransformData>(odomCorrection * pose);
                transform.send(out);
                passthroughRect.send(imgFrame);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::cloudFromSensorData(data,
                                                                                                 4,  // decimation
                                                                                                 0.0f);
                // pcl to dai PointCloudData
                auto pclData = std::make_shared<dai::PointCloudData>();

                int size = cloud->points.size();
                pclData->points.resize(size * 3);
                for(int i = 0; i < size; i++) {
                    pclData->points[i].x = cloud->points[i].x;
                    pclData->points[i].y = cloud->points[i].y;
                    pclData->points[i].z = cloud->points[i].z;
                }
                pointCloud.send(pclData);
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