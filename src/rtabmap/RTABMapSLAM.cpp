#include "depthai/rtabmap/RTABMapSLAM.hpp"

#include <pcl/point_cloud.h>

#include "depthai/pipeline/Pipeline.hpp"
#include "rtabmap/core/util3d.h"

namespace dai {
namespace node {
void RTABMapSLAM::build() {
    alphaScaling = -1.0;
    reuseFeatures = false;
    localTransform = rtabmap::Transform::getIdentity();
    inputSync.setMaxSize(0);
    inputSync.setBlocking(false);
    inputSync.addCallback(std::bind(&RTABMapSLAM::syncCB, this, std::placeholders::_1));
    inputOdomPose.setMaxSize(0);
    inputOdomPose.setBlocking(false);
    inputOdomPose.addCallback(std::bind(&RTABMapSLAM::odomPoseCB, this, std::placeholders::_1));
    rtabmap.init();
}

void RTABMapSLAM::stop() {
    dai::Node::stop();
}

void RTABMapSLAM::setParams(const rtabmap::ParametersMap& params) {
    rtabmap.init(params, databasePath);
    rtabParams = params;
}

void RTABMapSLAM::syncCB(std::shared_ptr<dai::ADatatype> data) {
    auto group = std::dynamic_pointer_cast<dai::MessageGroup>(data);
    std::shared_ptr<dai::ImgFrame> imgFrame = nullptr;
    std::shared_ptr<dai::ImgFrame> depthFrame = nullptr;
    std::shared_ptr<dai::TrackedFeatures> features = nullptr;
    std::shared_ptr<dai::TransformData> odomPose = nullptr;
    for(auto& msg : *group) {
        if(msg.first == "img_rect") {
            imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(msg.second);
        } else if(msg.first == "depth") {
            depthFrame = std::dynamic_pointer_cast<dai::ImgFrame>(msg.second);
        } else if(msg.first == "features") {
            features = std::dynamic_pointer_cast<dai::TrackedFeatures>(msg.second);
        } else if(msg.first == "odom_pose") {
            odomPose = std::dynamic_pointer_cast<dai::TransformData>(msg.second);
        }
    }
    if(imgFrame != nullptr && depthFrame != nullptr) {
        if(!modelSet) {
            auto pipeline = getParentPipeline();
            rtabmap::Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
            localTransform = localTransform * opticalTransform.inverse();
            getCalib(pipeline, imgFrame->getInstanceNum(), imgFrame->getWidth(), imgFrame->getHeight());
            lastProcessTime = std::chrono::steady_clock::now();
            startTime = std::chrono::steady_clock::now();
            grid = new rtabmap::OccupancyGrid(&localMaps, rtabParams);

            modelSet = true;
        } else {
            double stamp = std::chrono::duration<double>(imgFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();

            sensorData = rtabmap::SensorData(imgFrame->getCvFrame(), depthFrame->getCvFrame(), model.left(), imgFrame->getSequenceNum(), stamp);
            std::vector<cv::KeyPoint> keypoints;
            if(features != nullptr) {
                for(auto& feature : features->trackedFeatures) {
                    keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
                }
                sensorData.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
            }
        }
        passthroughRect.send(imgFrame);
    }
}

void RTABMapSLAM::odomPoseCB(std::shared_ptr<dai::ADatatype> data) {
    auto odomPose = std::dynamic_pointer_cast<dai::TransformData>(data);
    // convert odom pose to rtabmap pose
    rtabmap::Transform p;
    odomPose->getRTABMapTransform(p);
    rtabmap::Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);

    currPose = p * opticalTransform.inverse();

    auto out = std::make_shared<dai::TransformData>(odomCorrection * currPose);
    transform.send(out);
}

void RTABMapSLAM::run() {
    while(isRunning()) {
        if(!modelSet) {
            continue;
        } else {
            rtabmap::Statistics stats;
            // process at 1 Hz
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            if(now - lastProcessTime > std::chrono::milliseconds(1000)) {
                lastProcessTime = now;
                bool success = rtabmap.process(sensorData, currPose);
                if(success) {
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
                        localMaps.add(iter->first, ground, obstacles, empty, node.sensorData().gridCellSize(), node.sensorData().gridViewPoint());
                    }
                    if(grid->addedNodes().size() ||localMaps.size() > 0) {
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
                        cv::flip(map8U, map8U, 0);
                        
                        auto gridMap = std::make_shared<dai::ImgFrame>();
                        gridMap->setTimestamp(std::chrono::steady_clock::now());
                        cv::Mat flat = map8U.reshape(1, map8U.total() * map8U.channels());
                        std::vector<uchar> vec = map8U.isContinuous() ? flat : flat.clone();
                        gridMap->setData(vec);
                        gridMap->setType(dai::ImgFrame::Type::GRAY8);
                        gridMap->setHeight(map8U.rows);
                        gridMap->setWidth(map8U.cols);
                        occupancyMap.send(gridMap);
                    }
                }
            }

            // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::cloudFromSensorData(sensorData,
            //                                                                                  4,  // decimation
            //                                                                                  0.0f);
            // // pcl to dai PointCloudData
            // auto pclData = std::make_shared<dai::PointCloudData>();

            // int size = cloud->points.size();
            // pclData->points.resize(size * 3);
            // for(int i = 0; i < size; i++) {
            //     pclData->points[i].x = cloud->points[i].x;
            //     pclData->points[i].y = cloud->points[i].y;
            //     pclData->points[i].z = cloud->points[i].z;
            // }
            // pointCloud.send(pclData);
        }
    }
    // after 2m save the database
    // if(std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count() > 180.0) {
    //     rtabmap.close();
    //     rtabmap.init(rtabParams, "/rtabmap.tmp.db");
    //     std::cout << "Database saved" << std::endl;
    //     startTime = std::chrono::steady_clock::now();
    // }
}

void RTABMapSLAM::getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();
    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    calibHandler.getRTABMapCameraModel(model, cameraId, width, height, localTransform, alphaScaling);
    auto eeprom = calibHandler.getEepromData();
    std::cout << "Board name: " << eeprom.boardName << std::endl;
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
    imuLocalTransform = localTransform * imuLocalTransform;
}
}  // namespace node
}  // namespace dai