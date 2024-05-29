#include "depthai/rtabmap/RTABMapSLAM.hpp"

#include <pcl/point_cloud.h>

#include "depthai/pipeline/Pipeline.hpp"
#include "rtabmap/core/util3d.h"

namespace dai {
namespace node {
std::shared_ptr<RTABMapSLAM> RTABMapSLAM::build() {
    rtabmap.init();
    sync->out.link(inputSync);
    sync->setRunOnHost(false);
    alphaScaling = -1.0;
    localTransform = rtabmap::Transform::getIdentity();
    rtabmap::Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0); 
    localTransform = localTransform * opticalTransform;
    inputRect.setBlocking(false);
    inputRect.setMaxSize(1);
    inputDepth.setBlocking(false);
    inputDepth.setMaxSize(1);
    inputSync.setMaxSize(1);
    inputSync.setBlocking(false);
    inputSync.addCallback(std::bind(&RTABMapSLAM::syncCB, this, std::placeholders::_1));
    inputOdomPose.setMaxSize(1);
    inputOdomPose.setBlocking(false);
    inputOdomPose.addCallback(std::bind(&RTABMapSLAM::odomPoseCB, this, std::placeholders::_1));

    return std::static_pointer_cast<RTABMapSLAM>(shared_from_this());
}

void RTABMapSLAM::stop() {
    dai::Node::stop();
}

void RTABMapSLAM::setParams(const rtabmap::ParametersMap& params) {
    rtabmap.init(params, databasePath);
    rtabParams = params;
}

void RTABMapSLAM::triggerNewMap() {
    rtabmap.triggerNewMap();
}

void RTABMapSLAM::saveDatabase() {
    rtabmap.close();
    rtabmap.init(rtabParams, databasePath);
}

void RTABMapSLAM::setReuseFeatures(bool reuse){
    useFeatures = reuse;
    if(useFeatures){
        inputFeatures.setBlocking(false);
        inputFeatures.setMaxSize(1);
        inputs[featuresInputName] = inputFeatures;
    }
}

void RTABMapSLAM::syncCB(std::shared_ptr<dai::ADatatype> data) {
    auto group = std::dynamic_pointer_cast<dai::MessageGroup>(data);
    if (group == nullptr) return;
    std::shared_ptr<dai::ImgFrame> imgFrame = nullptr;
    std::shared_ptr<dai::ImgFrame> depthFrame = nullptr;
    std::shared_ptr<dai::TrackedFeatures> features = nullptr;
    imgFrame = group->get<dai::ImgFrame>(rectInputName);
    depthFrame = group->get<dai::ImgFrame>(depthInputName);
    if (useFeatures){
        features = group->get<dai::TrackedFeatures>(featuresInputName);
    }
    if(imgFrame != nullptr && depthFrame != nullptr) {
        if(!modelSet) {
            auto pipeline = getParentPipeline();
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
    currPose = p;

    auto out = std::make_shared<dai::TransformData>(odomCorrection * currPose);
    transform.send(out);
}

void RTABMapSLAM::run() {
    while(isRunning()) {
        if(!modelSet) {
            continue;
        } else {
            rtabmap::Statistics stats;
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            if(now - lastProcessTime > std::chrono::milliseconds(int(1000.0f / freq))) {
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
                    if(grid->addedNodes().size() || localMaps.size() > 0) {
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
            if(publishPCL) {
                // convert sensor data to pcl
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::cloudFromSensorData(sensorData, 4, 0.0f);
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
        // save database periodically if set
        if(saveDatabasePeriodically && std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count() > 30.0) {
            rtabmap.close();
            rtabmap.init(rtabParams, databasePath);
            std::cout << "Database saved" << std::endl;
            startTime = std::chrono::steady_clock::now();
        }
    }
}

void RTABMapSLAM::getCalib(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();
    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    model = calibHandler.getRTABMapCameraModel(cameraId, width, height, localTransform, alphaScaling);
}
}  // namespace node
}  // namespace dai