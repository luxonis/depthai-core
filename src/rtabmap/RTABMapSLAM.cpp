#include "depthai/rtabmap/RTABMapSLAM.hpp"

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <spdlog/spdlog.h>

#include "depthai/pipeline/Pipeline.hpp"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_mapping.h"

namespace dai {
namespace node {
std::shared_ptr<RTABMapSLAM> RTABMapSLAM::build() {
    sync->out.link(inSync);
    sync->setRunOnHost(false);
    alphaScaling = -1.0;
    localTransform = rtabmap::Transform::getIdentity();
    rtabmap::Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    localTransform = localTransform * opticalTransform;
    rect.setBlocking(false);
    rect.setMaxSize(1);
    depth.setBlocking(false);
    depth.setMaxSize(1);
    inSync.setMaxSize(1);
    inSync.setBlocking(false);
    inSync.addCallback(std::bind(&RTABMapSLAM::syncCB, this, std::placeholders::_1));
    odom.setMaxSize(1);
    odom.setBlocking(false);
    odom.addCallback(std::bind(&RTABMapSLAM::odomPoseCB, this, std::placeholders::_1));
    localMaps = std::make_shared<rtabmap::LocalGridCache>();
    return std::static_pointer_cast<RTABMapSLAM>(shared_from_this());
}

RTABMapSLAM::~RTABMapSLAM() {
    std::cout << "RTABMapSLAM destructor" << std::endl;
}
void RTABMapSLAM::setParams(const rtabmap::ParametersMap& params) {
    rtabParams = params;
}

void RTABMapSLAM::triggerNewMap() {
    rtabmap.triggerNewMap();
}

void RTABMapSLAM::saveDatabase() {
    rtabmap.close();
    rtabmap.init(rtabParams, databasePath);
}

void RTABMapSLAM::setUseFeatures(bool use) {
    useFeatures = use;
    if(useFeatures) {
        features.setBlocking(false);
        features.setMaxSize(1);
        inputs[featuresInputName] = features;
    }
}

void RTABMapSLAM::syncCB(std::shared_ptr<dai::ADatatype> data) {
    auto group = std::dynamic_pointer_cast<dai::MessageGroup>(data);
    if(group == nullptr) return;
    std::shared_ptr<dai::ImgFrame> imgFrame = nullptr;
    std::shared_ptr<dai::ImgFrame> depthFrame = nullptr;
    std::shared_ptr<dai::TrackedFeatures> featuresFrame = nullptr;
    imgFrame = group->get<dai::ImgFrame>(rectInputName);
    depthFrame = group->get<dai::ImgFrame>(depthInputName);
    if(useFeatures) {
        featuresFrame = group->get<dai::TrackedFeatures>(featuresInputName);
    }
    if(imgFrame != nullptr && depthFrame != nullptr) {
        if(!initialized) {
            auto pipeline = getParentPipeline();
            initialize(pipeline, imgFrame->getInstanceNum(), imgFrame->getWidth(), imgFrame->getHeight());
        } else {
            double stamp = std::chrono::duration<double>(imgFrame->getTimestampDevice(dai::CameraExposureOffset::MIDDLE).time_since_epoch()).count();

            sensorData = rtabmap::SensorData(imgFrame->getCvFrame(), depthFrame->getCvFrame(), model.left(), imgFrame->getSequenceNum(), stamp);
            std::vector<cv::KeyPoint> keypoints;
            if(featuresFrame != nullptr) {
                for(auto& feature : featuresFrame->trackedFeatures) {
                    keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
                }
                sensorData.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
            }
        }
        passthroughRect.send(imgFrame);
        passthroughDepth.send(depthFrame);
        if(useFeatures) {
            passthroughFeatures.send(featuresFrame);
        }
    }
}

void RTABMapSLAM::odomPoseCB(std::shared_ptr<dai::ADatatype> data) {
    auto odomPose = std::dynamic_pointer_cast<dai::TransformData>(data);
    // convert odom pose to rtabmap pose
    rtabmap::Transform p = odomPose->getRTABMapTransform();
    currPose = p;

    auto outTransform = std::make_shared<dai::TransformData>(odomCorr * currPose);
    auto outCorrection = std::make_shared<dai::TransformData>(odomCorr);
    transform.send(outTransform);
    odomCorrection.send(outCorrection);
    passthroughOdom.send(odomPose);
}

void RTABMapSLAM::run() {
    while(isRunning()) {
        if(!initialized) {
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
                        logger->debug("Loop closure detected! last loop closure id = {}", rtabmap.getLoopClosureId());
                    }
                    odomCorr = stats.mapCorrection();

                    std::map<int, rtabmap::Signature> nodes;
                    std::map<int, rtabmap::Transform> optimizedPoses;
                    std::multimap<int, rtabmap::Link> links;

                    rtabmap.getGraph(optimizedPoses, links, true, true, &nodes, true, true, true, true);
                    for(std::map<int, rtabmap::Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter) {
                        rtabmap::Signature node = nodes.find(iter->first)->second;
                        if(node.sensorData().gridCellSize() == 0.0f) {
                            logger->warn("Grid cell size is 0, skipping node {}", iter->first);
                            continue;
                        }
                        // uncompress grid data
                        cv::Mat ground, obstacles, empty;
                        node.sensorData().uncompressData(0, 0, 0, 0, &ground, &obstacles, &empty);
                        localMaps->add(iter->first, ground, obstacles, empty, node.sensorData().gridCellSize(), node.sensorData().gridViewPoint());
                    }

                    if(publishGrid) {
                        publishGridMap(optimizedPoses);
                    }

                    if(publishObstacleCloud || publishGroundCloud) {
                        publishPointClouds(optimizedPoses);
                    }
                }
            }
        }
        // save database periodically if set
        if(saveDatabasePeriodically && std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count() > databaseSaveInterval) {
            rtabmap.close();
            rtabmap.init(rtabParams, databasePath);
            logger->info("Database saved at {}", databasePath);
            startTime = std::chrono::steady_clock::now();
        }
    }
}

void RTABMapSLAM::publishGridMap(const std::map<int, rtabmap::Transform>& optimizedPoses) {
    if(occupancyGrid->addedNodes().size() || localMaps->size() > 0) {
        occupancyGrid->update(optimizedPoses);
    }
    float xMin, yMin;
    cv::Mat map = occupancyGrid->getMap(xMin, yMin);
    if(!map.empty()) {
        cv::Mat map8U = rtabmap::util3d::convertMap2Image8U(map);
        cv::flip(map8U, map8U, 0);

        auto mapMsg = std::make_shared<dai::ImgFrame>();
        mapMsg->setTimestamp(std::chrono::steady_clock::now());
        mapMsg->setCvFrame(map8U, ImgFrame::Type::GRAY8);
        occupancyGridMap.send(mapMsg);
    }
}

void RTABMapSLAM::publishPointClouds(const std::map<int, rtabmap::Transform>& optimizedPoses) {
    if(cloudMap->addedNodes().size() || localMaps->size() > 0) {
        cloudMap->update(optimizedPoses);
    }

    if(publishObstacleCloud) {
        auto obstaclesMap = cloudMap->getMapObstacles();
        auto pclData = std::make_shared<dai::PointCloudData>();
        pclData->setPclData(obstaclesMap);
        obstaclePCL.send(pclData);
    }
    if(publishGroundCloud) {
        auto groundMap = cloudMap->getMapGround();
        auto pclData = std::make_shared<dai::PointCloudData>();
        pclData->setPclData(groundMap);
        groundPCL.send(pclData);
    }
}

void RTABMapSLAM::initialize(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();
    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    model = calibHandler.getRTABMapCameraModel(cameraId, width, height, localTransform, alphaScaling);
    if(!databasePath.empty()) {
        rtabmap.init(rtabParams, databasePath);
    }
    else {
        rtabmap.init(rtabParams);
    }
    lastProcessTime = std::chrono::steady_clock::now();
    startTime = std::chrono::steady_clock::now();
    occupancyGrid = std::make_unique<rtabmap::OccupancyGrid>(localMaps.get(), rtabParams);
    cloudMap = std::make_unique<rtabmap::CloudMap>(localMaps.get(), rtabParams);
    initialized = true;
}
}  // namespace node
}  // namespace dai