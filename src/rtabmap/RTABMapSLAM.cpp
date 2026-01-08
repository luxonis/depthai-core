#include "depthai/rtabmap/RTABMapSLAM.hpp"

#include "../utility/PimplImpl.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/rtabmap/RTABMapConversions.hpp"
#include "pcl/filters/filter.h"
#include "pcl/point_cloud.h"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/ImgFrame.hpp"
#include "pipeline/datatype/MapData.hpp"
#include "pipeline/datatype/TransformData.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/LocalGrid.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/global_map/CloudMap.h"
#include "rtabmap/core/global_map/OccupancyGrid.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_mapping.h"
#include "spdlog/spdlog.h"

namespace dai {
namespace node {
class RTABMapSLAM::Impl {
   public:
    Impl() = default;
    void setLocalTransform(std::shared_ptr<TransformData> transform) {
        localTransform = getRTABMapTransform(transform->transform);
    }
    std::shared_ptr<TransformData> getLocalTransform() {
        return rtabmapToTransformData(localTransform);
    }
    void setPublishObstacleCloud(bool publish) {
        publishObstacleCloud = publish;
    }
    void setPublishGroundCloud(bool publish) {
        publishGroundCloud = publish;
    }
    void setPublishGrid(bool publish) {
        publishGrid = publish;
    }
    void publishGridMap(const std::map<int, rtabmap::Transform>& optimizedPoses, dai::Node::Output& occupancyGridMap) {
        if(occupancyGrid->addedNodes().size() || localMaps->size() > 0) {
            occupancyGrid->update(optimizedPoses);
        }
        float xMin, yMin;
        cv::Mat map = occupancyGrid->getMap(xMin, yMin);
        if(!map.empty()) {
            cv::Mat map8U = rtabmap::util3d::convertMap2Image8U(map);
            cv::flip(map8U, map8U, 0);

            auto mapMsg = std::make_shared<dai::MapData>();
            dai::ImgFrame mapImg;
            mapImg.setTimestamp(std::chrono::steady_clock::now());
            mapImg.setCvFrame(map8U, ImgFrame::Type::GRAY8);
            mapMsg->ts = mapImg.ts;
            mapMsg->minX = xMin;
            mapMsg->minY = yMin;
            mapMsg->map = mapImg;

            occupancyGridMap.send(mapMsg);
        }
    }

    void publishPointClouds(const std::map<int, rtabmap::Transform>& optimizedPoses, dai::Node::Output& obstaclePCL, dai::Node::Output& groundPCL) {
        if(cloudMap->addedNodes().size() || localMaps->size() > 0) {
            cloudMap->update(optimizedPoses);
        }

        if(publishObstacleCloud) {
            auto obstaclesMap = cloudMap->getMapObstacles();
            // convert point to point3frgba

            auto pclData = setPclDataRGB(obstaclesMap);
            obstaclePCL.send(pclData);
        }
        if(publishGroundCloud) {
            auto groundMap = cloudMap->getMapGround();
            auto pclData = setPclDataRGB(groundMap);
            groundPCL.send(pclData);
        }
    }
    std::shared_ptr<dai::PointCloudData> setPclDataRGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        // Ensure cloud is valid
        if(!cloud) {
            throw std::invalid_argument("Input cloud is null");
        }
        auto pcl = std::make_shared<dai::PointCloudData>();

        auto size = cloud->points.size();
        std::vector<uint8_t> data(size * sizeof(Point3fRGBA));
        auto* dataPtr = reinterpret_cast<Point3fRGBA*>(data.data());
        pcl->setWidth(cloud->width);
        pcl->setHeight(cloud->height);
        pcl->setSparse(!cloud->is_dense);

        std::for_each(cloud->points.begin(), cloud->points.end(), [dataPtr, &cloud](const pcl::PointXYZRGB& point) mutable {
            size_t i = &point - &cloud->points[0];
            dataPtr[i] = Point3fRGBA{point.x, point.y, point.z, point.r, point.g, point.b};
        });
        pcl->setColor(true);
        pcl->setData(data);
        return pcl;
    }

    rtabmap::StereoCameraModel model;
    rtabmap::Rtabmap rtabmap;
    rtabmap::Transform currPose, odomCorr;
    std::chrono::steady_clock::time_point lastProcessTime;
    std::chrono::steady_clock::time_point startTime;
    rtabmap::Transform imuLocalTransform;
    rtabmap::Transform localTransform;
    std::shared_ptr<rtabmap::LocalGridCache> localMaps;
    std::unique_ptr<rtabmap::OccupancyGrid> occupancyGrid;
    std::unique_ptr<rtabmap::CloudMap> cloudMap;
    rtabmap::SensorData sensorData;
    bool publishObstacleCloud = true;
    bool publishGroundCloud = true;
    bool publishGrid = true;
};

void RTABMapSLAM::buildInternal() {
    sync->out.link(inSync);
    sync->setRunOnHost(true);
    alphaScaling = -1.0;
    pimplRtabmap->localTransform = rtabmap::Transform::getIdentity();
    rtabmap::Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
    pimplRtabmap->localTransform = pimplRtabmap->localTransform * opticalTransform;
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
    pimplRtabmap->localMaps = std::make_shared<rtabmap::LocalGridCache>();
}
RTABMapSLAM::RTABMapSLAM() = default;
RTABMapSLAM::~RTABMapSLAM() {
    auto& logger = pimpl->logger;

    if(saveDatabaseOnClose) {
        if(databasePath.empty()) {
            databasePath = "/tmp/rtabmap.db";
        }
        logger->info("Saving database at {}", databasePath);
        pimplRtabmap->rtabmap.close(true, databasePath);
    }
}
void RTABMapSLAM::setParams(const rtabmap::ParametersMap& params) {
    rtabParams = params;
}

void RTABMapSLAM::triggerNewMap() {
    pimplRtabmap->rtabmap.triggerNewMap();
}

void RTABMapSLAM::saveDatabase() {
    pimplRtabmap->rtabmap.close();
    pimplRtabmap->rtabmap.init(rtabParams, databasePath);
}

std::shared_ptr<TransformData> RTABMapSLAM::getLocalTransform() {
    return pimplRtabmap->getLocalTransform();
}
void RTABMapSLAM::setLocalTransform(std::shared_ptr<TransformData> transform) {
    pimplRtabmap->setLocalTransform(transform);
}
void RTABMapSLAM::setUseFeatures(bool use) {
    useFeatures = use;
    if(useFeatures) {
        features.setBlocking(false);
        features.setMaxSize(1);
        inputs[featuresInputName] = features;
    }
}

void RTABMapSLAM::setPublishGrid(bool publish) {
    pimplRtabmap->setPublishGrid(publish);
}

void RTABMapSLAM::setPublishGroundCloud(bool publish) {
    pimplRtabmap->setPublishGroundCloud(publish);
}

void RTABMapSLAM::setPublishObstacleCloud(bool publish) {
    pimplRtabmap->setPublishObstacleCloud(publish);
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

            pimplRtabmap->sensorData =
                rtabmap::SensorData(imgFrame->getCvFrame(), depthFrame->getCvFrame(), pimplRtabmap->model.left(), imgFrame->getSequenceNum(), stamp);
            std::vector<cv::KeyPoint> keypoints;
            if(featuresFrame != nullptr) {
                for(auto& feature : featuresFrame->trackedFeatures) {
                    keypoints.emplace_back(cv::KeyPoint(feature.position.x, feature.position.y, 3));
                }
                pimplRtabmap->sensorData.setFeatures(keypoints, std::vector<cv::Point3f>(), cv::Mat());
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
    rtabmap::Transform p = getRTABMapTransform(odomPose->transform);
    pimplRtabmap->currPose = p;

    auto outTransform = rtabmapToTransformData(pimplRtabmap->odomCorr * pimplRtabmap->currPose);
    auto outCorrection = rtabmapToTransformData(pimplRtabmap->odomCorr);
    // set unit rotation for outCorrection
    transform.send(outTransform);
    odomCorrection.send(outCorrection);
    passthroughOdom.send(odomPose);
}

void RTABMapSLAM::run() {
    auto& logger = pimpl->logger;
    while(mainLoop()) {
        if(!initialized) {
            continue;
        } else {
            rtabmap::Statistics stats;
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            if(now - pimplRtabmap->lastProcessTime > std::chrono::milliseconds(int(1000.0f / freq))) {
                pimplRtabmap->lastProcessTime = now;
                bool success = pimplRtabmap->rtabmap.process(pimplRtabmap->sensorData, pimplRtabmap->currPose);
                if(success) {
                    stats = pimplRtabmap->rtabmap.getStatistics();
                    if(pimplRtabmap->rtabmap.getLoopClosureId() > 0) {
                        logger->debug("Loop closure detected! last loop closure id = {}", pimplRtabmap->rtabmap.getLoopClosureId());
                    }
                    pimplRtabmap->odomCorr = stats.mapCorrection();

                    const std::map<int, rtabmap::Transform>& optimizedPoses = pimplRtabmap->rtabmap.getLocalOptimizedPoses();

                    if(optimizedPoses.find(stats.getLastSignatureData().id()) != optimizedPoses.end()) {
                        const rtabmap::Signature& node = stats.getLastSignatureData();
                        pimplRtabmap->localMaps->add(node.id(),
                                                     node.sensorData().gridGroundCellsRaw(),
                                                     node.sensorData().gridObstacleCellsRaw(),
                                                     node.sensorData().gridEmptyCellsRaw(),
                                                     node.sensorData().gridCellSize(),
                                                     node.sensorData().gridViewPoint());
                    }

                    if(publishGrid) {
                        pimplRtabmap->publishGridMap(optimizedPoses, occupancyGridMap);
                    }

                    if(publishObstacleCloud || publishGroundCloud) {
                        pimplRtabmap->publishPointClouds(optimizedPoses, obstaclePCL, groundPCL);
                    }
                }
            }
        }
        // save database periodically if set
        if(saveDatabasePeriodically
           && std::chrono::duration<double>(std::chrono::steady_clock::now() - pimplRtabmap->startTime).count() > databaseSaveInterval) {
            pimplRtabmap->rtabmap.close(true, databasePath);
            pimplRtabmap->rtabmap.init(rtabParams, databasePath);
            logger->info("Database saved at {}", databasePath);
            pimplRtabmap->startTime = std::chrono::steady_clock::now();
        }
    }
}

void RTABMapSLAM::initialize(dai::Pipeline& pipeline, int instanceNum, int width, int height) {
    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();
    auto cameraId = static_cast<dai::CameraBoardSocket>(instanceNum);
    pimplRtabmap->model = getRTABMapCameraModel(cameraId, width, height, pimplRtabmap->localTransform, alphaScaling, calibHandler);
    if(!databasePath.empty()) {
        pimplRtabmap->rtabmap.init(rtabParams, databasePath);
    } else {
        pimplRtabmap->rtabmap.init(rtabParams);
    }
    pimplRtabmap->lastProcessTime = std::chrono::steady_clock::now();
    pimplRtabmap->startTime = std::chrono::steady_clock::now();
    pimplRtabmap->occupancyGrid = std::make_unique<rtabmap::OccupancyGrid>(pimplRtabmap->localMaps.get(), rtabParams);
    pimplRtabmap->cloudMap = std::make_unique<rtabmap::CloudMap>(pimplRtabmap->localMaps.get(), rtabParams);
    initialized = true;
}
}  // namespace node
}  // namespace dai
