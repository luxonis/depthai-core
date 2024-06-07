#pragma once
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#ifdef DEPTHAI_HAVE_PCL_SUPPORT
    #include <pcl/point_types.h>
#endif
#include "rerun.hpp"

rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat& img) {
    return {img.rows, img.cols, img.channels()};
};
class RerunStreamer : public dai::NodeCRTP<dai::node::ThreadedHostNode, RerunStreamer> {
   public:
    constexpr static const char* NAME = "RerunStreamer";

   public:
    void build() {
    }

    Input inputTrans{*this, {.name="inTrans", .types={{dai::DatatypeEnum::TransformData, true}}}};
    Input inputImg{*this, {.name="inImg", .types={{dai::DatatypeEnum::ImgFrame, true}}}};
    Input inputObstaclePCL{*this, {.name="inObstaclePCL", .types={{dai::DatatypeEnum::PointCloudData, true}}}};
    Input inputGroundPCL{*this, {.name="inGroundPCL", .types={{dai::DatatypeEnum::PointCloudData, true}}}};
    Input inputMap{*this, {.name="inMap", .types={{dai::DatatypeEnum::ImgFrame, true}}}};

    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_timeless("world", rerun::ViewCoordinates::FLU);
        rec.log("world/ground", rerun::Boxes3D::from_half_sizes({{3.f, 3.f, 0.00001f}}));
        while(isRunning()) {
            std::shared_ptr<dai::TransformData> transData = inputTrans.get<dai::TransformData>();
            auto imgFrame = inputImg.get<dai::ImgFrame>();
            auto pclObstData = inputObstaclePCL.tryGet<dai::PointCloudData>();
            auto pclGrndData = inputGroundPCL.tryGet<dai::PointCloudData>();
            auto mapData = inputMap.tryGet<dai::ImgFrame>();
            if(transData != nullptr) {
                auto trans = transData->getTranslation();
                auto quat = transData->getQuaternion();

                auto position = rerun::Vec3D(trans.x, trans.y, trans.z);

                rec.log("world/camera", rerun::Transform3D(position, rerun::datatypes::Quaternion::from_xyzw(quat.qx, quat.qy, quat.qz, quat.qw)));
                positions.push_back(position);
                rerun::LineStrip3D lineStrip(positions);
                rec.log("world/trajectory", rerun::LineStrips3D(lineStrip));
                rec.log("world/camera/image", rerun::Pinhole::from_focal_length_and_resolution({398.554f, 398.554f}, {640.0f, 400.0f}).with_camera_xyz(rerun::components::ViewCoordinates::FLU));
                rec.log("world/camera/image/rgb",
                        rerun::Image(tensor_shape(imgFrame->getCvFrame()), reinterpret_cast<const uint8_t*>(imgFrame->getCvFrame().data)));
                #ifdef DEPTHAI_HAVE_PCL_SUPPORT
                if(pclObstData != nullptr) {
                    std::vector<rerun::Position3D> points;
                    for(auto& point : pclObstData->getPclData()->points) {
                        points.push_back(rerun::Position3D(point.x, point.y, point.z));
                    }
                    rec.log("world/obstacle_pcl", rerun::Points3D(points).with_radii({0.01f}));
                }
                if(pclGrndData != nullptr) {
                    std::vector<rerun::Position3D> points;
                    for(auto& point : pclGrndData->getPclData()->points) {
                        points.push_back(rerun::Position3D(point.x, point.y, point.z));
                    }
                    rec.log("world/ground_pcl", rerun::Points3D(points).with_colors(rerun::Color{0,255,0}).with_radii({0.01f}));
                }
                #endif
                if(mapData != nullptr) {
                    rec.log("map", rerun::Image(tensor_shape(mapData->getCvFrame()), reinterpret_cast<const uint8_t*>(mapData->getCvFrame().data)));
                }
            }
        }
    }
    std::vector<rerun::Vec3D> positions;
};