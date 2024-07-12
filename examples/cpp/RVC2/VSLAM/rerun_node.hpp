#pragma once
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#ifdef DEPTHAI_HAVE_PCL_SUPPORT
    #include <pcl/point_types.h>
#endif
#include "rerun.hpp"


rerun::Collection<rerun::TensorDimension> tensorShape(const cv::Mat& img) {
    return {size_t(img.rows), size_t(img.cols), size_t(img.channels())};
};
class RerunNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, RerunNode> {
   public:
    constexpr static const char* NAME = "RerunNode";

   public:
    void build() {}

    Input inputTrans{*this, {.name = "inTrans", .types = {{dai::DatatypeEnum::TransformData, true}}}};
    Input inputImg{*this, {.name = "inImg", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};
    Input inputObstaclePCL{*this, {.name = "inObstaclePCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
    Input inputGroundPCL{*this, {.name = "inGroundPCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
    Input inputMap{*this, {.name = "inMap", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};

    void getFocalLengthFromImage(std::shared_ptr<dai::ImgFrame> imgFrame) {
        auto p = getParentPipeline();
        auto calibHandler = p.getDefaultDevice()->readCalibration();
        auto intrinsics =
            calibHandler.getCameraIntrinsics(static_cast<dai::CameraBoardSocket>(imgFrame->getInstanceNum()), imgFrame->getWidth(), imgFrame->getHeight());
        fx = intrinsics[0][0];
        fy = intrinsics[1][1];
        intrinsicsSet = true;
    }

    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_static("world", rerun::ViewCoordinates::FLU);
        rec.log("world/ground", rerun::Boxes3D::from_half_sizes({{3.f, 3.f, 0.00001f}}));
        while(isRunning()) {
            std::shared_ptr<dai::TransformData> transData = inputTrans.get<dai::TransformData>();
            auto imgFrame = inputImg.get<dai::ImgFrame>();
            if(!intrinsicsSet) {
                getFocalLengthFromImage(imgFrame);
            }
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
                rec.log("world/camera/image",
                        rerun::Pinhole::from_focal_length_and_resolution({fx, fy}, {float(imgFrame->getWidth()), float(imgFrame->getHeight())})
                            .with_camera_xyz(rerun::components::ViewCoordinates::FLU));
                auto image = imgFrame->getCvFrame();
                cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
                rec.log("world/camera/image/rgb",
                        rerun::Image(tensorShape(image), reinterpret_cast<const uint8_t*>(image.data)));
#ifdef DEPTHAI_HAVE_PCL_SUPPORT
                if(pclObstData != nullptr) {
                    std::vector<rerun::Position3D> points;
                    std::vector<rerun::Color> colors;
                    const auto& size = pclObstData->getWidth() * pclObstData->getHeight();
                    points.reserve(size);
                    colors.reserve(size);
                    const auto& pclData = pclObstData->getPointsRGB();
                    for(size_t i = 0; i < size; ++i) {
                        points.push_back(rerun::Position3D(pclData[i].x, pclData[i].y, pclData[i].z));
                        colors.push_back(rerun::Color{pclData[i].r, pclData[i].g, pclData[i].b});
                    }
                    rec.log("world/obstacle_pcl", rerun::Points3D(points).with_colors(colors).with_radii({0.01f}));
                }
                if(pclGrndData != nullptr) {
                    std::vector<rerun::Position3D> points;
                    std::vector<rerun::Color> colors;
                    const auto& size = pclGrndData->getWidth() * pclGrndData->getHeight();
                    points.reserve(size);
                    colors.reserve(size);
                    const auto& pclData = pclGrndData->getPointsRGB();
                    for(size_t i = 0; i < size; ++i) {
                        points.push_back(rerun::Position3D(pclData[i].x, pclData[i].y, pclData[i].z));
                        colors.push_back(rerun::Color{pclData[i].r, pclData[i].g, pclData[i].b});
                    }
                    rec.log("world/ground_pcl", rerun::Points3D(points).with_colors(colors).with_radii({0.01f}));
                }
#endif
                if(mapData != nullptr) {
                    rec.log("map", rerun::Image(tensorShape(mapData->getCvFrame()), reinterpret_cast<const uint8_t*>(mapData->getCvFrame().data)));
                }
            }
        }
    }
    std::vector<rerun::Vec3D> positions;
    float fx = 400.0;
    float fy = 400.0;
    bool intrinsicsSet = false;
};