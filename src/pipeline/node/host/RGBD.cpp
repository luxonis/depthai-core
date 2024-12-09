#include "depthai/pipeline/node/host/RGBD.hpp"

#include "common/Point3fRGB.hpp"
#include "depthai/common/Point3fRGB.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/Sync.hpp"

namespace dai {
namespace node {

std::shared_ptr<RGBD> RGBD::build() {
    // Link the inputs
    sync->setRunOnHost(false);
    sync->out.link(inSync);
    inColor.setBlocking(false);
    inColor.setMaxSize(1);
    inDepth.setBlocking(false);
    inDepth.setMaxSize(1);
    inSync.setMaxSize(1);
    inSync.setBlocking(false);
    return std::static_pointer_cast<RGBD>(shared_from_this());
}

// std::shared_ptr<RGBD> RGBD::build(bool autocreate) {
// if(!autocreate) {
//     return std::static_pointer_cast<RGBD>(shared_from_this());
// }
// auto pipeline = getParentPipeline();
// auto colorCam = pipeline.create<node::Camera>();
// auto depth = pipeline.create<node::StereoDepth>();
// return build(, depth->depth);
// }

void RGBD::initialize(std::vector<std::shared_ptr<ImgFrame>> frames) {
    // Initialize the camera intrinsics
    auto frame = frames.at(0);
    auto calibHandler = getParentPipeline().getDefaultDevice()->readCalibration();
    auto camID = static_cast<CameraBoardSocket>(frame->getInstanceNum());
    auto intrinsics = calibHandler.getCameraIntrinsics(camID, frame->getWidth(), frame->getHeight());
    fx = intrinsics[0][0];
    fy = intrinsics[1][1];
    cx = intrinsics[0][2];
    cy = intrinsics[1][2];
    initialized = true;
}

void RGBD::run() {
    while(isRunning()) {
        // Get the color and depth frames
        auto group = inSync.tryGet<MessageGroup>();
        if(group == nullptr) continue;
        if(!initialized) {
            std::vector<std::shared_ptr<ImgFrame>> imgFrames;
            for(auto& msg : *group) {
                imgFrames.emplace_back(std::dynamic_pointer_cast<ImgFrame>(msg.second));
            }

            initialize(imgFrames);
        }
        auto colorFrame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(inColor.getName()));
        auto depthFrame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(inDepth.getName()));

        // Create the point cloud
        auto pc = std::make_shared<PointCloudData>();
        pc->setTimestamp(colorFrame->getTimestamp());
        pc->setTimestampDevice(colorFrame->getTimestampDevice());
        pc->setSequenceNum(colorFrame->getSequenceNum());
        pc->setInstanceNum(colorFrame->getInstanceNum());
        uint width = colorFrame->getWidth();
        uint height = colorFrame->getHeight();
        pc->setSize(width, height);

        std::vector<Point3fRGB> points;
        const auto& size = width * height;
        points.reserve(size);
        // Fill the point cloud
        auto depthData = depthFrame->getCvFrame();
        auto colorData = colorFrame->getCvFrame();
        for(unsigned int i = 0; i < height; i++) {
            for(unsigned int j = 0; j < width; j++) {
                // Get the depth value as UINT16
                uint16_t depthValue = depthData.at<uint16_t>(i, j);
                // Convert the depth value to meters
                float z = depthValue / 1000.0f;
                // limit the depth value to 10m
                if(z > 10.0f) {
                    z = 10.0f;
                }

                // Calculate the 3D point
                float x = (j - cy) * z / fy;
                float y = (i - cx) * z / fx;

                // Get the color
                auto color = colorData.at<cv::Vec3b>(i, j);
                // Add the point to the point cloud
                points.emplace_back(Point3fRGB{x, y, z, static_cast<uint8_t>(color[2]), static_cast<uint8_t>(color[1]), static_cast<uint8_t>(color[0])});
            }
        }
        pc->setPointsRGB(points);
        pcl.send(pc);
    }
}
}  // namespace node
}  // namespace dai
