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

#include "kompute/Kompute.hpp"
#include "shaders/rgbd2pointcloud.hpp"

#include <future>

namespace dai {
namespace node {

std::shared_ptr<RGBD> RGBD::build() {
    align = getParentPipeline().create<node::ImageAlign>();
    align->outputAligned.link(inDepthSync);
    colorMux.link(inColorSync);
    colorMux.link(align->inputAlignTo);
    depthPT.link(align->input);
    sync->setRunOnHost(false);
    sync->out.link(inSync);
    inSync.setMaxSize(1);
    inSync.setBlocking(false);
    inDepth.addCallback([this](const std::shared_ptr<ADatatype>& data) { depthPT.send(data); });
    inColor.addCallback([this](const std::shared_ptr<ADatatype>& data) { colorMux.send(data); });
    mgr = std::make_shared<kp::Manager>(deviceIndex);
    return std::static_pointer_cast<RGBD>(shared_from_this());
}

std::shared_ptr<RGBD> RGBD::build(bool autocreate, std::pair<int, int> size) {
    if(!autocreate) {
        return std::static_pointer_cast<RGBD>(shared_from_this());
    }
    auto pipeline = getParentPipeline();
    auto colorCam = pipeline.create<node::Camera>()->build();
    auto depth = pipeline.create<node::StereoDepth>()->build(true);
    auto* out = colorCam->requestOutput(size);
    out->link(inColor);
    depth->depth.link(inDepth);
    return build();
}

void RGBD::initialize(std::vector<std::shared_ptr<ImgFrame>> frames) {
    // Initialize the camera intrinsics
    auto frame = frames.at(1);
    auto calibHandler = getParentPipeline().getDefaultDevice()->readCalibration();
    auto camID = static_cast<CameraBoardSocket>(frame->getInstanceNum());
    auto intrinsics = calibHandler.getCameraIntrinsics(camID, frame->getWidth(), frame->getHeight());
    fx = intrinsics[0][0];
    fy = intrinsics[1][1];
    cx = intrinsics[0][2];
    cy = intrinsics[1][2];
    initialized = true;
}

// GPU-based function to compute the point cloud using vulkan-kompute
void RGBD::computePointCloudGPU(
    const cv::Mat& depthMat,
    const cv::Mat& colorMat,
    std::vector<float>& xyzOut,
    std::vector<uint8_t>& rgbOut
) {
    using namespace kp;

    int width = depthMat.cols;
    int height = depthMat.rows;
    size_t size = width * height;

    xyzOut.resize(size * 3);
    rgbOut.resize(size * 3);

    // Convert depth to float
    // Depth is in mm by default, convert to meters if outputMeters == true
    // If outputMeters is true, scale = 1/1000.0 else scale = 1.0
    float scale = outputMeters ? (1.0f / 1000.0f) : 1.0f;

    std::vector<float> depthData(size);
    for(size_t i = 0; i < size; i++) {
        uint16_t d = depthMat.at<uint16_t>(i / width, i % width);
        depthData[i] = (float)d; // will multiply by scale in shader
    }

    // Convert color to float (R,G,B)
    std::vector<float> colorDataFloat(size * 3);
    for(size_t i = 0; i < size; i++) {
        auto c = colorMat.at<cv::Vec3b>(i / width, i % width);
        // OpenCV: c = [B, G, R]
        colorDataFloat[i * 3 + 0] = (float)c[2]; // R
        colorDataFloat[i * 3 + 1] = (float)c[1]; // G
        colorDataFloat[i * 3 + 2] = (float)c[0]; // B
    }

    // Intrinsics: [fx, fy, cx, cy, scale, width, height]
    std::vector<float> intrinsics = {fx, fy, cx, cy, scale, (float)width, (float)height};


    // Create Kompute tensors
    auto depthTensor = mgr->tensor(depthData);
    auto rgbTensor = mgr->tensor(colorDataFloat);
    auto intrinsicsTensor = mgr->tensor(intrinsics);
    auto xyzTensor = mgr->tensor(xyzOut);
    // We'll store output RGB as float as well, then convert back
    auto outRgbTensor = mgr->tensorT<float>(std::vector<float>(rgbOut.size()));

    // Load shader
    const std::vector<uint32_t> shader = std::vector<uint32_t>(shaders::RGBD2POINTCLOUD_COMP_SPV.begin(), shaders::RGBD2POINTCLOUD_COMP_SPV.end());
    const std::vector<std::shared_ptr<kp::Memory>> tensors = {depthTensor, rgbTensor, intrinsicsTensor, xyzTensor, outRgbTensor};
    auto algo = mgr->algorithm(tensors, shader);
    mgr->sequence()->record<kp::OpSyncDevice>(tensors)->record<kp::OpAlgoDispatch>(algo)->record<kp::OpSyncLocal>(tensors)->eval();
    // Retrieve results
    xyzOut = xyzTensor->vector();
    std::vector<float> outRgbFloat = outRgbTensor->vector();
    for(size_t i = 0; i < rgbOut.size(); i++) {
        rgbOut[i] = static_cast<uint8_t>(std::round(outRgbFloat[i]));
    }
}

 void RGBD::computePointCloudCPU(
    const cv::Mat& depthMat,
    const cv::Mat& colorMat,
    std::vector<Point3fRGB>& points
) {
    int width = depthMat.cols;
    int height = depthMat.rows;
    points.resize(width * height);

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            uint16_t depthValue = depthMat.at<uint16_t>(i, j);
            float z =  depthValue ;
            float x = (j - cx) * z / fx;
            float y = (i - cy) * z / fy;

            auto color = colorMat.at<cv::Vec3b>(i, j);
            points[i * width + j] = Point3fRGB{
                x, y, z,
                static_cast<uint8_t>(color[2]),
                static_cast<uint8_t>(color[1]),
                static_cast<uint8_t>(color[0])
            };
        }
    }
}

void RGBD::computePointCloudCPUMT(
    const cv::Mat& depthMat,
    const cv::Mat& colorMat,
    std::vector<Point3fRGB>& points
) {

    int width = depthMat.cols;
    int height = depthMat.rows;
    points.resize(width * height);
                // Lambda function for processing a block of rows
        auto processRows = [&](int startRow, int endRow) {
            for (int i = startRow; i < endRow; i++) {
                for (int j = 0; j < width; j++) {
                    uint16_t depthValue = depthMat.at<uint16_t>(i, j);
                    float z =  depthValue ;
                    float x = (j - cx) * z / fx;
                    float y = (i - cy) * z / fy;

                    auto color = colorMat.at<cv::Vec3b>(i, j);
                    points[i * width + j] = Point3fRGB{
                        x, y, z,
                        static_cast<uint8_t>(color[2]),
                        static_cast<uint8_t>(color[1]),
                        static_cast<uint8_t>(color[0])
                    };
                }
            }
        };

        // Divide rows into chunks and process in parallel
        const int numThreads = std::thread::hardware_concurrency();
        const int rowsPerThread = height / numThreads;
        std::vector<std::future<void>> futures;

        for (int t = 0; t < numThreads; ++t) {
            int startRow = t * rowsPerThread;
            int endRow = (t == numThreads - 1) ? height : startRow + rowsPerThread;

            futures.emplace_back(std::async(std::launch::async, processRows, startRow, endRow));
        }

        // Wait for all threads to finish
        for (auto& future : futures) {
            future.get();
        }

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
        auto colorFrame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(inColorSync.getName()));
        auto depthFrame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(inDepthSync.getName()));

        // Create the point cloud
        auto pc = std::make_shared<PointCloudData>();
        pc->setTimestamp(colorFrame->getTimestamp());
        pc->setTimestampDevice(colorFrame->getTimestampDevice());
        pc->setSequenceNum(colorFrame->getSequenceNum());
        pc->setInstanceNum(colorFrame->getInstanceNum());
        uint width = colorFrame->getWidth();
        uint height = colorFrame->getHeight();
        pc->setSize(width, height);

        std::vector<Point3fRGB> points(width * height);
        // Fill the point cloud
        auto depthData = depthFrame->getCvFrame();
        auto colorData = colorFrame->getCvFrame();
        // Use GPU to compute point cloud
        std::vector<float> xyzBuffer;
        std::vector<uint8_t> rgbBuffer;
        computePointCloudGPU(depthData, colorData, xyzBuffer, rgbBuffer);

        // Convert xyzBuffer & rgbBuffer to Point3fRGB vector
        points.reserve(width * height);
        for(size_t i = 0; i < width*height; i++) {
            Point3fRGB p;
            p.x = xyzBuffer[i*3+0];
            p.y = xyzBuffer[i*3+1];
            p.z = xyzBuffer[i*3+2];
            p.r = rgbBuffer[i*3+0];
            p.g = rgbBuffer[i*3+1];
            p.b = rgbBuffer[i*3+2];
            points.push_back(p);
        }
        pc->setPointsRGB(points);
        pcl.send(pc);
    }
}
}  // namespace node
}  // namespace dai
