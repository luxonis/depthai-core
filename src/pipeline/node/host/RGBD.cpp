#include "depthai/pipeline/node/host/RGBD.hpp"

#include <future>

#include "common/Point3fRGB.hpp"
#include "depthai/common/Point3fRGB.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "depthai/shaders/rgbd2pointcloud.hpp"
    #include "kompute/Kompute.hpp"
#endif
#include "utility/PimplImpl.hpp"

namespace dai {
namespace node {

class RGBD::Impl {
   public:
    Impl() = default;
    void computePointCloud(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& points) {
        if(!intrinsicsSet) {
            throw std::runtime_error("Intrinsics not set");
        }
        points.reserve(size);
        switch(computeMethod) {
            case ComputeMethod::CPU:
                computePointCloudCPU(depthData, colorData, points);
                break;
            case ComputeMethod::CPU_MT:
                computePointCloudCPUMT(depthData, colorData, points);
                break;
            case ComputeMethod::GPU:
                computePointCloudGPU(depthData, colorData, points);
                break;
        }
    }
    void setOutputMeters(bool outputMeters) {
        this->outputMeters = outputMeters;
    }
    void printDevices() {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        auto devices = mgr->listDevices();
        for(auto& device : devices) {
            std::cout << "Device: " << device.getProperties().deviceName << std::endl;
        }
#endif
    }
    void useCPU() {
        computeMethod = ComputeMethod::CPU;
    }
    void useCPUMT(uint32_t numThreads) {
        threadNum = numThreads;
        computeMethod = ComputeMethod::CPU_MT;
    }
    void useGPU(uint32_t device) {
        initializeGPU(device);
    }
    void setIntrinsics(float fx, float fy, float cx, float cy, unsigned int width, unsigned int height) {
        this->fx = fx;
        this->fy = fy;
        this->cx = cx;
        this->cy = cy;
        this->width = width;
        this->height = height;
        size = this->width * this->height;
        intrinsicsSet = true;
    }

   private:
    void initializeGPU(uint32_t device) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        // Initialize Kompute
        mgr = std::make_shared<kp::Manager>(device);
        shader = std::vector<uint32_t>(shaders::RGBD2POINTCLOUD_COMP_SPV.begin(), shaders::RGBD2POINTCLOUD_COMP_SPV.end());
        computeMethod = ComputeMethod::GPU;
#else
        throw std::runtime_error("Kompute not enabled in this build");
#endif
    }
    void computePointCloudGPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& points) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        std::vector<float> xyzOut;

        xyzOut.resize(size * 3);

        // Convert depth to float
        // Depth is in mm by default, convert to meters if outputMeters == true
        // If outputMeters is true, scale = 1/1000.0 else scale = 1.0
        float scale = outputMeters ? (1.0f / 1000.0f) : 1.0f;

        std::vector<float> depthDataFloat(size);
        for(int i = 0; i < size; i++) {
            uint16_t depthValue = *(reinterpret_cast<const uint16_t*>(depthData + i * 2));
            depthDataFloat[i] = static_cast<float>(depthValue);  // will multiply by scale in shader
        }

        // Intrinsics: [fx, fy, cx, cy, scale, width, height]
        std::vector<float> intrinsics = {fx, fy, cx, cy, scale, static_cast<float>(width), static_cast<float>(height)};

        // Create Kompute tensors
        if(!tensorsInitialized) {
            depthTensor = mgr->tensor(depthDataFloat);
            intrinsicsTensor = mgr->tensor(intrinsics);
            xyzTensor = mgr->tensor(xyzOut);
            tensorsInitialized = true;
        } else {
            depthTensor->setData(depthDataFloat);
        }
        // Load shader
        if(!algoInitialized) {
            tensors.emplace_back(depthTensor);
            tensors.emplace_back(intrinsicsTensor);
            tensors.emplace_back(xyzTensor);
            algo = mgr->algorithm(tensors, shader);
            algoInitialized = true;
        }
        mgr->sequence()->record<kp::OpSyncDevice>(tensors)->record<kp::OpAlgoDispatch>(algo)->record<kp::OpSyncLocal>(tensors)->eval();
        // Retrieve results
        xyzOut = xyzTensor->vector<float>();
        for(int i = 0; i < size; i++) {
            Point3fRGB p;
            p.x = xyzOut[i * 3 + 0];
            p.y = xyzOut[i * 3 + 1];
            p.z = xyzOut[i * 3 + 2];
            p.r = colorData[i * 3 + 0];
            p.g = colorData[i * 3 + 1];
            p.b = colorData[i * 3 + 2];
            points.emplace_back(p);
        }
#endif
    }
    void calcPointsChunk(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& outChunk, int startRow, int endRow) {
        float scale = outputMeters ? (1.0f / 1000.0f) : 1.0f;
        outChunk.reserve((endRow - startRow) * width);

        for(int row = startRow; row < endRow; row++) {
            int rowStart = row * width;
            for(int col = 0; col < width; col++) {
                size_t i = rowStart + col;

                uint16_t depthValue = *(reinterpret_cast<const uint16_t*>(depthData + i * 2));
                float z = static_cast<float>(depthValue) * scale;

                float xCoord = (col - cx) * z / fx;
                float yCoord = (row - cy) * z / fy;

                // BGR order in colorData
                uint8_t r = colorData[i * 3 + 0];
                uint8_t g = colorData[i * 3 + 1];
                uint8_t b = colorData[i * 3 + 2];

                outChunk.push_back(Point3fRGB{xCoord, yCoord, z, r, g, b});
            }
        }
    }

    void computePointCloudCPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& points) {
        // Single-threaded directly writes into points
        calcPointsChunk(depthData, colorData, points, 0, height);
    }

    void computePointCloudCPUMT(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& points) {
        int rowsPerThread = height / threadNum;
        std::vector<std::future<std::vector<Point3fRGB>>> futures;

        // Each thread returns a local vector
        auto processRows = [&](int startRow, int endRow) {
            std::vector<Point3fRGB> localPoints;
            calcPointsChunk(depthData, colorData, localPoints, startRow, endRow);
            return localPoints;
        };

        for(int t = 0; t < threadNum; ++t) {
            int startRow = t * rowsPerThread;
            int endRow = (t == threadNum - 1) ? height : (startRow + rowsPerThread);
            futures.emplace_back(std::async(std::launch::async, processRows, startRow, endRow));
        }

        // Merge all results
        for(auto& f : futures) {
            auto localPoints = f.get();
            // Now we do one lock per merge if needed
            // If this is not called concurrently from multiple places, no lock needed
            points.insert(points.end(), localPoints.begin(), localPoints.end());
        }
    }
    enum class ComputeMethod { CPU, CPU_MT, GPU };
    ComputeMethod computeMethod = ComputeMethod::CPU;
#ifdef DEPTHAI_ENABLE_KOMPUTE
    std::shared_ptr<kp::Manager> mgr;
    std::vector<uint32_t> shader;
    std::shared_ptr<kp::Algorithm> algo;
    std::shared_ptr<kp::Tensor> depthTensor;
    std::shared_ptr<kp::Tensor> intrinsicsTensor;
    std::shared_ptr<kp::Tensor> xyzTensor;
    std::vector<std::shared_ptr<kp::Memory>> tensors;
    bool algoInitialized = false;
    bool tensorsInitialized = false;
#endif
    bool outputMeters = false;
    float fx, fy, cx, cy;
    int width, height;
    int size;
    bool intrinsicsSet = false;
    int threadNum = 2;
};

RGBD::RGBD() = default;

RGBD::~RGBD() = default;

std::shared_ptr<RGBD> RGBD::build() {
    sync->out.link(inSync);
    sync->setRunOnHost(false);
    inColor.setBlocking(false);
    inColor.setMaxSize(1);
    inDepth.setBlocking(false);
    inDepth.setMaxSize(1);
    inSync.setBlocking(false);
    inSync.setMaxSize(1);
    return std::static_pointer_cast<RGBD>(shared_from_this());
}

std::shared_ptr<RGBD> RGBD::build(bool autocreate, std::pair<int, int> size) {
    if(!autocreate) {
        return std::static_pointer_cast<RGBD>(shared_from_this());
    }
    auto pipeline = getParentPipeline();
    auto colorCam = pipeline.create<node::Camera>()->build();
    auto depth = pipeline.create<node::StereoDepth>()->build(true);
    auto* out = colorCam->requestOutput(size, dai::ImgFrame::Type::RGB888i);
    out->link(inColor);
    out->link(depth->inputAlignTo);
    depth->depth.link(inDepth);
    return build();
}

void RGBD::initialize(std::shared_ptr<MessageGroup> frames) {
    // Initialize the camera intrinsics
    // Check if width and height match
    auto colorFrame = std::dynamic_pointer_cast<ImgFrame>(frames->group.at(inColor.getName()));
    if(colorFrame->getType() != dai::ImgFrame::Type::RGB888i) {
        throw std::runtime_error("RGBD node only supports RGB888i frames");
    }
    auto depthFrame = std::dynamic_pointer_cast<ImgFrame>(frames->group.at(inDepth.getName()));
    if(colorFrame->getWidth() != depthFrame->getWidth() || colorFrame->getHeight() != depthFrame->getHeight()) {
        throw std::runtime_error("Color and depth frame sizes do not match");
    }
    auto calibHandler = getParentPipeline().getDefaultDevice()->readCalibration();
    auto camID = static_cast<CameraBoardSocket>(colorFrame->getInstanceNum());
    auto width = colorFrame->getWidth();
    auto height = colorFrame->getHeight();
    auto intrinsics = calibHandler.getCameraIntrinsics(camID, width, height);
    float fx = intrinsics[0][0];
    float fy = intrinsics[1][1];
    float cx = intrinsics[0][2];
    float cy = intrinsics[1][2];
    pimpl->setIntrinsics(fx, fy, cx, cy, width, height);
    initialized = true;
}

void RGBD::run() {
    while(isRunning()) {
        // Get the color and depth frames
        auto group = inSync.get<MessageGroup>();
        if(group == nullptr) continue;
        if(!initialized) {
            initialize(group);
        }
        auto colorFrame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(inColor.getName()));
        if(colorFrame->getType() != dai::ImgFrame::Type::RGB888i) {
            throw std::runtime_error("RGBD node only supports RGB888i frames");
        }
        auto depthFrame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(inDepth.getName()));

        // Create the point cloud
        auto pc = std::make_shared<PointCloudData>();
        pc->setTimestamp(colorFrame->getTimestamp());
        pc->setTimestampDevice(colorFrame->getTimestampDevice());
        pc->setSequenceNum(colorFrame->getSequenceNum());
        pc->setInstanceNum(colorFrame->getInstanceNum());
        auto width = colorFrame->getWidth();
        auto height = colorFrame->getHeight();
        pc->setSize(width, height);

        std::vector<Point3fRGB> points;
        // Fill the point cloud
        auto* depthData = depthFrame->getData().data();
        auto* colorData = colorFrame->getData().data();
        // Use GPU to compute point cloud
        pimpl->computePointCloud(depthData, colorData, points);

        pc->setPointsRGB(points);
        pcl.send(pc);
    }
}
void RGBD::setOutputMeters(bool outputMeters) {
    pimpl->setOutputMeters(outputMeters);
}
void RGBD::useCPU() {
    pimpl->useCPU();
}
void RGBD::useCPUMT(uint32_t numThreads) {
    pimpl->useCPUMT(numThreads);
}
void RGBD::useGPU(uint32_t device) {
    pimpl->useGPU(device);
}
void RGBD::printDevices() {
    pimpl->printDevices();
}

}  // namespace node
}  // namespace dai
