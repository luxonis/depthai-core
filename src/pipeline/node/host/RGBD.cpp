#include "depthai/pipeline/node/host/RGBD.hpp"

#include <future>

#include "common/Point3fRGBA.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
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
    void computePointCloud(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points) {
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
    void setDepthUnit(StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit) {
        // Default is millimeter
        switch(depthUnit) {
            case StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER:
                scaleFactor = 1.0f;
                break;
            case StereoDepthConfig::AlgorithmControl::DepthUnit::METER:
                scaleFactor = 0.001f;
                break;
            case StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER:
                scaleFactor = 0.01f;
                break;
            case StereoDepthConfig::AlgorithmControl::DepthUnit::FOOT:
                scaleFactor = 0.3048f;
                break;
            case StereoDepthConfig::AlgorithmControl::DepthUnit::INCH:
                scaleFactor = 0.0254f;
                break;
            case StereoDepthConfig::AlgorithmControl::DepthUnit::CUSTOM:
                scaleFactor = 1.0f;
                break;
        }
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
        (void)device;
        throw std::runtime_error("Kompute not enabled in this build");
#endif
    }
    void computePointCloudGPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        std::vector<float> xyzOut;

        xyzOut.resize(size * 3);

        // Convert depth to float
        // Depth is in mm by default, convert to meters if outputMeters == true
        // If outputMeters is true, scale = 1/1000.0 else scale = 1.0
        float scale = scaleFactor;

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
            Point3fRGBA p;
            p.x = xyzOut[i * 3 + 0];
            p.y = xyzOut[i * 3 + 1];
            p.z = xyzOut[i * 3 + 2];
            p.r = colorData[i * 3 + 0];
            p.g = colorData[i * 3 + 1];
            p.b = colorData[i * 3 + 2];
            points.emplace_back(p);
        }
#else
        (void)depthData;
        (void)colorData;
        (void)points;
        throw std::runtime_error("Kompute not enabled in this build");
#endif
    }
    void calcPointsChunk(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& outChunk, int startRow, int endRow) {
        float scale = scaleFactor;
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

                outChunk.push_back(Point3fRGBA{xCoord, yCoord, z, r, g, b});
            }
        }
    }

    void computePointCloudCPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points) {
        // Single-threaded directly writes into points
        calcPointsChunk(depthData, colorData, points, 0, height);
    }

    void computePointCloudCPUMT(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points) {
        int rowsPerThread = height / threadNum;
        std::vector<std::future<std::vector<Point3fRGBA>>> futures;

        // Each thread returns a local vector
        auto processRows = [&](int startRow, int endRow) {
            std::vector<Point3fRGBA> localPoints;
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
    float scaleFactor = 1.0f;
    float fx, fy, cx, cy;
    int width, height;
    int size;
    bool intrinsicsSet = false;
    int threadNum = 2;
};

RGBD::RGBD() = default;

RGBD::~RGBD() = default;

void RGBD::buildInternal() {
    sync->out.link(inSync);
    sync->setRunOnHost(false);
    inColor.setBlocking(false);
    inColor.setMaxSize(1);
    inDepth.setBlocking(false);
    inDepth.setMaxSize(1);
    inSync.setBlocking(false);
    inSync.setMaxSize(1);
}

std::shared_ptr<RGBD> RGBD::build() {
    return std::static_pointer_cast<RGBD>(shared_from_this());
}
std::shared_ptr<RGBD> RGBD::build(bool autocreate, StereoDepth::PresetMode mode, std::pair<int, int> size) {
    if(!autocreate) {
        return std::static_pointer_cast<RGBD>(shared_from_this());
    }
    auto pipeline = getParentPipeline();
    auto colorCam = pipeline.create<node::Camera>()->build();
    auto platform = pipeline.getDefaultDevice()->getPlatform();
    auto stereo = pipeline.create<node::StereoDepth>()->build(true, mode, size);
    std::shared_ptr<node::ImageAlign> align = nullptr;
    if(platform == Platform::RVC4) {
        align = pipeline.create<node::ImageAlign>();
    }
    if(platform == Platform::RVC4) {
        auto* out = colorCam->requestOutput(size, ImgFrame::Type::RGB888i);
        out->link(inColor);
        stereo->depth.link(align->input);
        out->link(align->inputAlignTo);
        align->outputAligned.link(inDepth);
    } else {
        auto* out = colorCam->requestOutput(size, ImgFrame::Type::RGB888i, ImgResizeMode::CROP, 30, true);
        out->link(inColor);
        out->link(stereo->inputAlignTo);
        stereo->depth.link(inDepth);
    }
    return build();
}

void RGBD::initialize(std::shared_ptr<MessageGroup> frames) {
    // Initialize the camera intrinsics
    // Check if width, width and cameraID match
    auto colorFrame = std::dynamic_pointer_cast<ImgFrame>(frames->group.at(inColor.getName()));
    if(colorFrame->getType() != ImgFrame::Type::RGB888i) {
        throw std::runtime_error("RGBD node only supports RGB888i frames");
    }
    auto depthFrame = std::dynamic_pointer_cast<ImgFrame>(frames->group.at(inDepth.getName()));
    if(colorFrame->getWidth() != depthFrame->getWidth() || colorFrame->getHeight() != depthFrame->getHeight()) {
        throw std::runtime_error("Color and depth frame sizes do not match");
    }
    if(colorFrame->getInstanceNum() != depthFrame->getInstanceNum()) {
        throw std::runtime_error("Depth is not aligned to color");
    }
    auto width = colorFrame->getWidth();
    auto height = colorFrame->getHeight();
    auto intrinsics = colorFrame->transformation.getIntrinsicMatrix();
    float fx = intrinsics[0][0];
    float fy = intrinsics[1][1];
    float cx = intrinsics[0][2];
    float cy = intrinsics[1][2];
    pimpl->setIntrinsics(fx, fy, cx, cy, width, height);
    initialized = true;
}

void RGBD::run() {
    while(isRunning()) {
        if(!pcl.getQueueConnections().empty() || !pcl.getConnections().empty() || !rgbd.getQueueConnections().empty() || !rgbd.getConnections().empty()) {
            // Get the color and depth frames
            auto group = inSync.get<MessageGroup>();
            if(group == nullptr) continue;
            if(!initialized) {
                initialize(group);
            }
            auto colorFrame = std::dynamic_pointer_cast<ImgFrame>(group->group.at(inColor.getName()));
            if(colorFrame->getType() != ImgFrame::Type::RGB888i) {
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

            std::vector<Point3fRGBA> points;
            // Fill the point cloud
            auto* depthData = depthFrame->getData().data();
            auto* colorData = colorFrame->getData().data();
            // Use GPU to compute point cloud
            pimpl->computePointCloud(depthData, colorData, points);

            float minX = 0.0;
            float minY = 0.0;
            float minZ = 0.0;
            float maxX = 0.0;
            float maxY = 0.0;
            float maxZ = 0.0;
            for(const auto& p : points) {
                minX = std::min(minX, p.x);
                minY = std::min(minY, p.y);
                minZ = std::min(minZ, p.z);
                maxX = std::max(maxX, p.x);
                maxY = std::max(maxY, p.y);
                maxZ = std::max(maxZ, p.z);
            }
            pc->setMinX(minX);
            pc->setMinY(minY);
            pc->setMinZ(minZ);
            pc->setMaxX(maxX);
            pc->setMaxY(maxY);
            pc->setMaxZ(maxZ);

            pc->setPointsRGB(points);
            pc->setTimestamp(colorFrame->getTimestamp());
            pc->setTimestampDevice(colorFrame->getTimestampDevice());
            pc->setSequenceNum(colorFrame->getSequenceNum());
            pc->setInstanceNum(colorFrame->getInstanceNum());
            if(!pcl.getQueueConnections().empty() || !pcl.getConnections().empty()) {
                pcl.send(pc);
            }
            if(!rgbd.getQueueConnections().empty() || !rgbd.getConnections().empty()) {
                auto rgbdData = std::make_shared<RGBDData>();
                rgbdData->setTimestamp(colorFrame->getTimestamp());
                rgbdData->setTimestampDevice(colorFrame->getTimestampDevice());
                rgbdData->setSequenceNum(colorFrame->getSequenceNum());
                rgbdData->setDepthFrame(depthFrame);
                rgbdData->setRGBFrame(colorFrame);
                rgbd.send(rgbdData);
            }
        }
    }
}
void RGBD::setDepthUnit(StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit) {
    pimpl->setDepthUnit(depthUnit);
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
