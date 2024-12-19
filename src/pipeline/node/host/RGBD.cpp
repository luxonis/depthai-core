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
    void setGPUDevice(uint32_t deviceIndex) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        this->deviceIndex = deviceIndex;
#endif
    }
    void setCPUThreadNum(uint32_t numThreads) {
        threadNum = numThreads;
    }
    void useCPU() {
        computeMethod = ComputeMethod::CPU;
    }
    void useCPUMT() {
        computeMethod = ComputeMethod::CPU_MT;
    }
    void useGPU() {
        initializeGPU();
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
    void initializeGPU() {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        // Initialize Kompute
        mgr = std::make_shared<kp::Manager>(deviceIndex);
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
    void calcPoints(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& points, int startRow, int endRow) {
        float scale = outputMeters ? (1.0f / 1000.0f) : 1.0f;
        for(int i = startRow * width; i < endRow * width; i++) {
            float x = i % width;
            float y = i / width;
            uint16_t depthValue = *(reinterpret_cast<const uint16_t*>(depthData + i * 2));
            float z = static_cast<float>(depthValue) * scale;
            x = (x - cx) * z / fx;
            y = (y - cy) * z / fy;
            uint8_t r = static_cast<uint8_t>(colorData[i * 3 + 0]);
            uint8_t g = static_cast<uint8_t>(colorData[i * 3 + 1]);
            uint8_t b = static_cast<uint8_t>(colorData[i * 3 + 2]);
            Point3fRGB p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.r = r;
            p.g = g;
            p.b = b;
            std::lock_guard<std::mutex> lock(pointsMtx);
            points.emplace_back(p);
        }
    }
    void computePointCloudCPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& points) {
        calcPoints(depthData, colorData, points, 0, height);
    }
    void computePointCloudCPUMT(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGB>& points) {
        // Lambda function for processing a block of rows
        auto processRows = [&](int startRow, int endRow) { calcPoints(depthData, colorData, points, startRow, endRow); };

        // Divide rows into chunks and process in parallel
        const int numThreads = threadNum;
        const int rowsPerThread = height / numThreads;
        std::vector<std::future<void>> futures;

        for(int t = 0; t < numThreads; ++t) {
            int startRow = t * rowsPerThread;
            int endRow = (t == numThreads - 1) ? height : startRow + rowsPerThread;

            futures.emplace_back(std::async(std::launch::async, processRows, startRow, endRow));
        }

        // Wait for all threads to finish
        for(auto& future : futures) {
            future.get();
        }
    }
    enum class ComputeMethod { CPU, CPU_MT, GPU };
    ComputeMethod computeMethod = ComputeMethod::CPU;
#ifdef DEPTHAI_ENABLE_KOMPUTE
    std::shared_ptr<kp::Manager> mgr;
    uint32_t deviceIndex = 0;
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
    std::mutex pointsMtx;
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
    depth->depth.link(inDepth);
    return build();
}

void RGBD::initialize(std::vector<std::shared_ptr<ImgFrame>> frames) {
    // Initialize the camera intrinsics
    // Check if width and height match
    if(frames.at(0)->getWidth() != frames.at(1)->getWidth() || frames.at(0)->getHeight() != frames.at(1)->getHeight()) {
        throw std::runtime_error("Color and depth frame sizes do not match");
    }
    auto frame = frames.at(1);
    auto calibHandler = getParentPipeline().getDefaultDevice()->readCalibration();
    auto camID = static_cast<CameraBoardSocket>(frame->getInstanceNum());
    auto width = frame->getWidth();
    auto height = frame->getHeight();
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
void RGBD::useCPUMT() {
    pimpl->useCPUMT();
}
void RGBD::useGPU() {
    pimpl->useGPU();
}
void RGBD::setGPUDevice(uint32_t deviceIndex) {
    pimpl->setGPUDevice(deviceIndex);
}
void RGBD::setCPUThreadNum(uint32_t numThreads) {
    pimpl->setCPUThreadNum(numThreads);
}
void RGBD::printDevices() {
    pimpl->printDevices();
}

}  // namespace node
}  // namespace dai
