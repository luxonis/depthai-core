#include "depthai/pipeline/node/PointCloud.hpp"

#include <chrono>
#include <future>
#include <thread>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "common/Point3f.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "device/CalibrationHandler.hpp"

#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "depthai/shaders/depth2pointcloud.hpp"
    #include "kompute/Kompute.hpp"
#endif
#include "utility/PimplImpl.hpp"

namespace dai {
namespace node {

// PointCloud::Impl - Same structure as RGBD::Impl
class PointCloud::Impl {
   public:
    Impl() = default;
    
    // Add method to set logger
    void setLogger(std::shared_ptr<spdlog::logger> log) {
        logger = log;
    }
    
    void computePointCloud(const uint8_t* depthData, std::vector<Point3f>& points) {
        if(!intrinsicsSet) {
            throw std::runtime_error("Intrinsics not set");
        }
        points.reserve(size);
        switch(computeMethod) {
            case ComputeMethod::CPU:
                computePointCloudCPU(depthData, points);
                break;
            case ComputeMethod::CPU_MT:
                computePointCloudCPUMT(depthData, points);
                break;
            case ComputeMethod::GPU:
                computePointCloudGPU(depthData, points);
                break;
        }
        
        // Apply extrinsic transformation if set
        if(hasExtrinsics) {
            transformPoints(points);
        }
    }
    
    void setDepthUnit(dai::DepthUnit depthUnit) {
        // Get multiplier from DepthUnit.hpp (converts from METER to target unit)
        depthUnitMultiplier = getDepthUnitMultiplier(depthUnit);
        
        // Calibration depth values are in MILLIMETERS
        // Calibration translation values are in CENTIMETERS
        
        // Calculate conversion factors using getDepthUnitMultiplier
        constexpr float MM_MULTIPLIER = getDepthUnitMultiplier(DepthUnit::MILLIMETER);  // 1000.0
        constexpr float CM_MULTIPLIER = getDepthUnitMultiplier(DepthUnit::CENTIMETER);  // 100.0
        
        // Convert from MM to target unit:
        // MM -> target unit = (target_multiplier / MM_multiplier)
        scaleFactor = depthUnitMultiplier / MM_MULTIPLIER;
        
        // Convert from CM to target unit:
        // CM -> target unit = (target_multiplier / CM_multiplier)
        translationScaleFactor = depthUnitMultiplier / CM_MULTIPLIER;
        
        if(logger) {
            logger->debug("Set depth unit: multiplier={}, scaleFactor={}, translationScaleFactor={}", 
                        depthUnitMultiplier, scaleFactor, translationScaleFactor);
        }
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
    
    void setExtrinsics(const std::vector<std::vector<float>>& transformMatrix) {
        if(transformMatrix.size() != 4 || transformMatrix[0].size() != 4) {
            throw std::runtime_error("Transformation matrix must be 4x4");
        }
        extrinsics = transformMatrix;
        hasExtrinsics = true;
    }

   private:
    void initializeGPU(uint32_t device) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        mgr = std::make_shared<kp::Manager>(device);
        shader = std::vector<uint32_t>(shaders::DEPTH2POINTCLOUD_COMP_SPV.begin(), shaders::DEPTH2POINTCLOUD_COMP_SPV.end());
        computeMethod = ComputeMethod::GPU;
#else
        (void)device;
        throw std::runtime_error("Kompute not enabled in this build");
#endif
    }
    
    void transformPoints(std::vector<Point3f>& points) {
        for(auto& p : points) {
            // Apply 4x4 transformation: [X', Y', Z', 1]^T = T * [X, Y, Z, 1]^T
            // Points are in target units (from scaleFactor)
            // Calibration translations are in CM, so use translationScaleFactor to convert to target units
            p.x = extrinsics[0][0] * p.x + extrinsics[0][1] * p.y + extrinsics[0][2] * p.z + extrinsics[0][3] * translationScaleFactor;
            p.y = extrinsics[1][0] * p.x + extrinsics[1][1] * p.y + extrinsics[1][2] * p.z + extrinsics[1][3] * translationScaleFactor;
            p.z = extrinsics[2][0] * p.x + extrinsics[2][1] * p.y + extrinsics[2][2] * p.z + extrinsics[2][3] * translationScaleFactor;
        }
    }
    
    void computePointCloudGPU(const uint8_t* depthData, std::vector<Point3f>& points) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
        std::vector<float> xyzOut;
        xyzOut.resize(size * 3);

        float scale = scaleFactor;

        std::vector<float> depthDataFloat(size);
        for(size_t i = 0; i < size; i++) {
            uint16_t depthValue = *(reinterpret_cast<const uint16_t*>(depthData + i * 2));
            depthDataFloat[i] = static_cast<float>(depthValue);
        }

        std::vector<float> intrinsics = {fx, fy, cx, cy, scale, static_cast<float>(width), static_cast<float>(height)};

        if(!tensorsInitialized) {
            depthTensor = mgr->tensor(depthDataFloat);
            intrinsicsTensor = mgr->tensor(intrinsics);
            xyzTensor = mgr->tensor(xyzOut);
            tensorsInitialized = true;
        } else {
            depthTensor->setData(depthDataFloat);
        }
        
        if(!algoInitialized) {
            tensors.emplace_back(depthTensor);
            tensors.emplace_back(intrinsicsTensor);
            tensors.emplace_back(xyzTensor);
            algo = mgr->algorithm(tensors, shader);
            algoInitialized = true;
        }
        
        mgr->sequence()->record<kp::OpSyncDevice>(tensors)->record<kp::OpAlgoDispatch>(algo)->record<kp::OpSyncLocal>(tensors)->eval();
        
        xyzOut = xyzTensor->vector<float>();
        for(size_t i = 0; i < size; i++) {
            Point3f p;
            p.x = xyzOut[i * 3 + 0];
            p.y = xyzOut[i * 3 + 1];
            p.z = xyzOut[i * 3 + 2];
            if(p.z > 0.0f) {  // Only add valid points
                points.emplace_back(p);
            }
        }
#else
        (void)depthData;
        (void)points;
        throw std::runtime_error("Kompute not enabled in this build");
#endif
    }
    
    void calcPointsChunk(const uint8_t* depthData, std::vector<Point3f>& outChunk, unsigned int startRow, unsigned int endRow) {
        float scale = scaleFactor;
        outChunk.reserve((endRow - startRow) * width);

        for(unsigned int row = startRow; row < endRow; row++) {
            unsigned int rowStart = row * width;
            for(unsigned int col = 0; col < width; col++) {
                size_t i = rowStart + col;

                uint16_t depthValue = *(reinterpret_cast<const uint16_t*>(depthData + i * 2));
                float z = static_cast<float>(depthValue) * scale;

                // Skip invalid depth values
                if(z <= 0.0f) {
                    continue;
                }

                // Back-project pixel to 3D using camera intrinsics
                float xCoord = (col - cx) * z / fx;
                float yCoord = (row - cy) * z / fy;

                outChunk.push_back(Point3f{xCoord, yCoord, z});
            }
        }
    }

    void computePointCloudCPU(const uint8_t* depthData, std::vector<Point3f>& points) {
        calcPointsChunk(depthData, points, 0, height);
    }

    void computePointCloudCPUMT(const uint8_t* depthData, std::vector<Point3f>& points) {
        unsigned int rowsPerThread = height / threadNum;
        std::vector<std::future<std::vector<Point3f>>> futures;

        auto processRows = [&](unsigned int startRow, unsigned int endRow) {
            std::vector<Point3f> localPoints;
            calcPointsChunk(depthData, localPoints, startRow, endRow);
            return localPoints;
        };

        for(uint32_t t = 0; t < threadNum; ++t) {
            unsigned int startRow = t * rowsPerThread;
            unsigned int endRow = (t == threadNum - 1) ? height : (startRow + rowsPerThread);
            futures.emplace_back(std::async(std::launch::async, processRows, startRow, endRow));
        }

        for(auto& f : futures) {
            auto localPoints = f.get();
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
    
    // Default to MILLIMETER unit
    static constexpr float DEFAULT_DEPTH_UNIT_MULTIPLIER = getDepthUnitMultiplier(DepthUnit::MILLIMETER);  // 1000.0
    static constexpr float MM_MULTIPLIER = getDepthUnitMultiplier(DepthUnit::MILLIMETER);  // 1000.0
    static constexpr float CM_MULTIPLIER = getDepthUnitMultiplier(DepthUnit::CENTIMETER);  // 100.0
    
    float scaleFactor = DEFAULT_DEPTH_UNIT_MULTIPLIER / MM_MULTIPLIER;  // 1.0 (MM->MM)
    float translationScaleFactor = DEFAULT_DEPTH_UNIT_MULTIPLIER / CM_MULTIPLIER;  // 10.0 (CM->MM)
    float depthUnitMultiplier = DEFAULT_DEPTH_UNIT_MULTIPLIER;  // 1000.0
    
    float fx, fy, cx, cy;
    unsigned int width, height;
    size_t size;
    bool intrinsicsSet = false;
    uint32_t threadNum = 2;
    
    // Extrinsic transformation
    std::vector<std::vector<float>> extrinsics;
    bool hasExtrinsics = false;
    
    // Logger
    std::shared_ptr<spdlog::logger> logger;
};

PointCloud::PointCloud() = default;
PointCloud::~PointCloud() = default;

PointCloud::Properties& PointCloud::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void PointCloud::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void PointCloud::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool PointCloud::runOnHost() const {
    return runOnHostVar;
}

void PointCloud::setDepthUnit(dai::DepthUnit depthUnit) {
    pimplPointCloud->setDepthUnit(depthUnit);
}

void PointCloud::useCPU() {
    pimplPointCloud->useCPU();
}

void PointCloud::useCPUMT(uint32_t numThreads) {
    pimplPointCloud->useCPUMT(numThreads);
}

void PointCloud::useGPU(uint32_t device) {
    pimplPointCloud->useGPU(device);
}

void PointCloud::setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpecTranslation) {
    coordSystemType = CoordinateSystemType::CAMERA_SOCKET;
    targetCameraSocket = targetCamera;
    this->useSpecTranslation = useSpecTranslation;
}

void PointCloud::setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpecTranslation) {
    coordSystemType = CoordinateSystemType::HOUSING;
    targetHousingCS = housingCS;
    this->useSpecTranslation = useSpecTranslation;
}

void PointCloud::initialize(std::shared_ptr<ImgFrame> depthFrame) {
    pimpl->logger->debug("PointCloud::initialize() called");
    pimplPointCloud->setLogger(pimpl->logger);
    
    auto width = depthFrame->getWidth();
    auto height = depthFrame->getHeight();
    
    // Get camera intrinsics from the depth frame
    auto intrinsics = depthFrame->transformation.getIntrinsicMatrix();
    float fx = intrinsics[0][0];
    float fy = intrinsics[1][1];
    float cx = intrinsics[0][2];
    float cy = intrinsics[1][2];
    
    pimpl->logger->debug("Setting intrinsics: fx={}, fy={}, cx={}, cy={}, size={}x{}", 
                        fx, fy, cx, cy, width, height);
    
    pimplPointCloud->setIntrinsics(fx, fy, cx, cy, width, height);
    
    // Get source camera socket from depth frame
    auto srcCamera = static_cast<CameraBoardSocket>(depthFrame->getInstanceNum());
    
    pimpl->logger->debug("Source camera: {}, Coord system type: {}", 
                        toString(srcCamera), (int)coordSystemType);
    
    // Get calibration handler and setup extrinsics if needed
    if(coordSystemType != CoordinateSystemType::NONE) {        
        try {
            auto calibHandler = device->getCalibration();
            std::vector<std::vector<float>> transformMatrix;
            
            switch(coordSystemType) {
                case CoordinateSystemType::CAMERA_SOCKET:
                    pimpl->logger->info("Using CAMERA_SOCKET transformation from {} to {}", 
                                       toString(srcCamera), toString(targetCameraSocket));
                    transformMatrix = calibHandler.getCameraExtrinsics(srcCamera, targetCameraSocket, useSpecTranslation);
                    break;
                    
                case CoordinateSystemType::HOUSING:
                    pimpl->logger->info("Using HOUSING transformation from {} to housing {}", 
                                       toString(srcCamera), static_cast<int>(targetHousingCS));
                    transformMatrix = calibHandler.getHousingCalibration(srcCamera, targetHousingCS, useSpecTranslation);
                    break;
                    
                case CoordinateSystemType::NONE:
                    // This shouldn't happen due to outer if, but handle it for completeness
                    pimpl->logger->warn("CoordinateSystemType::NONE in transformation block - this shouldn't happen");
                    break;
            }
            
            // Log the transformation matrix
            if(pimpl->logger->level() <= spdlog::level::debug) {
                pimpl->logger->debug("Transformation matrix (4x4):");
                for(size_t i = 0; i < 4; i++) {
                    pimpl->logger->debug("  [{:8.4f}, {:8.4f}, {:8.4f}, {:8.4f}]", 
                                        transformMatrix[i][0], transformMatrix[i][1], 
                                        transformMatrix[i][2], transformMatrix[i][3]);
                }
            }
            
            pimplPointCloud->setExtrinsics(transformMatrix);
            
        } catch(const std::exception& e) {
            pimpl->logger->error("Failed to get extrinsics: {}", e.what());
            throw;
        }
    } else {
        pimpl->logger->info("PointCloud: No coordinate system transformation applied (identity)");
    }
    
    pimpl->logger->debug("PointCloud initialized with intrinsics: fx={}, fy={}, cx={}, cy={}, size={}x{}", 
                        fx, fy, cx, cy, width, height);
    
    initialized = true;
    
    pimpl->logger->info("PointCloud::initialize() completed");
}

void PointCloud::run() {
    pimpl->logger->info("PointCloud node started");

    while(mainLoop()) {
        if(!outputPointCloud.getQueueConnections().empty() || !outputPointCloud.getConnections().empty()) {
            
            std::shared_ptr<ImgFrame> depthFrame = nullptr;
            
            {
                auto blockEvent = this->inputBlockEvent();
                depthFrame = inputDepth.tryGet<ImgFrame>();
            }
            
            if(!depthFrame) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            // Initialize on first frame
            if(!initialized) {
                initialize(depthFrame);
            }
            
            auto cameraInstanceNum = depthFrame->getInstanceNum();
            
            
            // Create point cloud output
            auto pc = std::make_shared<PointCloudData>();
            pc->setTimestamp(depthFrame->getTimestamp());
            pc->setTimestampDevice(depthFrame->getTimestampDevice());
            pc->setSequenceNum(depthFrame->getSequenceNum());
            pc->setInstanceNum(cameraInstanceNum);
            
            auto width = depthFrame->getWidth();
            auto height = depthFrame->getHeight();
            pc->setSize(width, height);
            
            // Compute point cloud from depth using intrinsics
            std::vector<Point3f> points;
            auto* depthData = depthFrame->getData().data();
            pimplPointCloud->computePointCloud(depthData, points);
            
            // Calculate bounding box
            float minX = 0.0f, minY = 0.0f, minZ = 0.0f;
            float maxX = 0.0f, maxY = 0.0f, maxZ = 0.0f;
            
            if(!points.empty()) {
                minX = maxX = points[0].x;
                minY = maxY = points[0].y;
                minZ = maxZ = points[0].z;
                
                for(const auto& p : points) {
                    minX = std::min(minX, p.x);
                    minY = std::min(minY, p.y);
                    minZ = std::min(minZ, p.z);
                    maxX = std::max(maxX, p.x);
                    maxY = std::max(maxY, p.y);
                    maxZ = std::max(maxZ, p.z);
                }
            }
            
            pc->setMinX(minX);
            pc->setMinY(minY);
            pc->setMinZ(minZ);
            pc->setMaxX(maxX);
            pc->setMaxY(maxY);
            pc->setMaxZ(maxZ);
            
            pc->setPoints(points);
            pc->setSparse(true);
            
            {
                auto blockEvent = this->outputBlockEvent();
                outputPointCloud.send(pc);
                
                if(!passthroughDepth.getQueueConnections().empty() || !passthroughDepth.getConnections().empty()) {
                    passthroughDepth.send(depthFrame);
                }
            }
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    pimpl->logger->info("PointCloud node stopped");
}

}  // namespace node

// Explicit template instantiation must be in dai namespace, not dai::node
// This is placed OUTSIDE the dai::node namespace but inside dai namespace
template class Pimpl<node::PointCloud::Impl>;

}  // namespace dai
