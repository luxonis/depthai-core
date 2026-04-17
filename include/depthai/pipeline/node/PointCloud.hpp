#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <depthai/properties/PointCloudProperties.hpp>
#include <memory>
#include <optional>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/HousingCoordinateSystem.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "kompute/Kompute.hpp"
#endif

namespace dai {
namespace node {

/**
 * @brief PointCloud node. Computes point cloud from depth frames.
 */
class PointCloud : public DeviceNodeCRTP<DeviceNode, PointCloud, PointCloudProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "PointCloud";

    class Impl {
       public:
        Impl() = default;

        void setLogger(std::shared_ptr<spdlog::logger> log);

        // Compute DENSE point cloud (width * height points, includes invalid z=0 or negative)
        void computePointCloudDense(const uint8_t* depthData, std::vector<Point3f>& points);

        // Compute DENSE colored point cloud from aligned depth+color (like RGBD node)
        void computePointCloudDenseColored(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points);

        // Apply extrinsic transformation to points
        template <typename PointT>
        void applyTransformation(std::vector<PointT>& points);

        // Filter dense points to sparse (only z > 0)
        template <typename PointT>
        std::vector<PointT> filterValidPoints(const std::vector<PointT>& densePoints);

        void setLengthUnit(dai::LengthUnit lengthUnit);
        void useCPU();
        void useCPUMT(uint32_t numThreads);
        void useGPU(uint32_t device);
        void setIntrinsics(float fx, float fy, float cx, float cy, unsigned int width, unsigned int height);
        void setExtrinsics(const std::vector<std::vector<float>>& transformMatrix);
        void clearExtrinsics();

        LengthUnit targetLengthUnit = LengthUnit::MILLIMETER;

       private:
        void initializeGPU(uint32_t device);
        template <typename PointT>
        void transformPointsCPU(std::vector<PointT>& points);
        void calcPointsChunkDense(const uint8_t* depthData, std::vector<Point3f>& points, unsigned int startRow, unsigned int endRow);
        void calcPointsChunkDenseColored(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points, unsigned int startRow, unsigned int endRow);
        void computePointCloudDenseCPU(const uint8_t* depthData, std::vector<Point3f>& points);
        void computePointCloudDenseCPUMT(const uint8_t* depthData, std::vector<Point3f>& points);
        void computePointCloudDenseGPU(const uint8_t* depthData, std::vector<Point3f>& points);
        void computePointCloudDenseColoredCPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points);
        void computePointCloudDenseColoredCPUMT(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points);

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

        static constexpr float DEFAULT_LENGTH_UNIT_MULTIPLIER = getLengthUnitMultiplier(LengthUnit::MILLIMETER);
        static constexpr float MM_MULTIPLIER = getLengthUnitMultiplier(LengthUnit::MILLIMETER);

        float scaleFactor = DEFAULT_LENGTH_UNIT_MULTIPLIER / MM_MULTIPLIER;  // = 1.0 (mm to mm by default)
        float lengthUnitMultiplier = DEFAULT_LENGTH_UNIT_MULTIPLIER;

        float fx = 0.0f, fy = 0.0f, cx = 0.0f, cy = 0.0f;
        unsigned int width = 0u, height = 0u;
        size_t size = 0;
        bool intrinsicsSet = false;
        uint32_t threadNum = 2;

        std::vector<std::vector<float>> extrinsics;
        bool hasExtrinsics = false;

        std::shared_ptr<spdlog::logger> logger;
    };

   protected:
    Properties& getProperties() override;
    using DeviceNodeCRTP::DeviceNodeCRTP;

   public:
    PointCloud();
    ~PointCloud();

    /**
     * Initial config to use when computing the point cloud.
     */
    std::shared_ptr<PointCloudConfig> initialConfig = std::make_shared<PointCloudConfig>();

    /**
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::PointCloudConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Sync subnode for synchronized depth + color input.
     * When only depth is connected, Sync passes through single-item MessageGroups.
     * When both depth and color are connected, Sync pairs them by timestamp.
     */
    Subnode<node::Sync> sync{*this, "sync"};

    static constexpr const char* depthInputName = "depth";
    static constexpr const char* colorInputName = "color";

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    InputMap& syncInputs = sync->inputs;

    /**
     * Input message with depth data used to create the point cloud.
     * Routed through the internal Sync subnode.
     */
    Input& inputDepth = syncInputs[depthInputName];

    /**
     * Get the optional color input for colorized point clouds.
     * Lazily creates the Sync entry so that depth-only mode works
     * without Sync waiting for a color frame that never arrives.
     *
     * Link an aligned color image (RGB888i, same dimensions as depth)
     * to this input to enable colored point cloud output.
     */
    Input& getColorInput();
#endif

    /**
     * Outputs PointCloudData message
     */
    Output outputPointCloud{*this, {"outputPointCloud", DEFAULT_GROUP, {{{DatatypeEnum::PointCloudData, false}}}}};

    /**
     * Passthrough depth from which the point cloud was calculated.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Specify whether to run on host or device
     * By default, the node will run on host.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Use single-threaded CPU for processing
     */
    void useCPU();

    /**
     * Use multi-threaded CPU for processing
     */
    void useCPUMT(uint32_t numThreads = 2);

    /**
     * Use GPU for point cloud computation
     * @param device GPU device index (default 0)
     */
    void useGPU(uint32_t device = 0);

    /**
     * Set target coordinate system to transform point cloud
     * @param targetCamera Target camera socket
     * @param useSpecTranslation Use spec translation instead of calibration (default: false)
     */
    void setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpecTranslation = false);

    /**
     * Set target coordinate system to housing coordinate system
     * Point cloud will be transformed to this housing coordinate system
     * @param housingCS Target housing coordinate system
     * @param useSpecTranslation Whether to use spec translation (default: true)
     */
    void setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpecTranslation = true);

    bool runOnHost() const override;

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    void buildInternal() override;
#endif

   private:
    Pimpl<Impl> pimplPointCloud;

    /// Private input receiving synced MessageGroup from Sync subnode
    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};

    void run() override;
    void initialize(const ImgFrame& depthFrame, const PointCloudConfig& config);
    bool hasTransformationChanged(const ImgFrame& frame);

    // Helper methods for initialize()
    void setIntrinsicsFromFrame(const ImgFrame& frame);
    void setCoordinateTransformation(const ImgFrame& depthFrame, const PointCloudConfig& config);

    // Processing methods for the two code paths
    void processDepthOnly(std::shared_ptr<ImgFrame> depthFrame, std::shared_ptr<PointCloudData> pc, bool organized);
    void processColorized(std::shared_ptr<ImgFrame> depthFrame, std::shared_ptr<ImgFrame> colorFrame, std::shared_ptr<PointCloudData> pc, bool organized);

    bool runOnHostVar = true;
    bool initialized = false;
    bool colorMode = false;

    // Extrinsics to set on the output PointCloudData after coordinate transformation
    std::optional<Extrinsics> targetExtrinsics_;

    // Cached frame transformation — used to detect intrinsic/extrinsic/size changes at runtime
    std::optional<ImgTransformation> lastTransformation_;
};

}  // namespace node
}  // namespace dai
