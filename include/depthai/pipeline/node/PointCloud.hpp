#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/properties/PointCloudProperties.hpp>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/HousingCoordinateSystem.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace spdlog {
class logger;
}  // namespace spdlog

#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "kompute/Kompute.hpp"
#endif

namespace dai {
namespace node {

/**
 * @brief PointCloud node. Computes point cloud from depth frames.
 *
 * One depth stream is linked to `inputDepth` (optionally colorized through `getColorInput()`).
 * Additional depth streams can be linked with `getDepthInput(name)`; every stream is deprojected
 * with its own intrinsics, transformed into the common target coordinate system using its
 * frame extrinsics and the merged result is sent as a single PointCloudData message. Depth
 * streams from several devices share a coordinate system once the pipeline carries a
 * multi-device calibration (Pipeline::setMultiDeviceCalibration).
 */
class PointCloud : public DeviceNodeCRTP<DeviceNode, PointCloud, PointCloudProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "PointCloud";

    class Impl {
       public:
        Impl() = default;

        void setLogger(const std::shared_ptr<::spdlog::logger>& log);

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
        void setDistortion(CameraModel model, std::vector<float> coefficients);
        void setExtrinsics(const std::vector<std::vector<float>>& transformMatrix);
        void clearExtrinsics();

        // Adopt the compute method (CPU / multi-threaded CPU / GPU) of another Impl
        void copyComputeSettingsFrom(const Impl& other);

        LengthUnit targetLengthUnit = LengthUnit::MILLIMETER;

       private:
        void initializeGPU(uint32_t device);
        void cacheUndistortedRays();
        template <typename PointT>
        void transformPointsCPU(std::vector<PointT>& points);
        void calcPointsChunkDense(const uint8_t* depthData, std::vector<Point3f>& points, unsigned int startRow, unsigned int endRow);
        void calcPointsChunkDenseColored(
            const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points, unsigned int startRow, unsigned int endRow);
        void computePointCloudDenseCPU(const uint8_t* depthData, std::vector<Point3f>& points);
        void computePointCloudDenseCPUMT(const uint8_t* depthData, std::vector<Point3f>& points);
        void computePointCloudDenseGPU(const uint8_t* depthData, std::vector<Point3f>& points);
        void computePointCloudDenseColoredCPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points);
        void computePointCloudDenseColoredCPUMT(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points);

        enum class ComputeMethod { CPU, CPU_MT, GPU };
        ComputeMethod computeMethod = ComputeMethod::CPU;
        uint32_t gpuDevice = 0;

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
        CameraModel distortionModel = CameraModel::Perspective;
        std::vector<float> distortionCoefficients;
        std::vector<Point2f> undistortedRays;
        bool hasDistortion = false;
        bool gpuDistortionFallbackWarned = false;
        unsigned int width = 0u, height = 0u;
        size_t size = 0;
        bool intrinsicsSet = false;
        uint32_t threadNum = 2;

        std::vector<std::vector<float>> extrinsics;
        bool hasExtrinsics = false;

        std::shared_ptr<::spdlog::logger> logger;
    };

   protected:
    Properties& getProperties() override;
    using DeviceNodeCRTP::DeviceNodeCRTP;

   public:
    PointCloud();
    PointCloud(std::unique_ptr<Properties> props);
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

    /**
     * Sync input key of a depth stream: "depth" for the default stream (empty name), "depth/<name>" otherwise.
     * The same key identifies the depth frame inside the synced MessageGroup.
     */
    static std::string getDepthInputKey(const std::string& name);

    /**
     * Sync input key of the color stream paired with a depth stream: "color" for the default stream (empty name), "color/<name>" otherwise.
     */
    static std::string getColorInputKey(const std::string& name);

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    InputMap& syncInputs = sync->inputs;

    /**
     * Input message with depth data used to create the point cloud.
     * Routed through the internal Sync subnode. Equivalent to getDepthInput("").
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

    /**
     * Get (or create) an additional depth input.
     *
     * Every depth stream linked to this node is synchronized by the internal Sync subnode,
     * deprojected with its own intrinsics and transformed into the target coordinate system
     * using its frame extrinsics. The clouds of all streams are merged into one output
     * PointCloudData. All depth frames must therefore share a target coordinate system
     * (Extrinsics::toDeviceId / Extrinsics::toCameraSocket); depth from several devices
     * requires a multi-device calibration on the pipeline. Groups whose streams do not share
     * a coordinate system are dropped with a warning.
     *
     * When the depth streams come from more than one device and the Sync timestamp source
     * is left at its default, the Sync subnode is moved to the host at build time so that
     * the streams can be paired with host timestamps.
     *
     * @param name Name of the depth stream. An empty name refers to inputDepth.
     */
    Input& getDepthInput(const std::string& name);

    /**
     * Get (or create) the color input paired with the depth stream `name`.
     * The output is colorized only when every depth stream has a matching color frame.
     *
     * @param name Name of the depth stream. An empty name refers to the default color input.
     */
    Input& getColorInput(const std::string& name);

    /**
     * Names of the depth streams linked to this node, the default (unnamed) stream first.
     */
    std::vector<std::string> getDepthInputNames() const;

    /**
     * Moves the Sync subnode to the host when depth streams come from more than one device.
     */
    void buildStage1() override;

    /**
     * Drops Sync entries nobody linked (an unused default depth input next to named depth
     * streams, unused color inputs) so that the Sync subnode does not wait for them.
     */
    void postBuildStage() override;
#endif

    /**
     * Outputs PointCloudData message
     */
    Output outputPointCloud{*this, {"outputPointCloud", DEFAULT_GROUP, {{{DatatypeEnum::PointCloudData, false}}}}};

    /**
     * Passthrough depth from which the point cloud was calculated.
     * Suitable for when input queue is set to non-blocking behavior.
     * With several depth streams every depth frame of the merged group is passed through, in stream order.
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
     */
    void setTargetCoordinateSystem(CameraBoardSocket targetCamera);

    /**
     * Set target coordinate system to housing coordinate system
     * Point cloud will be transformed to this housing coordinate system
     * @param housingCS Target housing coordinate system
     */
    void setTargetCoordinateSystem(HousingCoordinateSystem housingCS);

    /**
     * Deprecated: use setTargetCoordinateSystem(targetCamera) instead.
     */
    void setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpecTranslation);

    /**
     * Deprecated: use setTargetCoordinateSystem(housingCS) instead.
     */
    void setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpecTranslation);

    bool runOnHost() const override;

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    void buildInternal() override;
#endif

   private:
    Pimpl<Impl> pimplPointCloud;

    /// Private input receiving synced MessageGroup from Sync subnode
    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};

    /// Per depth stream state. The default stream uses pimplPointCloud, additional streams own an Impl configured alike.
    struct DepthStream {
        std::unique_ptr<Impl> impl;
        bool initialized = false;
        // Cached frame transformation — used to detect intrinsic/extrinsic/size changes at runtime
        std::optional<ImgTransformation> lastTransformation;
        // Extrinsics to set on the output PointCloudData after coordinate transformation
        std::optional<Extrinsics> targetExtrinsics;
    };

    /// Depth frame (and optional color frame) of one stream inside a synced group
    struct StreamFrames {
        std::string name;
        std::shared_ptr<ImgFrame> depth;
        std::shared_ptr<ImgFrame> color;
    };

    void run() override;
    DepthStream& getDepthStream(const std::string& name);
    Impl& getImpl(DepthStream& stream);
    std::vector<StreamFrames> collectStreamFrames(MessageGroup& group);
    void initialize(DepthStream& stream, const ImgFrame& depthFrame, const PointCloudConfig& config);
    bool hasTransformationChanged(DepthStream& stream, const ImgFrame& frame);
    bool isValidDepthFrame(const ImgFrame& depthFrame);
    bool haveCommonTargetCoordinateSystem(const std::vector<StreamFrames>& frames);
    CalibrationHandler getCalibrationFor(const std::string& deviceId);

    // Helper methods for initialize()
    void setIntrinsicsFromFrame(Impl& impl, const ImgFrame& frame);
    void setCoordinateTransformation(DepthStream& stream, const ImgFrame& depthFrame, const PointCloudConfig& config);

    // Processing methods for the two code paths; append the points of one stream
    void computeDepthOnly(Impl& impl, const ImgFrame& depthFrame, bool organized, std::vector<Point3f>& points);
    bool canColorize(const ImgFrame& depthFrame, const ImgFrame& colorFrame);
    void computeColorized(Impl& impl, const ImgFrame& depthFrame, const ImgFrame& colorFrame, bool organized, std::vector<Point3fRGBA>& points);

    bool runOnHostVar = true;
    bool colorMode = false;
    bool coordinateSystemMismatchWarned = false;
    bool mixedColorWarned = false;
    bool organizedLayoutWarned = false;

    std::map<std::string, DepthStream> depthStreams;
};

}  // namespace node
}  // namespace dai
