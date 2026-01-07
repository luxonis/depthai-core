#pragma once

#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/DetectionNetwork.hpp>
#include <depthai/pipeline/node/ImageAlign.hpp>
#include <depthai/pipeline/node/SpatialLocationCalculator.hpp>
#include <variant>

// depth map source nodes
#include <depthai/pipeline/node/NeuralDepth.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/ToF.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// shared
#include <depthai/properties/SpatialDetectionNetworkProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Variant type representing different depth sources.
 * Supported depth sources: StereoDepth, NeuralDepth, ToF
 */
using DepthSource = std::variant<std::shared_ptr<StereoDepth>, std::shared_ptr<NeuralDepth>, std::shared_ptr<ToF>>;

/**
 * @brief SpatialDetectionNetwork node. Runs a neural inference on input image and calculates spatial location data.
 */
class SpatialDetectionNetwork : public DeviceNodeGroup {
   public:
    using Properties = SpatialDetectionNetworkProperties;

    SpatialDetectionNetwork(const std::shared_ptr<Device>& device);

    [[nodiscard]] static std::shared_ptr<SpatialDetectionNetwork> create(const std::shared_ptr<Device>& device) {
        auto networkPtr = std::make_shared<SpatialDetectionNetwork>(device);
        networkPtr->buildInternal();
        return networkPtr;
    }

    SpatialDetectionNetwork(std::unique_ptr<Properties> props);

    SpatialDetectionNetwork(std::unique_ptr<Properties> props, bool confMode);

    SpatialDetectionNetwork(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool confMode);

    constexpr static const char* NAME = "SpatialDetectionNetwork";

    Properties& properties;

    /**
     * @brief Build SpatialDetectionNetwork node with specified depth source. Connect Camera and depth source outputs to this node's inputs.
     * Also call setNNArchive() with provided model description.
     * @param inputRgb Camera node
     * @param depthSource Depth source node (StereoDepth, NeuralDepth, or ToF)
     * @param modelDesc Neural network model description
     * @param fps Desired frames per second
     * @param resizeMode Resize mode for input color frames
     * @returns Shared pointer to SpatialDetectionNetwork node
     */
    std::shared_ptr<SpatialDetectionNetwork> build(const std::shared_ptr<Camera>& inputRgb,
                                                   const DepthSource& depthSource,
                                                   dai::NNModelDescription modelDesc,
                                                   std::optional<float> fps = std::nullopt,
                                                   std::optional<dai::ImgResizeMode> resizeMode = std::nullopt);

    /**
     * @brief Build SpatialDetectionNetwork node with specified depth source. Connect Camera and depth source outputs to this node's inputs.
     * Also call setNNArchive() with provided NNArchive.
     * @param inputRgb Camera node
     * @param depthSource Depth source node (StereoDepth, NeuralDepth, or ToF)
     * @param nnArchive Neural network archive
     * @param fps Desired frames per second
     * @param resizeMode Resize mode for input color frames
     * @returns Shared pointer to SpatialDetectionNetwork node
     */
    std::shared_ptr<SpatialDetectionNetwork> build(const std::shared_ptr<Camera>& inputRgb,
                                                   const DepthSource& depthSource,
                                                   const dai::NNArchive& nnArchive,
                                                   std::optional<float> fps = std::nullopt,
                                                   std::optional<dai::ImgResizeMode> resizeMode = std::nullopt);

    Subnode<NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};
    Subnode<DetectionParser> detectionParser{*this, "detectionParser"};
    Subnode<SpatialLocationCalculator> spatialLocationCalculator{*this, "spatialLocationCalculator"};

    std::unique_ptr<Subnode<ImageAlign>> depthAlign;

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    /**
     * Input message with data to be inferred upon
     * Default queue is blocking with size 5
     */
    Input& input;

    /**
     * Outputs unparsed inference results.
     */
    Output& outNetwork;

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output& passthrough;

    /**
     * Input message with depth data used to retrieve spatial information about detected object
     * Default queue is non-blocking with size 4
     */
    Input& inputDepth;

    // /**
    //  * Input SpatialLocationCalculatorConfig message with ability to modify parameters in runtime.
    //  * Default queue is non-blocking with size 4.
    //  */
    // Input& inputConfig;

    /**
     * Outputs ImgDetections message that carries parsed detection results.
     */
    Output& out;

    /**
     * Passthrough message for depth frame on which the spatial location calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output& passthroughDepth;

#endif

    /**
     * @brief Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default number of shaves.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Download model from zoo and set it for this Node
     *
     * @param description: Model description to download
     * @param useCached: Use cached model if available
     */
    void setFromModelZoo(NNModelDescription description, bool useCached = true);

    /**
     * @brief Set NNArchive for this Node, throws if the archive's type is not SUPERBLOB
     *
     * @param nnArchive: NNArchive to set
     * @param numShaves: Number of shaves to use
     */
    void setNNArchive(const NNArchive& nnArchive, int numShaves);

    /** Backwards compatibility interface **/
    // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
    /**
     * Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlobPath(const std::filesystem::path& path);

    /**
     * Load network blob into assets and use once pipeline is started.
     *
     * @param blob Network blob
     */
    void setBlob(OpenVINO::Blob blob);

    /**
     * Same functionality as the setBlobPath(). Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlob(const std::filesystem::path& path);

    /**
     * Load network file into assets.
     * @param modelPath Path to the model file.
     */
    void setModelPath(const std::filesystem::path& modelPath);

    /**
     * Specifies how many frames will be available in the pool
     * @param numFrames How many frames will pool have
     */
    void setNumPoolFrames(int numFrames);

    /**
     * How many threads should the node use to run the network.
     * @param numThreads Number of threads to dedicate to this node
     */
    void setNumInferenceThreads(int numThreads);

    /**
     * How many Neural Compute Engines should a single thread use for inference
     * @param numNCEPerThread Number of NCE per thread
     */
    void setNumNCEPerInferenceThread(int numNCEPerThread);

    /**
     * How many Shaves should a single thread use for inference
     * @param numShavesPerThread Number of shaves per thread
     */
    void setNumShavesPerInferenceThread(int numShavesPerThread);

    /**
     * Specifies backend to use
     * @param backend String specifying backend to use
     */
    void setBackend(std::string backend);

    /**
     * Set backend properties
     * @param backendProperties backend properties map
     */
    void setBackendProperties(std::map<std::string, std::string> properties);

    /**
     * How many inference threads will be used to run the network
     * @returns Number of threads, 0, 1 or 2. Zero means AUTO
     */
    int getNumInferenceThreads();

    /**
     * Specifies confidence threshold at which to filter the rest of the detections.
     * @param thresh Detection confidence must be greater than specified threshold to be added to the list
     */
    void setConfidenceThreshold(float thresh);

    /**
     * Retrieves threshold at which to filter the rest of the detections.
     * @returns Detection confidence
     */
    float getConfidenceThreshold() const;

    /** Custom interface **/
    /**
     * Specifies scale factor for detected bounding boxes.
     * @param scaleFactor Scale factor must be in the interval (0,1].
     */
    void setBoundingBoxScaleFactor(float scaleFactor);

    /**
     * Specifies lower threshold in depth units (millimeter by default) for depth values which will used to calculate spatial data
     * @param lowerThreshold LowerThreshold must be in the interval [0,upperThreshold] and less than upperThreshold.
     */
    void setDepthLowerThreshold(uint32_t lowerThreshold);

    /**
     * Specifies upper threshold in depth units (millimeter by default) for depth values which will used to calculate spatial data
     * @param upperThreshold UpperThreshold must be in the interval (lowerThreshold,65535].
     */
    void setDepthUpperThreshold(uint32_t upperThreshold);

    /**
     * Specifies spatial location calculator algorithm: Average/Min/Max
     * @param calculationAlgorithm Calculation algorithm.
     */
    void setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm calculationAlgorithm);

    /**
     * Specifies spatial location calculator step size for depth calculation.
     * Step size 1 means that every pixel is taken into calculation, size 2 means every second etc.
     * @param stepSize Step size.
     */
    void setSpatialCalculationStepSize(int stepSize);

    /// Get classes labels
    std::optional<std::vector<std::string>> getClasses() const;

    void buildInternal() override;

   private:
    void setNNArchiveBlob(const NNArchive& nnArchive);
    void setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves);
    void setNNArchiveOther(const NNArchive& nnArchive);
    NNArchive createNNArchive(NNModelDescription& modelDesc);

    // Unified depth alignment helper
    void alignDepth(const DepthSource& depthSource, const std::shared_ptr<Camera>& camera);

    // Type-specific alignment implementations
    void alignDepthImpl(const std::shared_ptr<StereoDepth>& stereo, const std::shared_ptr<Camera>& camera);
    void alignDepthImpl(const std::shared_ptr<NeuralDepth>& neuralDepth, const std::shared_ptr<Camera>& camera);
    void alignDepthImpl(const std::shared_ptr<ToF>& tof, const std::shared_ptr<Camera>& camera);
};

}  // namespace node
}  // namespace dai
