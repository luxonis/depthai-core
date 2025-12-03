#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/DetectionNetwork.hpp>
#include <depthai/pipeline/node/ImageAlign.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

// shared
#include <depthai/properties/SpatialDetectionNetworkProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief SpatialDetectionNetwork node. Runs a neural inference on input image and calculates spatial location data.
 */
class SpatialDetectionNetwork : public DeviceNodeCRTP<DeviceNode, SpatialDetectionNetwork, SpatialDetectionNetworkProperties> {
   public:
    explicit SpatialDetectionNetwork(const std::shared_ptr<Device>& device)
        : DeviceNodeCRTP<DeviceNode, SpatialDetectionNetwork, SpatialDetectionNetworkProperties>(device)
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
          ,
          input{neuralNetwork->input},
          outNetwork{neuralNetwork->out},
          passthrough{neuralNetwork->passthrough}
#endif
    {
        if(device) {
            auto platform = device->getPlatform();
            if(platform == Platform::RVC4) {
                if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
            }
        }
    };
    SpatialDetectionNetwork(std::unique_ptr<Properties> props)
        : DeviceNodeCRTP(std::move(props))
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
          ,
          input{neuralNetwork->input},
          outNetwork{neuralNetwork->out},
          passthrough{neuralNetwork->passthrough}
#endif
    {
        auto device = getDevice();
        if(device) {
            auto platform = device->getPlatform();
            if(platform == Platform::RVC4) {
                if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
            }
        }
    };
    SpatialDetectionNetwork(std::unique_ptr<Properties> props, bool confMode)
        : DeviceNodeCRTP(std::move(props), confMode)
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
          ,
          input{neuralNetwork->input},
          outNetwork{neuralNetwork->out},
          passthrough{neuralNetwork->passthrough}
#endif
    {
        auto device = getDevice();
        if(device) {
            auto platform = device->getPlatform();
            if(platform == Platform::RVC4) {
                if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
            }
        }
    };
    SpatialDetectionNetwork(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool confMode)
        : DeviceNodeCRTP(device, std::move(props), confMode)
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
          ,

          input{neuralNetwork->input},
          outNetwork{neuralNetwork->out},
          passthrough{neuralNetwork->passthrough}
#endif
    {
        if(device) {
            auto platform = device->getPlatform();
            if(platform == Platform::RVC4) {
                if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
            }
        }
    };

    constexpr static const char* NAME = "SpatialDetectionNetwork";

    /**
     * @brief Build SpatialDetectionNetwork node. Connect Camera and StereoDepth outputs to this node's inputs. Also call setNNArchive() with provided model
     * description.
     * @param inputRgb: Camera node
     * @param stereo: StereoDepth node
     * @param modelDesc: Neural network model description
     * @param fps: Desired frames per second
     * @param resizeMode: Resize mode for input color frames
     * @returns Shared pointer to SpatialDetectionNetwork node
     */
    std::shared_ptr<SpatialDetectionNetwork> build(const std::shared_ptr<Camera>& inputRgb,
                                                   const std::shared_ptr<StereoDepth>& stereo,
                                                   dai::NNModelDescription modelDesc,
                                                   std::optional<float> fps = std::nullopt,
                                                   std::optional<dai::ImgResizeMode> resizeMode = std::nullopt);

    /**
     * @brief Build SpatialDetectionNetwork node. Connect Camera and StereoDepth outputs to this node's inputs. Also call setNNArchive() with provided
     * NNArchive.
     * @param inputRgb: Camera node
     * @param stereo: StereoDepth node
     * @param nnArchive: Neural network archive
     * @param fps: Desired frames per second
     * @param resizeMode: Resize mode for input color frames
     * @returns Shared pointer to SpatialDetectionNetwork node
     */
    std::shared_ptr<SpatialDetectionNetwork> build(const std::shared_ptr<Camera>& inputRgb,
                                                   const std::shared_ptr<StereoDepth>& stereo,
                                                   const dai::NNArchive& nnArchive,
                                                   std::optional<float> fps = std::nullopt,
                                                   std::optional<dai::ImgResizeMode> resizeMode = std::nullopt);

    Subnode<NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};
    Subnode<DetectionParser> detectionParser{*this, "detectionParser"};
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
#endif

    /**
     * Input message with depth data used to retrieve spatial information about detected object
     * Default queue is non-blocking with size 4
     */
    Input inputDepth{*this, {"inputDepth", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, true}};

    /**
     * Input message with image data used to retrieve image transformation from detected object
     * Default queue is blocking with size 1
     */
    Input inputImg{*this, {"inputImg", DEFAULT_GROUP, true, 2, {{{DatatypeEnum::ImgFrame, false}}}, true}};

    /**
     * Input message with input detections object
     * Default queue is blocking with size 1
     */
    Input inputDetections{*this, {"inputDetections", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::ImgDetections, false}}}, true}};

    /**
     * Outputs ImgDetections message that carries parsed detection results.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::SpatialImgDetections, false}}}}};

    /**
     * Outputs mapping of detected bounding boxes relative to depth map
     * Suitable for when displaying remapped bounding boxes on depth frame
     */
    Output boundingBoxMapping{*this, {"boundingBoxMapping", DEFAULT_GROUP, {{{DatatypeEnum::SpatialLocationCalculatorConfig, false}}}}};

    /**
     * Passthrough message for depth frame on which the spatial location calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output of SpatialLocationCalculator node, which is used internally by SpatialDetectionNetwork.
     * Suitable when extra information is required from SpatialLocationCalculator node, e.g. minimum, maximum distance.
     */
    Output spatialLocationCalculatorOutput{*this, {"spatialLocationCalculatorOutput", DEFAULT_GROUP, {{{DatatypeEnum::SpatialLocationCalculatorData, false}}}}};

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
    void alignDepth(const std::shared_ptr<StereoDepth>& stereo, const std::shared_ptr<Camera>& camera);

   protected:
    using DeviceNodeCRTP::DeviceNodeCRTP;
};

}  // namespace node
}  // namespace dai
