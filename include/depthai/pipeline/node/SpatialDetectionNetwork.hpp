#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/node/DetectionNetwork.hpp>

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
    SpatialDetectionNetwork() : input{neuralNetwork->input}, outNetwork{neuralNetwork->out}, passthrough{neuralNetwork->passthrough} {};
    SpatialDetectionNetwork(std::unique_ptr<Properties> props)
        :DeviceNodeCRTP(std::move(props)), input{neuralNetwork->input}, outNetwork{neuralNetwork->out}, passthrough{neuralNetwork->passthrough} {};
    SpatialDetectionNetwork(std::unique_ptr<Properties> props, bool confMode)
        :DeviceNodeCRTP(std::move(props), confMode), input{neuralNetwork->input}, outNetwork{neuralNetwork->out}, passthrough{neuralNetwork->passthrough} {};

    constexpr static const char* NAME = "SpatialDetectionNetwork";
    Subnode<NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};
    Subnode<DetectionParser> detectionParser{*this, "detectionParser"};
    void build();

   public:
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
    Input inputDepth{true, *this, "inputDepth", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Input message with image data used to retrieve image transformation from detected object
     * Default queue is blocking with size 1
     */
    Input inputImg{true, *this, "inputImg", Input::Type::SReceiver, true, 2, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Input message with input detections object
     * Default queue is blocking with size 1
     */
    Input inputDetections{true, *this, "inputDetections", Input::Type::SReceiver, true, 5, true, {{DatatypeEnum::ImgDetections, false}}};

    /**
     * Outputs ImgDetections message that carries parsed detection results.
     */
    Output out{true, *this, "out", Output::Type::MSender, {{DatatypeEnum::SpatialImgDetections, false}}};

    /**
     * Outputs mapping of detected bounding boxes relative to depth map
     *
     * Suitable for when displaying remapped bounding boxes on depth frame
     */
    Output boundingBoxMapping{true, *this, "boundingBoxMapping", Output::Type::MSender, {{DatatypeEnum::SpatialLocationCalculatorConfig, false}}};

    /**
     * Passthrough message for depth frame on which the spatial location calculation was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{true, *this, "passthroughDepth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Output of SpatialLocationCalculator node, which is used internally by SpatialDetectionNetwork.
     * Suitable when extra information is required from SpatialLocationCalculator node, e.g. minimum, maximum distance.
     */
    Output spatialLocationCalculatorOutput{
        true, *this, "spatialLocationCalculatorOutput", Output::Type::MSender, {{DatatypeEnum::SpatialLocationCalculatorData, false}}};

    /** Backwards compatibility interface **/
    // Specify local filesystem path to load the blob (which gets loaded at loadAssets)
    /**
     * Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlobPath(const dai::Path& path);

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
    void setBlob(const dai::Path& path);

    /**
     * Load network xml and bin files into assets.
     * @param xmlModelPath Path to the .xml model file.
     * @param binModelPath Path to the .bin file of the model. If left empty, it is assumed that the
     *                     name is the same as the xml model with a .bin extension.
     * @note If this function is called, the device automatically loads the model from the XML and not the blob
     */
    void setXmlModelPath(const dai::Path& xmlModelPath, const dai::Path& binModelPath = "");

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
};

/**
 * MobileNetSpatialDetectionNetwork node. Mobilenet-SSD based network with spatial location data.
 */
class MobileNetSpatialDetectionNetwork : public SpatialDetectionNetwork {
   public:
    void build();
    [[nodiscard]] static std::shared_ptr<MobileNetSpatialDetectionNetwork> create() {
        auto n = std::make_shared<MobileNetSpatialDetectionNetwork>();
        n->build();
        return n;
    }
};

/**
 * YoloSpatialDetectionNetwork node. Yolo-based network with spatial location data.
 */
class YoloSpatialDetectionNetwork : public SpatialDetectionNetwork {
   public:
    void build();
    [[nodiscard]] static std::shared_ptr<YoloSpatialDetectionNetwork> create() {
        auto n = std::make_shared<YoloSpatialDetectionNetwork>();
        n->build();
        return n;
    }

    /// Set num classes
    void setNumClasses(const int numClasses);
    /// Set coordianate size
    void setCoordinateSize(const int coordinates);
    /// Set anchors
    void setAnchors(std::vector<float> anchors);
    /// Set anchor masks
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);
    /// Set Iou threshold
    void setIouThreshold(float thresh);

    /// Get num classes
    int getNumClasses() const;
    /// Get coordianate size
    int getCoordinateSize() const;
    /// Get anchors
    std::vector<float> getAnchors() const;
    /// Get anchor masks
    std::map<std::string, std::vector<int>> getAnchorMasks() const;
    /// Get Iou threshold
    float getIouThreshold() const;
};

}  // namespace node
}  // namespace dai
