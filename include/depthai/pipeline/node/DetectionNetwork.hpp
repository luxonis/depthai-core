#pragma once

#include <depthai/pipeline/NodeGroup.hpp>
#include <depthai/pipeline/node/DetectionParser.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>

#include "depthai/openvino/OpenVINO.hpp"

// standard
#include <fstream>

namespace dai {
namespace node {

/**
 * @brief DetectionNetwork, base for different network specializations
 */
class DetectionNetwork : public NodeGroup {
   public:
    void build();
    DetectionNetwork() : out{detectionParser->out}, outNetwork{neuralNetwork->out}, input{neuralNetwork->input}, passthrough{neuralNetwork->passthrough}, outSegmentation{detectionParser->outSegmentation} {};

   public:
    Subnode<NeuralNetwork> neuralNetwork{*this, "neuralNetwork"};
    Subnode<DetectionParser> detectionParser{*this, "detectionParser"};
    /**
     * Outputs ImgDetections message that carries parsed detection results.
     * Overrides NeuralNetwork 'out' with ImgDetections output message type.
     */
    Output& out;

    /**
     * Outputs unparsed inference results.
     */
    Output& outNetwork;

    /**
     * Input message with data to be inferred upon
     * Default queue is blocking with size 5
     */
    Input& input;

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output& passthrough;

    /**
     * Outputs image frame segmentation
     */
    Output& outSegmentation;


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
};

/**
 * @brief MobileNetDetectionNetwork node. Parses MobileNet results
 */
class MobileNetDetectionNetwork : public DetectionNetwork {
   public:
    [[nodiscard]] static std::shared_ptr<MobileNetDetectionNetwork> create() {
        auto n = std::make_shared<MobileNetDetectionNetwork>();
        n->build();
        return n;
    }
    void build();
};

/**
 * @brief YoloDetectionNetwork node. Parses Yolo results
 */
class YoloDetectionNetwork : public DetectionNetwork {
   public:
    void build();
    [[nodiscard]] static std::shared_ptr<YoloDetectionNetwork> create() {
        auto n = std::make_shared<YoloDetectionNetwork>();
        n->build();
        return n;
    }

    /// Set num classes
    void setNumClasses(int numClasses);
    /// Set coordianate size
    void setCoordinateSize(int coordinates);
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
