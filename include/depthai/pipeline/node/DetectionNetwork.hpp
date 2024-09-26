#pragma once

#include <depthai/modelzoo/NNModelDescription.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Camera.hpp>
#include <depthai/pipeline/node/DetectionParser.hpp>
#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <optional>
#include <vector>

#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/openvino/OpenVINO.hpp"

namespace dai {
namespace node {

/**
 * @brief DetectionNetwork, base for different network specializations
 */
class DetectionNetwork : public DeviceNodeGroup {
   public:
    DetectionNetwork(const std::shared_ptr<Device>& device);

    [[nodiscard]] static std::shared_ptr<DetectionNetwork> create(const std::shared_ptr<Device>& device) {
        auto networkPtr = std::make_shared<DetectionNetwork>(device);
        networkPtr->buildInternal();
        return networkPtr;
    }


    void setModel(const depthai::model::ModelVariant& model) {
        std::visit([this](auto &&p){this->setModel(p);}, model);
    }

    void setModel(const depthai::model::BlobModel& model) {
        neuralNetwork->setModel(model);
        detectionParser->setModel(model);
    }

    void setModel(const depthai::model::SuperBlobModel& model) {
        neuralNetwork->setModel(model);
        detectionParser->setModel(model);
    }

    void setModel(const depthai::model::DlcModel& model) {
        neuralNetwork->setModel(model);
        detectionParser->setModel(model);
    }

    std::shared_ptr<DetectionNetwork> build(Node::Output& input, const NNArchive& nnArchive);
    std::shared_ptr<DetectionNetwork> build(Node::Output& input, const depthai::model::ModelVariant& model) {
        setModel(model);
        input.link(this->input);
        return std::static_pointer_cast<DetectionNetwork>(shared_from_this());
    }
    std::shared_ptr<DetectionNetwork> build(std::shared_ptr<Camera> input, dai::NNModelDescription modelDesc, float fps = 30.0f);

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
     * @brief Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default number of shaves.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive for this Node, throws if the archive's type is not SUPERBLOB
     *
     * @param nnArchive: NNArchive to set
     * @param numShaves: Number of shaves to use
     */
    void setNNArchive(const NNArchive& nnArchive, int numShaves);

    /**
     * @brief Download model from zoo and set it for this Node
     *
     * @param description: Model description to download
     * @param useCached: Use cached model if available
     */
    void setFromModelZoo(NNModelDescription description, bool useCached = true);

    /**
     * @brief Download model from zoo and set it for this node.
     *
     * @param description: Model description to download
     * @param numShaves: Number of shaves to use
     * @param useCached: Use cached model if available
     */
    void setFromModelZoo(NNModelDescription description, int numShaves, bool useCached = true);

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
     * Load network model into assets.
     * @param modelPath Path to the model file.
     */
    void setModelPath(const dai::Path& modelPath);

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

    std::vector<std::pair<Input&, std::shared_ptr<Capability>>> getRequiredInputs() override;

    std::optional<std::vector<std::string>> getClasses() const;

    virtual void buildInternal();

   private:
    void setNNArchiveBlob(const NNArchive& nnArchive);
    void setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves);
    void setNNArchiveOther(const NNArchive& nnArchive);
};

/**
 * @brief MobileNetDetectionNetwork node. Parses MobileNet results
 */
class MobileNetDetectionNetwork : public DetectionNetwork {
   public:
    MobileNetDetectionNetwork(const std::shared_ptr<Device>& device) : DetectionNetwork(device) {
        static bool warned = false;
        if(!warned) {
            std::cerr << "MobileNetDetectionNetwork is deprecated, use DetectionNetwork instead" << std::endl;
            warned = true;
        }
    }

    [[nodiscard]] static std::shared_ptr<MobileNetDetectionNetwork> create(const std::shared_ptr<Device>& device) {
        auto networkPtr = std::make_shared<MobileNetDetectionNetwork>(device);
        networkPtr->buildInternal();
        return networkPtr;
    }

    void buildInternal() override;
};

/**
 * @brief YoloDetectionNetwork node. Parses Yolo results
 */
class YoloDetectionNetwork : public DetectionNetwork {
   public:
    YoloDetectionNetwork(const std::shared_ptr<Device>& device) : DetectionNetwork(device) {
        static bool warned = false;
        if(!warned) {
            std::cerr << "YoloDetectionNetwork is deprecated, use DetectionNetwork instead" << std::endl;
            warned = true;
        }
    }

    [[nodiscard]] static std::shared_ptr<YoloDetectionNetwork> create(const std::shared_ptr<Device>& device) {
        auto networkPtr = std::make_shared<YoloDetectionNetwork>(device);
        networkPtr->buildInternal();
        return networkPtr;
    }

    /// Set num classes
    void setNumClasses(int numClasses);
    /// Set coordianate size
    void setCoordinateSize(int coordinates);
    /// Set anchors
    [[deprecated("Use setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors) instead")]] void setAnchors(std::vector<float> anchors);
    /// Set anchor masks
    [[deprecated("Use setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors) instead")]] void setAnchorMasks(
        std::map<std::string, std::vector<int>> anchorMasks);

    /**
     * Sets anchors with masks included.
     *
     * @param anchors Anchors grouped by masks from biggest mask(ex.: side26) to smallest(ex.: side13)
     *
     * Ordering should be from biggest to smallest (example: { side26Anchors, side13Anchors, side7Anchors }).
     * Format is:
     * {
     *   { // side26Anchors
     *     {
     *       10, // width
     *       14 // height
     *     },
     *     {
     *       23, // width
     *       27 // height
     *     },
     *     {
     *       37, // width
     *       58 // height
     *     }
     *   },
     *   { // side13Anchors
     *     {
     *       81, // width
     *       82 // height
     *     },
     *     {
     *       135, // width
     *       169 // height
     *     },
     *     {
     *       344, // width
     *       319 // height
     *     },
     *     ... other anchors for side13
     *   },
     *   ... other sides (ordered from biggest to smallest) anchors
     * }
     */
    void setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors);

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

    void buildInternal() override;
};

}  // namespace node
}  // namespace dai
