#pragma once

#include <depthai/modelzoo/Zoo.hpp>
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
    /**
     * Construct a DetectionNetwork node bound to a device.
     */
    DetectionNetwork(const std::shared_ptr<Device>& device);

    [[nodiscard]] static std::shared_ptr<DetectionNetwork> create(const std::shared_ptr<Device>& device) {
        auto networkPtr = std::make_shared<DetectionNetwork>(device);
        networkPtr->buildInternal();
        return networkPtr;
    }

    /**
     * @brief Build DetectionNetwork node. Connect output to this node's input. Also call setNNArchive() with provided NNArchive.
     * @param output: Output to link
     * @param nnArchive: Neural network archive
     * @returns Shared pointer to DetectionNetwork node
     */
    std::shared_ptr<DetectionNetwork> build(Node::Output& input, const NNArchive& nnArchive);
    /**
     * Build DetectionNetwork node with a model description.
     */
    std::shared_ptr<DetectionNetwork> build(const std::shared_ptr<Camera>& input,
                                            NNModelDescription modelDesc,
                                            std::optional<float> fps = std::nullopt,
                                            std::optional<dai::ImgResizeMode> resizeMode = dai::ImgResizeMode::CROP);

    /**
     * @brief Build DetectionNetwork node. Connect Camera output to this node's input. Also call setNNArchive() with provided NNArchive.
     * @param input: Camera node
     * @param nnArchive: Neural network archive
     * @param fps: Desired frames per second
     * @param resizeMode: Resize mode for input frames
     * @returns Shared pointer to DetectionNetwork node
     */
    std::shared_ptr<DetectionNetwork> build(const std::shared_ptr<Camera>& input,
                                            const NNArchive& nnArchive,
                                            std::optional<float> fps = std::nullopt,
                                            std::optional<dai::ImgResizeMode> resizeMode = dai::ImgResizeMode::CROP);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * @brief Build DetectionNetwork node. Connect ReplayVideo output to this node's input. Also call setNNArchive() with provided model description.
     * @param input: ReplayVideo node
     * @param modelDesc: Neural network model description
     * @param fps: Desired frames per second
     * @returns Shared pointer to DetectionNetwork node
     */
    std::shared_ptr<DetectionNetwork> build(const std::shared_ptr<ReplayVideo>& input, NNModelDescription modelDesc, std::optional<float> fps = std::nullopt);
    /**
     * @brief Build DetectionNetwork node. Connect ReplayVideo output to this node's input.
     * @param input: ReplayVideo node
     * @param nnArchive: Neural network archive
     * @param fps: Desired frames per second
     * @returns Shared pointer to DetectionNetwork node
     */

    std::shared_ptr<DetectionNetwork> build(const std::shared_ptr<ReplayVideo>& input, const NNArchive& nnArchive, std::optional<float> fps = std::nullopt);
#endif
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
     * Load network model into assets.
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

    std::vector<std::pair<Input&, std::shared_ptr<Capability>>> getRequiredInputs() override;

    /**
     * Return class labels if available.
     */
    std::optional<std::vector<std::string>> getClasses() const;

    virtual void buildInternal() override;

   private:
    void setNNArchiveBlob(const NNArchive& nnArchive);
    void setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves);
    void setNNArchiveOther(const NNArchive& nnArchive);
    NNArchive createNNArchive(NNModelDescription& modelDesc);
};

}  // namespace node
}  // namespace dai
