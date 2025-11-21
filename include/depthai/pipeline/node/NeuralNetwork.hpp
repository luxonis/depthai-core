#pragma once

#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "depthai/pipeline/node/host/Replay.hpp"
#endif
// standard
#include <fstream>

// shared
#include <depthai/properties/NeuralNetworkProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief NeuralNetwork node. Runs a neural inference on input data.
 */
class NeuralNetwork : public DeviceNodeCRTP<DeviceNode, NeuralNetwork, NeuralNetworkProperties> {
   public:
    constexpr static const char* NAME = "NeuralNetwork";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    ~NeuralNetwork() override;

    /**
     * @brief Build NeuralNetwork node. Connect output to this node's input. Also call setNNArchive() with provided NNArchive.
     *
     * @param output: Output to link
     * @param nnArchive: Neural network archive
     * @returns Shared pointer to NeuralNetwork node
     */
    std::shared_ptr<NeuralNetwork> build(Node::Output& input, const NNArchive& nnArchive);
    std::shared_ptr<NeuralNetwork> build(const std::shared_ptr<Camera>& input, NNModelDescription modelDesc, std::optional<float> fps = std::nullopt);
    std::shared_ptr<NeuralNetwork> build(const std::shared_ptr<Camera>& input, NNArchive nnArchive, std::optional<float> fps = std::nullopt);
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::shared_ptr<NeuralNetwork> build(const std::shared_ptr<ReplayVideo>& input, NNModelDescription modelDesc, std::optional<float> fps = std::nullopt);
    std::shared_ptr<NeuralNetwork> build(const std::shared_ptr<ReplayVideo>& input, const NNArchive& nnArchive, std::optional<float> fps = std::nullopt);
#endif

    /**
     * Input message with data to be inferred upon
     */
    Input input{*this, {"in", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::Buffer, true}}}, true}};

    /**
     * Outputs NNData message that carries inference results
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::NNData, false}}}}};

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthrough{*this, {"passthrough", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * Inputs mapped to network inputs. Useful for inferring from separate data sources
     * Default input is non-blocking with queue size 1 and waits for messages
     */
    InputMap inputs{*this, "inputs", {DEFAULT_NAME, DEFAULT_GROUP, false, 1, {{{DatatypeEnum::Buffer, true}}}, true}};

    /**
     * Passthroughs which correspond to specified input
     */
    OutputMap passthroughs{*this, "passthroughs", {"", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    /**
     * @brief Get the archive owned by this Node.
     *
     * @returns constant reference to this Nodes archive
     */
    std::optional<std::reference_wrapper<const NNArchive>> getNNArchive() const;

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
     * Load network model into assets and use once pipeline is started.
     *
     * @param model Network model
     */
    void setOtherModelFormat(std::vector<uint8_t> model);

    /**
     * Load network model into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network model.
     * @param path Path to the network model
     */
    void setOtherModelFormat(const std::filesystem::path& path);

    /**
     * Load network xml and bin files into assets.
     * @param xmlModelPath Path to the neural network model file.
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
     * Set model from Device Model Zoo
     * @param model DeviceModelZoo model enum
     * @note Only applicable for RVC4 devices with OS 1.20.5 or higher
     */
    void setModelFromDeviceZoo(DeviceModelZoo model);

   private:
    void setNNArchiveBlob(const NNArchive& nnArchive);
    void setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves);
    void setNNArchiveOther(const NNArchive& nnArchive);
    NNArchive createNNArchive(NNModelDescription& modelDesc);
    ImgFrameCapability getFrameCapability(const NNArchive& nnArchive, std::optional<float> fps);
    std::optional<NNArchive> nnArchive;
};

}  // namespace node
}  // namespace dai
