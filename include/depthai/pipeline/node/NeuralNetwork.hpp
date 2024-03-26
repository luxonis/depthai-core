#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/openvino/OpenVINO.hpp"

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

   protected:
    std::optional<OpenVINO::Version> getRequiredOpenVINOVersion() override;
    std::optional<OpenVINO::Version> networkOpenvinoVersion;

   public:
    /**
     * Input message with data to be inferred upon
     * Default queue is blocking with size 5
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 5, true, {{DatatypeEnum::Buffer, true}}};

    /**
     * Outputs NNData message that carries inference results
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::NNData, false}}};

    /**
     * Passthrough message on which the inference was performed.
     *
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthrough{*this, "passthrough", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}};

    /**
     * Inputs mapped to network inputs. Useful for inferring from separate data sources
     * Default input is non-blocking with queue size 1 and waits for messages
     */
    InputMap inputs{*this, "inputs", Input(*this, "", Input::Type::SReceiver, false, 1, true, {{DatatypeEnum::Buffer, true}}, false)};

    /**
     * Passthroughs which correspond to specified input
     */
    OutputMap passthroughs{true, *this, "passthroughs", Output(*this, "", Output::Type::MSender, {{DatatypeEnum::Buffer, true}}, false)};

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
    // TODO add getters for other API
};

}  // namespace node
}  // namespace dai
