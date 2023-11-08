#pragma once

#include "depthai-shared/common/optional.hpp"
#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for NeuralNetwork such as blob path, ...
 */
struct NeuralNetworkProperties : PropertiesSerializable<Properties, NeuralNetworkProperties> {
    /**
     * Specify where the node should source the model
     */
    enum class ModelSource { BLOB, XML };
    ModelSource modelSource = ModelSource::BLOB;  // Default to blob source
    /**
     * Blob binary size in bytes
     */
    tl::optional<std::uint32_t> blobSize;
    /**
     * Uri which points to blob
     */
    std::string blobUri;
    /**
     * Uri which points to the xml model description
     */
    std::string xmlUri;
    /**
     * Uri which points to the bin model description
     */
    std::string binUri;
    /**
     * Number of available output tensors in pool
     */
    std::uint32_t numFrames = 8;
    /**
     * Number of threads to create for running inference. 0 = auto
     */
    std::uint32_t numThreads = 0;
    /**
     * Number of NCE (Neural Compute Engine) per inference thread. 0 = auto
     */
    std::uint32_t numNCEPerThread = 0;
    /**
     * Number of Shaves per inference thread. 0 = auto
     */
    std::uint32_t numShavesPerThread = 0;
    /**
     * Specify which backend is used. "" = auto
     */
    std::string backend;
    /**
     * Specify backend properties
     */
    std::map<std::string, std::string> backendProperties;
};

DEPTHAI_SERIALIZE_EXT(NeuralNetworkProperties,
                      modelSource,
                      blobSize,
                      blobUri,
                      xmlUri,
                      binUri,
                      numFrames,
                      numThreads,
                      numNCEPerThread,
                      numShavesPerThread,
                      backend,
                      backendProperties);

}  // namespace dai
