#pragma once

#include <map>
#include <string>
#include <vector>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"
namespace dai {

/**
 * Specify properties for NeuralNetwork such as blob path, ...
 */
struct NeuralNetworkProperties : PropertiesSerializable<Properties, NeuralNetworkProperties> {
    /**
     * Specify where the node should source the model
     */
    enum class ModelSource { BLOB, CUSTOM_MODEL };

    /**
     * SNPE User Defined Operation (UDO) package from an NNArchive. RVC4 only.
     */
    struct UdoPackage {
        /** Archive path of the AArch64 registration library. */
        std::string registration;
        /**
         * Archive paths of the libraries declared for the "dsp" runtime: Hexagon implementations and
         * their AArch64 "<name>_AltPrep.so" builds, which SNPE uses to prepare the network on the CPU.
         */
        std::vector<std::string> dspImplementations;
    };

    ModelSource modelSource = ModelSource::BLOB;  // Default to blob source
    /**
     * Blob binary size in bytes
     */
    std::optional<std::uint32_t> blobSize;

    /**
     * Uri which points to blob
     */
    std::string blobUri;

    /**
     * Uri which points to the model description
     */
    std::string modelUri;

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
    /**
     * Device model from device model zoo
     */
    std::optional<DeviceModelZoo> deviceModel = std::nullopt;

    /**
     * SNPE UDO packages that the device registers before it builds the network
     */
    std::vector<UdoPackage> udoPackages = {};
    /**
     * Pipeline asset URI of each UDO library, keyed by its archive path
     */
    std::map<std::string, std::string> udoLibraryUris = {};

    ~NeuralNetworkProperties() override;
};
DEPTHAI_SERIALIZE_EXT_DECL(NeuralNetworkProperties::UdoPackage, registration, dspImplementations);
DEPTHAI_SERIALIZE_EXT_DECL(NeuralNetworkProperties,
                           modelSource,
                           blobSize,
                           blobUri,
                           modelUri,
                           numFrames,
                           numThreads,
                           numNCEPerThread,
                           numShavesPerThread,
                           backend,
                           backendProperties,
                           deviceModel,
                           udoPackages,
                           udoLibraryUris);

}  // namespace dai

extern template struct dai::PropertiesSerializable<dai::Properties, dai::NeuralNetworkProperties>;
