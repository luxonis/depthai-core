#pragma once

#include "depthai/common/EepromData.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

namespace dai {

/**
 * Specify properties which apply for whole pipeline
 */
struct GlobalProperties : PropertiesSerializable<Properties, GlobalProperties> {
    constexpr static uint32_t SIPP_BUFFER_DEFAULT_SIZE = 18 * 1024;
    constexpr static uint32_t SIPP_DMA_BUFFER_DEFAULT_SIZE = 16 * 1024;

    /**
     * Set frequency of Leon OS - Increasing can improve performance, at the cost of higher power
     * draw
     */
    [[deprecated("Set leonCssFrequencyHz on DeviceProperties instead")]]
    double leonCssFrequencyHz = 700 * 1000 * 1000;

    /**
     * Set frequency of Leon RT - Increasing can improve performance, at the cost of higher power
     * draw
     */
    [[deprecated("Set leonMssFrequencyHz on DeviceProperties instead")]]
    double leonMssFrequencyHz = 700 * 1000 * 1000;

    std::optional<std::string> pipelineName;
    std::optional<std::string> pipelineVersion;

    /**
     * Calibration data sent through pipeline
     */
    [[deprecated("Set calibData on DeviceProperties instead")]]
    std::optional<dai::EepromData> calibData;

    /**
     * Unique identifier for the eeprom data in the pipeline.
     */
    [[deprecated("Set eepromId on DeviceProperties instead")]]
    uint32_t eepromId = 0;

    /**
     * Camera tuning blob size in bytes
     */
    [[deprecated("Set cameraTuningBlobSize on DeviceProperties instead")]]
    std::optional<std::uint32_t> cameraTuningBlobSize;
    /**
     * Uri which points to camera tuning blob
     */
    [[deprecated("Set cameraTuningBlobUri on DeviceProperties instead")]]
    std::string cameraTuningBlobUri;

    /**
     * Socket specific camera tuning blob size in bytes
     */
    [[deprecated("Set cameraSocketTuningBlobSize on DeviceProperties instead")]]
    std::unordered_map<CameraBoardSocket, std::uint32_t> cameraSocketTuningBlobSize;

    /**
     * Socket specific camera tuning blob uri
     */
    [[deprecated("Set cameraSocketTuningBlobUri on DeviceProperties instead")]]
    std::unordered_map<CameraBoardSocket, std::string> cameraSocketTuningBlobUri;

    /**
     * Chunk size for splitting device-sent XLink packets, in bytes. A larger value could
     * increase performance, with 0 disabling chunking. A negative value won't modify the
     * device defaults - configured per protocol, currently 64*1024 for both USB and Ethernet.
     */
    [[deprecated("Set xlinkChunkSize on DeviceProperties instead")]]
    int32_t xlinkChunkSize = -1;

    /**
     * SIPP (Signal Image Processing Pipeline) internal memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * By default memory is allocated in high speed CMX memory. Setting to 0 will allocate in DDR 256 kilobytes.
     * Units are bytes.
     */
    [[deprecated("Set sippBufferSize on DeviceProperties instead")]]
    uint32_t sippBufferSize = SIPP_BUFFER_DEFAULT_SIZE;

    /**
     * SIPP (Signal Image Processing Pipeline) internal DMA memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * Memory is allocated in high speed CMX memory
     * Units are bytes.
     */
    [[deprecated("Set sippDmaBufferSize on DeviceProperties instead")]]
    uint32_t sippDmaBufferSize = SIPP_DMA_BUFFER_DEFAULT_SIZE;

    ~GlobalProperties() override;

    GlobalProperties& setFrom(const GlobalProperties& other) {
        DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING

        leonCssFrequencyHz = other.leonCssFrequencyHz;
        leonMssFrequencyHz = other.leonMssFrequencyHz;
        if(other.calibData) {
            calibData = other.calibData;
            eepromId = other.eepromId;
        }
        if(other.cameraTuningBlobSize) cameraTuningBlobSize = other.cameraTuningBlobSize;
        if(other.pipelineName) pipelineName = other.pipelineName;
        if(other.pipelineVersion) pipelineVersion = other.pipelineVersion;
        cameraTuningBlobUri = other.cameraTuningBlobUri;
        cameraSocketTuningBlobSize = other.cameraSocketTuningBlobSize;
        cameraSocketTuningBlobUri = other.cameraSocketTuningBlobUri;
        xlinkChunkSize = other.xlinkChunkSize;
        sippBufferSize = other.sippBufferSize;
        sippDmaBufferSize = other.sippDmaBufferSize;

        DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING

        return *this;
    }
};

/**
 * Specify properties which apply for a device
 */
struct DeviceProperties : PropertiesSerializable<Properties, DeviceProperties> {
    constexpr static uint32_t SIPP_BUFFER_DEFAULT_SIZE = 18 * 1024;
    constexpr static uint32_t SIPP_DMA_BUFFER_DEFAULT_SIZE = 16 * 1024;

    /**
     * Set frequency of Leon OS - Increasing can improve performance, at the cost of higher power
     * draw
     */
    double leonCssFrequencyHz = 700 * 1000 * 1000;
    /**
     * Set frequency of Leon RT - Increasing can improve performance, at the cost of higher power
     * draw
     */
    double leonMssFrequencyHz = 700 * 1000 * 1000;
    /**
     * Calibration data sent through pipeline
     */

    std::optional<dai::EepromData> calibData;

    /**
     * Unique identifier for the eeprom data in the pipeline.
     */
    uint32_t eepromId = 0;

    /**
     * Camera tuning blob size in bytes
     */
    std::optional<std::uint32_t> cameraTuningBlobSize;
    /**
     * Uri which points to camera tuning blob
     */
    std::string cameraTuningBlobUri;

    /**
     * Socket specific camera tuning blob size in bytes
     */
    std::unordered_map<CameraBoardSocket, std::uint32_t> cameraSocketTuningBlobSize;
    /**
     * Socket specific camera tuning blob uri
     */
    std::unordered_map<CameraBoardSocket, std::string> cameraSocketTuningBlobUri;

    /**
     * Chunk size for splitting device-sent XLink packets, in bytes. A larger value could
     * increase performance, with 0 disabling chunking. A negative value won't modify the
     * device defaults - configured per protocol, currently 64*1024 for both USB and Ethernet.
     */
    int32_t xlinkChunkSize = -1;

    /**
     * SIPP (Signal Image Processing Pipeline) internal memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * By default memory is allocated in high speed CMX memory. Setting to 0 will allocate in DDR 256 kilobytes.
     * Units are bytes.
     */
    uint32_t sippBufferSize = SIPP_BUFFER_DEFAULT_SIZE;
    /**
     * SIPP (Signal Image Processing Pipeline) internal DMA memory pool.
     * SIPP is a framework used to schedule HW filters, e.g. ISP, Warp, Median filter etc.
     * Changing the size of this pool is meant for advanced use cases, pushing the limits of the HW.
     * Memory is allocated in high speed CMX memory
     * Units are bytes.
     */
    uint32_t sippDmaBufferSize = SIPP_DMA_BUFFER_DEFAULT_SIZE;

    ~DeviceProperties() override;

    DeviceProperties& setFrom(const DeviceProperties& other) {
        leonCssFrequencyHz = other.leonCssFrequencyHz;
        leonMssFrequencyHz = other.leonMssFrequencyHz;
        if(other.calibData) {
            calibData = other.calibData;
            eepromId = other.eepromId;
        }
        if(other.cameraTuningBlobSize) cameraTuningBlobSize = other.cameraTuningBlobSize;
        cameraTuningBlobUri = other.cameraTuningBlobUri;
        cameraSocketTuningBlobSize = other.cameraSocketTuningBlobSize;
        cameraSocketTuningBlobUri = other.cameraSocketTuningBlobUri;
        xlinkChunkSize = other.xlinkChunkSize;
        sippBufferSize = other.sippBufferSize;
        sippDmaBufferSize = other.sippDmaBufferSize;
        return *this;
    }
};

DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING

DEPTHAI_SERIALIZE_EXT(GlobalProperties,
                      leonCssFrequencyHz,
                      leonMssFrequencyHz,
                      pipelineName,
                      pipelineVersion,
                      cameraTuningBlobSize,
                      cameraTuningBlobUri,
                      cameraSocketTuningBlobSize,
                      cameraSocketTuningBlobUri,
                      calibData,
                      eepromId,
                      xlinkChunkSize,
                      sippBufferSize,
                      sippDmaBufferSize);

DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING

DEPTHAI_SERIALIZE_EXT(DeviceProperties,
                      leonCssFrequencyHz,
                      leonMssFrequencyHz,
                      cameraTuningBlobSize,
                      cameraTuningBlobUri,
                      cameraSocketTuningBlobSize,
                      cameraSocketTuningBlobUri,
                      calibData,
                      eepromId,
                      xlinkChunkSize,
                      sippBufferSize,
                      sippDmaBufferSize);

}  // namespace dai
