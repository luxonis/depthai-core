#pragma once

#include <cstdint>
#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Configuration of a RecordVideo node. Setters record the values here.
 * The node checks them at pipeline start. Field order is part of the device contract.
 */
struct RecordVideoProperties : PropertiesSerializable<Properties, RecordVideoProperties> {
    /// 0 = VIDEO, 1 = RAW_FRAMES. Matches dai::RecordFormat.
    std::int32_t format = 0;
    std::string recordingName;
    bool recordingNameSet = false;
    std::string outputDirectory;
    bool outputDirectorySet = false;
    std::string streamName = "frames";
    bool streamNameSet = false;
    float fps = 30.f;
    bool fpsSet = false;
    /// dai::RecordConfig::CompressionLevel stored as an integer.
    std::int32_t compressionLevel = 3;
    bool waitForTrigger = false;
    std::int32_t preTriggerFrames = 60;
    std::uint64_t maxPreTriggerBytes = 268435456ull;
    bool transferToHost = true;
    bool transferToHostSet = false;
    bool keepOnDevice = false;
    bool keepOnDeviceSet = false;
    /// 0 = automatic, 1 = host, 2 = device.
    std::int32_t runOnHostMode = 0;
    std::string recordVideoFile;
    bool recordVideoFileSet = false;
    std::string recordMetadataFile;
    bool recordMetadataFileSet = false;

    ~RecordVideoProperties() override;
};

DEPTHAI_SERIALIZE_EXT(RecordVideoProperties,
                      format,
                      recordingName,
                      recordingNameSet,
                      outputDirectory,
                      outputDirectorySet,
                      streamName,
                      streamNameSet,
                      fps,
                      fpsSet,
                      compressionLevel,
                      waitForTrigger,
                      preTriggerFrames,
                      maxPreTriggerBytes,
                      transferToHost,
                      transferToHostSet,
                      keepOnDevice,
                      keepOnDeviceSet,
                      runOnHostMode,
                      recordVideoFile,
                      recordVideoFileSet,
                      recordMetadataFile,
                      recordMetadataFileSet);

}  // namespace dai
