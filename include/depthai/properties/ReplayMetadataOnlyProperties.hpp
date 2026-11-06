#pragma once

#include <cstdint>
#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Configuration of a ReplayMetadataOnly node. Setters record the values here.
 * The node checks them at pipeline start. Field order is part of the device contract.
 */
struct ReplayMetadataOnlyProperties : PropertiesSerializable<Properties, ReplayMetadataOnlyProperties> {
    std::string sourcePath;
    bool sourcePathSet = false;
    /// 0 = HOST, 1 = DEVICE. Matches dai::ReplaySourceLocation.
    std::int32_t sourceLocation = 0;
    bool sourceLocationSet = false;
    float fps = 0.f;
    bool fpsSet = false;
    bool loop = true;
    bool loopSet = false;
    /// 0 = automatic, 1 = host, 2 = device.
    std::int32_t runOnHostMode = 0;
    bool persistentUpload = false;
    bool persistentUploadSet = false;
    std::string archiveEntry;
    bool archiveEntrySet = false;
    bool stopPipelineAtEnd = true;
    bool stopPipelineAtEndSet = false;
    std::string resolvedPath;
    std::string sourceHash;
    bool resolvedOnHost = true;

    ~ReplayMetadataOnlyProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ReplayMetadataOnlyProperties,
                      sourcePath,
                      sourcePathSet,
                      sourceLocation,
                      sourceLocationSet,
                      fps,
                      fpsSet,
                      loop,
                      loopSet,
                      runOnHostMode,
                      persistentUpload,
                      persistentUploadSet,
                      archiveEntry,
                      archiveEntrySet,
                      stopPipelineAtEnd,
                      stopPipelineAtEndSet,
                      resolvedPath,
                      sourceHash,
                      resolvedOnHost);

}  // namespace dai
