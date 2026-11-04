#pragma once

#include <cstdint>
#include <string>

#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Configuration of a ReplayVideo node. Setters record the values here.
 * The node checks them at pipeline start. Field order is part of the device contract.
 * Resolved fields are filled before the schema is sent so firmware can open the source.
 */
struct ReplayVideoProperties : PropertiesSerializable<Properties, ReplayVideoProperties> {
    std::string sourcePath;
    bool sourcePathSet = false;
    /// 0 = HOST, 1 = DEVICE. Matches dai::ReplaySourceLocation.
    std::int32_t sourceLocation = 0;
    bool sourceLocationSet = false;
    std::string metadataPath;
    bool metadataPathSet = false;
    /// dai::ImgFrame::Type stored as an integer. Meaningful when outFrameTypeSet is true.
    std::int32_t outFrameType = 2;
    bool outFrameTypeSet = false;
    std::int32_t width = 0;
    std::int32_t height = 0;
    bool sizeSet = false;
    float fps = 0.f;
    bool fpsSet = false;
    bool loop = true;
    bool loopSet = false;
    /// 0 = STREAM, 1 = PRE_DECODE. Matches dai::ReplayDecodeMode.
    std::int32_t decodeMode = 0;
    bool decodeModeSet = false;
    /// 0 = automatic, 1 = host, 2 = device.
    std::int32_t runOnHostMode = 0;
    bool persistentUpload = false;
    bool persistentUploadSet = false;
    std::string archiveEntry;
    bool archiveEntrySet = false;
    bool stopPipelineAtEnd = true;
    bool stopPipelineAtEndSet = false;
    std::string resolvedPath;
    std::string resolvedMetadataPath;
    std::string sourceHash;
    std::int32_t resolvedWidth = 0;
    std::int32_t resolvedHeight = 0;
    std::int32_t resolvedType = 2;
    float resolvedFps = 0.f;
    bool resolvedOnHost = true;

    ~ReplayVideoProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ReplayVideoProperties,
                      sourcePath,
                      sourcePathSet,
                      sourceLocation,
                      sourceLocationSet,
                      metadataPath,
                      metadataPathSet,
                      outFrameType,
                      outFrameTypeSet,
                      width,
                      height,
                      sizeSet,
                      fps,
                      fpsSet,
                      loop,
                      loopSet,
                      decodeMode,
                      decodeModeSet,
                      runOnHostMode,
                      persistentUpload,
                      persistentUploadSet,
                      archiveEntry,
                      archiveEntrySet,
                      stopPipelineAtEnd,
                      stopPipelineAtEndSet,
                      resolvedPath,
                      resolvedMetadataPath,
                      sourceHash,
                      resolvedWidth,
                      resolvedHeight,
                      resolvedType,
                      resolvedFps,
                      resolvedOnHost);

}  // namespace dai
