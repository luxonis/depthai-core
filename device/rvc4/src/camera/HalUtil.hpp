#pragma once

#include <camera/CameraMetadata.h>

#include <cstdint>
#include <optional>

#include "vendor/dai_device_camera.h"

class HalUtil {
   public:
    static uint32_t getVendorTag(const char* tag);
    static std::optional<uint32_t> tryGetVendorTag(const char* tag);
    /// The tag id of a Luxonis tag (DAI_CAMERA_TAG_* of vendor/dai_device_camera.h): its name comes from the vendor module
    static uint32_t getCameraTag(uint32_t daiTag);
    static std::optional<uint32_t> tryGetCameraTag(uint32_t daiTag);
    static bool getMetaFloat(const camera_metadata_t* meta, uint32_t tag, float& value);
    static bool getMetaU8(const camera_metadata_t* meta, uint32_t tag, uint8_t& value);
    static bool getMetaInt64(const camera_metadata_t* meta, uint32_t tag, int64_t& value);
    static bool getMetaInt32(const camera_metadata_t* meta, uint32_t tag, int32_t& value);
    static bool getMetaSensorTemperature(const camera_metadata_t* meta, uint32_t tag, float& value);
};
