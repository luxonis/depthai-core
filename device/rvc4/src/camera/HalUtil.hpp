#pragma once

#include <camera/CameraMetadata.h>

#include <array>
#include <cstdint>
#include <optional>

class HalUtil {
   public:
    static uint32_t getVendorTag(const char* tag);
    static std::optional<uint32_t> tryGetVendorTag(const char* tag);
    static bool getMetaFloat(const camera_metadata_t* meta, uint32_t tag, float& value);
    static bool getMetaU8(const camera_metadata_t* meta, uint32_t tag, uint8_t& value);
    static bool getMetaInt64(const camera_metadata_t* meta, uint32_t tag, int64_t& value);
    static bool getMetaInt32(const camera_metadata_t* meta, uint32_t tag, int32_t& value);
    static bool getMetaSensorReadOutput(const camera_metadata_t* meta, uint32_t tag, std::array<uint8_t, 2>& bytes);
};
