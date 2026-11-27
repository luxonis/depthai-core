#include "HalUtil.hpp"

#include <camera/VendorTagDescriptor.h>

#include "utilities/ErrorMacros.hpp"

using CameraMetadata = android::hardware::camera::common::V1_0::helper::CameraMetadata;
using VendorTagDescriptor = android::hardware::camera::common::V1_0::helper::VendorTagDescriptor;

uint32_t HalUtil::getVendorTag(const char* tag) {
    uint32_t tagId{0};
    auto res = CameraMetadata::getTagFromName(tag, VendorTagDescriptor::getGlobalVendorTagDescriptor().get(), &tagId);
    DAI_CHECK_V(res == 0, "Failed getting tag id for vendor tag: {}", tag);
    return tagId;
}

std::optional<uint32_t> HalUtil::tryGetVendorTag(const char* tag) {
    uint32_t tagId{0};
    auto res = CameraMetadata::getTagFromName(tag, VendorTagDescriptor::getGlobalVendorTagDescriptor().get(), &tagId);
    if(res != 0) {
        return std::nullopt;
    }
    return tagId;
}

bool HalUtil::getMetaFloat(const camera_metadata_t* meta, uint32_t tag, float& value) {
    DAI_CHECK_IN(meta);
    camera_metadata_ro_entry_t entry;
    auto suc = find_camera_metadata_ro_entry(meta, tag, &entry);
    if(suc == 0) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-union-access)
        value = entry.data.f[0];
        return true;
    }
    return false;
}

bool HalUtil::getMetaU8(const camera_metadata_t* meta, uint32_t tag, uint8_t& value) {
    DAI_CHECK_IN(meta);
    camera_metadata_ro_entry_t entry;
    auto suc = find_camera_metadata_ro_entry(meta, tag, &entry);
    if(suc == 0) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-union-access)
        value = entry.data.u8[0];
        return true;
    }
    return false;
}

bool HalUtil::getMetaInt64(const camera_metadata_t* meta, uint32_t tag, int64_t& value) {
    DAI_CHECK_IN(meta);
    camera_metadata_ro_entry_t entry;
    auto suc = find_camera_metadata_ro_entry(meta, tag, &entry);
    if(suc == 0) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-union-access)
        value = entry.data.i64[0];
        return true;
    }
    return false;
}

bool HalUtil::getMetaInt32(const camera_metadata_t* meta, uint32_t tag, int32_t& value) {
    DAI_CHECK_IN(meta);
    camera_metadata_ro_entry_t entry;
    auto suc = find_camera_metadata_ro_entry(meta, tag, &entry);
    if(suc == 0) {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-union-access)
        value = entry.data.i32[0];
        return true;
    }
    return false;
}

bool HalUtil::getMetaSensorReadOutput(const camera_metadata_t* meta, uint32_t tag, std::array<uint8_t, 2>& bytes) {
    DAI_CHECK_IN(meta);
    camera_metadata_ro_entry_t entry;
    auto suc = find_camera_metadata_ro_entry(meta, tag, &entry);
    if(suc != 0 || entry.type != TYPE_BYTE || entry.count < bytes.size()) {
        return false;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-type-union-access)
    bytes = {entry.data.u8[0], entry.data.u8[1]};
    return true;
}
