#pragma once

#include <sys/types.h>

#include "depthai/common/ProcessorType.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Sync.
 */
struct SyncProperties : PropertiesSerializable<Properties, SyncProperties> {
    enum class TimestampSource : uint8_t { DEFAULT, DEVICE, HOST, SYSTEM };

    /**
     * The maximal interval the messages can be apart in nanoseconds.
     */
    int64_t syncThresholdNs = 10e6;

    /**
     * The number of syncing attempts before fail (num of replaced messages).
     */
    int32_t syncAttempts = -1;

    /**
     * Which processor should execute the node.
     */
    ProcessorType processor = ProcessorType::LEON_CSS;

    /**
     * Which timestamp to use for synchronization. On device the default is DEVICE, on host the default is HOST
     */
    TimestampSource timestampSource = TimestampSource::DEFAULT;

    /** Compare individual IMU reports and image exposure-middle timestamps instead of message headers. */
    bool syncOnIndividualReports = false;

    ~SyncProperties() override;
};

inline void to_json(nlohmann::json& json, const SyncProperties& properties) {
    json = {{"syncThresholdNs", properties.syncThresholdNs},
            {"syncAttempts", properties.syncAttempts},
            {"processor", properties.processor},
            {"timestampSource", properties.timestampSource},
            {"syncOnIndividualReports", properties.syncOnIndividualReports}};
}

inline void from_json(const nlohmann::json& json, SyncProperties& properties) {
    json.at("syncThresholdNs").get_to(properties.syncThresholdNs);
    json.at("syncAttempts").get_to(properties.syncAttempts);
    json.at("processor").get_to(properties.processor);
    json.at("timestampSource").get_to(properties.timestampSource);
    properties.syncOnIndividualReports = json.value("syncOnIndividualReports", false);
}

}  // namespace dai

namespace nop {
// libnop structures normally require an exact field count. Keep the legacy four-field
// encoding when disabled; accept either version when reading. Opt-in requires updated firmware.
template <>
struct Encoding<dai::SyncProperties> : EncodingIO<dai::SyncProperties> {
    using Type = dai::SyncProperties;
    static constexpr EncodingByte Prefix(const Type&) {
        return EncodingByte::Structure;
    }
    static constexpr bool Match(EncodingByte prefix) {
        return prefix == EncodingByte::Structure;
    }
    static std::size_t Size(const Type& value) {
        return BaseEncodingSize(Prefix(value)) + Encoding<SizeType>::Size(value.syncOnIndividualReports ? 5 : 4)
               + Encoding<int64_t>::Size(value.syncThresholdNs) + Encoding<int32_t>::Size(value.syncAttempts)
               + Encoding<dai::ProcessorType>::Size(value.processor) + Encoding<Type::TimestampSource>::Size(value.timestampSource)
               + (value.syncOnIndividualReports ? Encoding<bool>::Size(true) : 0);
    }
    template <typename Writer>
    static Status<void> WritePayload(EncodingByte, const Type& value, Writer* writer) {
        auto status = Encoding<SizeType>::Write(value.syncOnIndividualReports ? 5 : 4, writer);
        if(!status) return status;
        status = Encoding<int64_t>::Write(value.syncThresholdNs, writer);
        if(!status) return status;
        status = Encoding<int32_t>::Write(value.syncAttempts, writer);
        if(!status) return status;
        status = Encoding<dai::ProcessorType>::Write(value.processor, writer);
        if(!status) return status;
        status = Encoding<Type::TimestampSource>::Write(value.timestampSource, writer);
        if(!status || !value.syncOnIndividualReports) return status;
        return Encoding<bool>::Write(value.syncOnIndividualReports, writer);
    }
    template <typename Reader>
    static Status<void> ReadPayload(EncodingByte, Type* value, Reader* reader) {
        SizeType count = 0;
        auto status = Encoding<SizeType>::Read(&count, reader);
        if(!status) return status;
        if(count != 4 && count != 5) return ErrorStatus::InvalidMemberCount;
        value->syncOnIndividualReports = false;
        status = Encoding<int64_t>::Read(&value->syncThresholdNs, reader);
        if(!status) return status;
        status = Encoding<int32_t>::Read(&value->syncAttempts, reader);
        if(!status) return status;
        status = Encoding<dai::ProcessorType>::Read(&value->processor, reader);
        if(!status) return status;
        status = Encoding<Type::TimestampSource>::Read(&value->timestampSource, reader);
        if(!status || count == 4) return status;
        return Encoding<bool>::Read(&value->syncOnIndividualReports, reader);
    }
};
}  // namespace nop
