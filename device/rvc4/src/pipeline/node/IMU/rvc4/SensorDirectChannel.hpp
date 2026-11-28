#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "vendor/dai_device_sensor.h"

namespace dai {
namespace gate {

enum class SensorChannelType {
    STRUCTURED_MUX = 0,  // Fixed-size Android sensors_event_t records, one per sample
    GENERIC = 1,         // Variable-length events, decoded with SensorEventReader
};

struct SensorStreamConfig {
    std::string sensorDataType;  // One of "accel", "gyro", "mag", "rotv", "game_rv"

    uint32_t sampleRateHz = 0;

    bool resampled = false;

    bool calibrated = false;

    SensorChannelType channelType = SensorChannelType::STRUCTURED_MUX;
};

class SensorDirectChannel {  // A moved-from object must not be used
   public:
    SensorDirectChannel();
    ~SensorDirectChannel();
    SensorDirectChannel(SensorDirectChannel&&) noexcept;
    SensorDirectChannel& operator=(SensorDirectChannel&&) noexcept;
    SensorDirectChannel(const SensorDirectChannel&) = delete;
    SensorDirectChannel& operator=(const SensorDirectChannel&) = delete;

    void lookupSensor(const SensorStreamConfig& config);

    int sensorsFound() const;

    int queryAttributes(const SensorStreamConfig& config);  // Returns 0 on success; fills supportedSampleRates()

    const std::vector<int>& supportedSampleRates() const;  // In Hz, ascending; filled by queryAttributes()

    void adjustSampleRate(uint32_t sampleRateHz, bool resampled);

    char* startStreaming(const SensorStreamConfig& config);  // Returns the shared buffer the sensor hub writes into, or nullptr on failure

    void stopStreaming(bool calibrated, bool resampled);

    void cleanup();

    char* sharedBuffer() const;  // nullptr when not streaming

    static size_t sharedBufferSize();

   private:
    std::unique_ptr<dai_device_sensor_channel, void (*)(dai_device_sensor_channel*)> handle;
    std::vector<int> sampleRates;
};

struct SensorSample {
    enum Kind : int32_t {
        OTHER = 0,
        SENSOR_EVENT = 1,
    };

    int32_t messageKind = OTHER;

    uint64_t timestamp = 0;  // Nanoseconds, sensor hub (QTimer) clock domain

    uint32_t sequence = 0;  // 0 when the channel does not carry a counter

    float data[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // The first numData entries are valid, the rest are zero
    size_t numData = 0;
};

class SensorEventReader {
   public:
    enum class Status {
        NO_EVENT,  // The caller should wait, then call next() again
        SKIPPED,   // Wrap-around or an event without a sample: call next() again at once
        SAMPLE,
    };

    SensorEventReader(const char* buffer, size_t size);

    Status next(SensorSample& sample);

   private:
    const char* buffer;
    size_t size;
    size_t offset = 0;
};

}  // namespace gate
}  // namespace dai
