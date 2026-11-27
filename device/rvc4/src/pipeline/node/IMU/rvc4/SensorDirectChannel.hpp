#pragma once

// std
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// C interface of the vendor module
#include "vendor/dai_device_sensor.h"

namespace dai {
namespace gate {

/**
 * Data layout of a sensor direct channel (the shared memory the sensor hub streams samples into).
 */
enum class SensorChannelType {
    /// Fixed-size Android `sensors_event_t` records, one per sample (accelerometer, gyroscope, magnetometer).
    STRUCTURED_MUX = 0,
    /// Variable-length sensor events, decoded with SensorEventReader (rotation vector streams).
    GENERIC = 1,
};

/**
 * One requested sensor stream.
 */
struct SensorStreamConfig {
    /// Sensor data type identifier: "accel", "gyro", "mag", "rotv", "game_rv".
    std::string sensorDataType;

    /// Requested sample rate in Hz.
    uint32_t sampleRateHz = 0;

    /// Request a resampled stream.
    bool resampled = false;

    /// Request calibrated (true) or raw (false) samples.
    bool calibrated = false;

    /// Data layout of the channel the sensor streams into.
    SensorChannelType channelType = SensorChannelType::STRUCTURED_MUX;
};

/**
 * Direct (shared memory) channel to one sensor of the sensor hub.
 *
 * The object is movable and non-copyable;
 * a moved-from object must not be used.
 */
class SensorDirectChannel {
   public:
    SensorDirectChannel();
    ~SensorDirectChannel();
    SensorDirectChannel(SensorDirectChannel&&) noexcept;
    SensorDirectChannel& operator=(SensorDirectChannel&&) noexcept;
    SensorDirectChannel(const SensorDirectChannel&) = delete;
    SensorDirectChannel& operator=(const SensorDirectChannel&) = delete;

    /// Looks the sensor described by `config` up on the sensor hub. Check sensorsFound() afterwards.
    void lookupSensor(const SensorStreamConfig& config);

    /// Number of sensors the last lookupSensor() call found.
    int sensorsFound() const;

    /// Queries the sensor attributes (fills supportedSampleRates()). Returns 0 on success.
    int queryAttributes(const SensorStreamConfig& config);

    /// Sample rates the sensor supports, in Hz and ascending, as reported by queryAttributes().
    const std::vector<int>& supportedSampleRates() const;

    /// Selects the sample rate the sensor will stream at for the requested rate (depends on resampling).
    void adjustSampleRate(uint32_t sampleRateHz, bool resampled);

    /// Starts streaming. Returns the shared buffer the sensor hub writes into, or nullptr on failure.
    char* startStreaming(const SensorStreamConfig& config);

    /// Stops streaming and releases the channel.
    void stopStreaming(bool calibrated, bool resampled);

    /// Releases the connection to the sensor hub.
    void cleanup();

    /// Shared buffer of the running stream (nullptr when not streaming).
    char* sharedBuffer() const;

    /// Size in bytes of the (circular) shared buffer of every channel.
    static size_t sharedBufferSize();

   private:
    std::unique_ptr<dai_device_sensor_channel, void (*)(dai_device_sensor_channel*)> handle;
    /// Copy of the sample rates of the last queryAttributes()
    std::vector<int> sampleRates;
};

/**
 * One decoded sample of a GENERIC channel.
 */
struct SensorSample {
    enum Kind : int32_t {
        /// An event that carries no sample (configuration, latency statistics, ...).
        OTHER = 0,
        /// A sensor sample: `data` holds `numData` values.
        SENSOR_EVENT = 1,
    };

    /// Kind of the decoded event.
    int32_t messageKind = OTHER;

    /// Sensor hub timestamp of the sample in nanoseconds (sensor hub / QTimer domain).
    uint64_t timestamp = 0;

    /// Sample counter; 0 when the channel does not carry one.
    uint32_t sequence = 0;

    /// Sample values; the first `numData` entries are valid, the rest are zero.
    float data[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    size_t numData = 0;
};

/**
 * Reads the variable-length events of a GENERIC channel out of its circular shared buffer.
 *
 * The reader owns only the read position; the caller owns the loop and decides when to wait
 * (Status::NO_EVENT) and when to stop.
 */
class SensorEventReader {
   public:
    enum class Status {
        /// No new event at the read position; the caller should wait and call next() again.
        NO_EVENT,
        /// The read position moved on (wrap-around or an event without a sample); call next() again.
        SKIPPED,
        /// `sample` holds a decoded sensor sample.
        SAMPLE,
    };

    SensorEventReader(const char* buffer, size_t size);

    /// Decodes the event at the read position and moves past it.
    Status next(SensorSample& sample);

   private:
    const char* buffer;
    size_t size;
    size_t offset = 0;
};

}  // namespace gate
}  // namespace dai
