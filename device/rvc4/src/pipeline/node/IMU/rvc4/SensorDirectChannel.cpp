#include "SensorDirectChannel.hpp"

#include <algorithm>
#include <iterator>

#include "vendor/DeviceError.hpp"

namespace dai {
namespace gate {

namespace {

constexpr const char* UNIT = "Sensor direct channel";

static_assert(SensorSample::OTHER == DAI_SENSOR_MESSAGE_OTHER && SensorSample::SENSOR_EVENT == DAI_SENSOR_MESSAGE_EVENT,
              "SensorSample::Kind must have the values of DAI_SENSOR_MESSAGE_*");
static_assert(sizeof(SensorSample::data) == sizeof(dai_device_sensor_sample::data), "SensorSample::data must have the size of dai_device_sensor_sample::data");

/// As DAI_CHECK_DEVICE (utilities/ErrorMacros.hpp). That header includes pipeline/PipelineBuilder.hpp and with it the
/// GStreamer headers, which are not in the include directories of this target (pipeline.node.IMU).
void check(dai_status status, const char* call) {
    if(status != DAI_STATUS_OK) {
        device::throwDeviceError(status, UNIT, call, nullptr);
    }
}

uint32_t toDeviceChannelType(SensorChannelType type) {
    switch(type) {
        case SensorChannelType::GENERIC:
            return DAI_SENSOR_CHANNEL_GENERIC;
        case SensorChannelType::STRUCTURED_MUX:
        default:
            return DAI_SENSOR_CHANNEL_STRUCTURED_MUX;
    }
}

/// The stream refers to the text of config: config must stay valid during the call
dai_device_sensor_stream toDeviceStream(const SensorStreamConfig& config) {
    dai_device_sensor_stream stream{};
    stream.sensor_data_type = config.sensorDataType.c_str();
    stream.sample_rate_hz = config.sampleRateHz;
    stream.resampled = config.resampled ? 1 : 0;
    stream.calibrated = config.calibrated ? 1 : 0;
    stream.channel_type = toDeviceChannelType(config.channelType);
    return stream;
}

}  // namespace

SensorDirectChannel::SensorDirectChannel() : handle(nullptr, &dai_device_sensor_destroy) {
    device::installDeviceLog();
    dai_device_sensor_channel* channel = nullptr;
    check(dai_device_sensor_create(&channel), "dai_device_sensor_create");
    handle.reset(channel);
}

SensorDirectChannel::~SensorDirectChannel() = default;

SensorDirectChannel::SensorDirectChannel(SensorDirectChannel&&) noexcept = default;

SensorDirectChannel& SensorDirectChannel::operator=(SensorDirectChannel&&) noexcept = default;

void SensorDirectChannel::lookupSensor(const SensorStreamConfig& config) {
    const dai_device_sensor_stream stream = toDeviceStream(config);
    check(dai_device_sensor_lookup(handle.get(), &stream), "dai_device_sensor_lookup");
}

int SensorDirectChannel::sensorsFound() const {
    return dai_device_sensor_found(handle.get());
}

int SensorDirectChannel::queryAttributes(const SensorStreamConfig& config) {
    const dai_device_sensor_stream stream = toDeviceStream(config);
    int32_t result = 0;
    check(dai_device_sensor_query_attributes(handle.get(), &stream, &result), "dai_device_sensor_query_attributes");

    // The query adds the sample rates to the list of the vendor module: copy the list
    const int32_t* rates = nullptr;
    size_t count = 0;
    check(dai_device_sensor_sample_rates(handle.get(), &rates, &count), "dai_device_sensor_sample_rates");
    if(rates != nullptr) {
        sampleRates.assign(rates, rates + count);
    } else {
        sampleRates.clear();
    }
    return result;
}

const std::vector<int>& SensorDirectChannel::supportedSampleRates() const {
    return sampleRates;
}

void SensorDirectChannel::adjustSampleRate(uint32_t sampleRateHz, bool resampled) {
    check(dai_device_sensor_adjust_sample_rate(handle.get(), sampleRateHz, resampled ? 1 : 0), "dai_device_sensor_adjust_sample_rate");
}

char* SensorDirectChannel::startStreaming(const SensorStreamConfig& config) {
    const dai_device_sensor_stream stream = toDeviceStream(config);
    char* buffer = nullptr;
    // A channel that does not start gives DAI_STATUS_OK and no buffer (nullptr), as before the C interface
    check(dai_device_sensor_start(handle.get(), &stream, &buffer), "dai_device_sensor_start");
    return buffer;
}

void SensorDirectChannel::stopStreaming(bool calibrated, bool resampled) {
    check(dai_device_sensor_stop(handle.get(), calibrated ? 1 : 0, resampled ? 1 : 0), "dai_device_sensor_stop");
}

void SensorDirectChannel::cleanup() {
    dai_device_sensor_cleanup(handle.get());
}

char* SensorDirectChannel::sharedBuffer() const {
    return dai_device_sensor_shared_buffer(handle.get());
}

size_t SensorDirectChannel::sharedBufferSize() {
    return dai_device_sensor_shared_buffer_size();
}

SensorEventReader::SensorEventReader(const char* eventBuffer, size_t bufferSize) : buffer(eventBuffer), size(bufferSize) {}

SensorEventReader::Status SensorEventReader::next(SensorSample& sample) {
    // The vendor module writes the sample only when it reads an event, as before the C interface: start from the
    // values of the caller
    dai_device_sensor_sample deviceSample{};
    deviceSample.message_kind = sample.messageKind;
    deviceSample.timestamp = sample.timestamp;
    deviceSample.sequence = sample.sequence;
    std::copy(std::begin(sample.data), std::end(sample.data), std::begin(deviceSample.data));
    deviceSample.num_data = static_cast<uint32_t>(sample.numData);

    const size_t startOffset = offset;
    const int32_t result = dai_device_sensor_event_next(buffer, size, &offset, &deviceSample);

    // A sample that is not newer than the last one is a slot from the previous lap of the ring: stay on it until
    // the producer overwrites it
    if(result == DAI_SENSOR_EVENT_SAMPLE) {
        if(deviceSample.timestamp <= lastTimestamp) {
            offset = startOffset;
            return Status::NO_EVENT;
        }
        lastTimestamp = deviceSample.timestamp;
    }

    sample.messageKind = deviceSample.message_kind;
    sample.timestamp = deviceSample.timestamp;
    sample.sequence = deviceSample.sequence;
    std::copy(std::begin(deviceSample.data), std::end(deviceSample.data), std::begin(sample.data));
    sample.numData = deviceSample.num_data;

    switch(result) {
        case DAI_SENSOR_EVENT_SAMPLE:
            return Status::SAMPLE;
        case DAI_SENSOR_EVENT_SKIPPED:
            return Status::SKIPPED;
        case DAI_SENSOR_EVENT_NONE:
        default:
            return Status::NO_EVENT;
    }
}

}  // namespace gate
}  // namespace dai
