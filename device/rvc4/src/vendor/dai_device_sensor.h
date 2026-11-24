/*
 * C interface of the sensor hub direct channel of the vendor module (IMU). See dai_device_common.h for the rules.
 */
#ifndef DAI_DEVICE_SENSOR_H
#define DAI_DEVICE_SENSOR_H

#include "dai_device_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Data layout of a direct channel (the shared memory that the sensor hub writes the samples into) */
#define DAI_SENSOR_CHANNEL_STRUCTURED_MUX 0
#define DAI_SENSOR_CHANNEL_GENERIC 1

/* One requested sensor stream */
typedef struct dai_device_sensor_stream {
    const char* sensor_data_type; /* for example "accel"; the vendor module copies what it keeps */
    uint32_t sample_rate_hz;
    uint8_t resampled;
    uint8_t calibrated;
    uint32_t channel_type; /* DAI_SENSOR_CHANNEL_* */
} dai_device_sensor_stream;

/* Direct (shared memory) channel to one sensor of the sensor hub */
typedef struct dai_device_sensor_channel dai_device_sensor_channel;

dai_status dai_device_sensor_create(dai_device_sensor_channel** out);
void dai_device_sensor_destroy(dai_device_sensor_channel* h);
dai_status dai_device_sensor_lookup(dai_device_sensor_channel* h, const dai_device_sensor_stream* stream);
/* Number of sensors that the last lookup found */
int32_t dai_device_sensor_found(const dai_device_sensor_channel* h);
/* result: the result of the attribute query of the sensor hub client */
dai_status dai_device_sensor_query_attributes(dai_device_sensor_channel* h, const dai_device_sensor_stream* stream, int32_t* result);
/* The sample rates of the last attribute query; valid until the next query */
dai_status dai_device_sensor_sample_rates(const dai_device_sensor_channel* h, const int32_t** rates, size_t* count);
dai_status dai_device_sensor_adjust_sample_rate(dai_device_sensor_channel* h, uint32_t sample_rate_hz, uint8_t resampled);
/* buffer: the shared memory of the channel (dai_device_sensor_shared_buffer_size() bytes). When the sensor hub client
 * gives no buffer, the call returns DAI_STATUS_OK and *buffer is NULL. */
dai_status dai_device_sensor_start(dai_device_sensor_channel* h, const dai_device_sensor_stream* stream, char** buffer);
dai_status dai_device_sensor_stop(dai_device_sensor_channel* h, uint8_t calibrated, uint8_t resampled);
void dai_device_sensor_cleanup(dai_device_sensor_channel* h);
char* dai_device_sensor_shared_buffer(const dai_device_sensor_channel* h);
size_t dai_device_sensor_shared_buffer_size(void);

/* One decoded sample of a GENERIC channel */
#define DAI_SENSOR_MESSAGE_OTHER 0
#define DAI_SENSOR_MESSAGE_EVENT 1
typedef struct dai_device_sensor_sample {
    int32_t message_kind; /* DAI_SENSOR_MESSAGE_* */
    uint64_t timestamp;
    uint32_t sequence;
    float data[4];
    uint32_t num_data;
} dai_device_sensor_sample;

/* Results of dai_device_sensor_event_next */
#define DAI_SENSOR_EVENT_NONE 0    /* no new event: wait and read again */
#define DAI_SENSOR_EVENT_SKIPPED 1 /* an event that is not a sample was read */
#define DAI_SENSOR_EVENT_SAMPLE 2  /* *sample has a sample */

/* Reads the next event of a GENERIC channel from its circular shared buffer. The caller keeps the read position
 * (*offset, 0 at the start). The vendor module keeps no state. */
int32_t dai_device_sensor_event_next(const char* buffer, size_t size, size_t* offset, dai_device_sensor_sample* sample);

#ifdef __cplusplus
}
#endif

#endif /* DAI_DEVICE_SENSOR_H */
