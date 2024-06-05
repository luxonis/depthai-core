#pragma once

#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace dai {
namespace utility {

enum class RecordType { Other = 0, Video = 1, Imu = 2 };

struct VersionSchema {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VersionSchema, major, minor, patch)

struct TimestampSchema {
    uint64_t seconds;
    uint32_t nanoseconds;

    void set(std::chrono::nanoseconds time) {
        seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
        nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time).count() % 1000000000;
    }

    std::chrono::nanoseconds get() const {
        return std::chrono::seconds(seconds) + std::chrono::nanoseconds(nanoseconds);
    }
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TimestampSchema, seconds, nanoseconds)

// Make sure these structs are in sync with the JSON schema
struct DefaultRecordSchema {
    VersionSchema version{0, 0, 1};
    RecordType type = RecordType::Other;
    TimestampSchema timestamp;
    uint64_t sequenceNumber;
    std::vector<uint8_t> data;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DefaultRecordSchema, version, type, timestamp, sequenceNumber, data)

struct ImuOrientationSchema {
    TimestampSchema timestamp;
    uint64_t sequenceNumber;

    float x;
    float y;
    float z;
    float w;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuOrientationSchema, x, y, z, w)

struct IMUAccelerationSchema {
    TimestampSchema timestamp;
    uint64_t sequenceNumber;

    float x;
    float y;
    float z;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IMUAccelerationSchema, x, y, z)

struct ImuPacketSchema {
    ImuOrientationSchema orientation;
    IMUAccelerationSchema acceleration;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuPacketSchema, orientation, acceleration)

struct ImuRecordSchema {
    VersionSchema version{0, 0, 1};
    RecordType type = RecordType::Imu;
    std::vector<ImuPacketSchema> packets;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuRecordSchema, version, type, packets)

struct VideoCameraSettingsSchema {
    int32_t exposure;
    int32_t sensitivity;
    int32_t lensPosition;
    int32_t wbColorTemp;
    float lensPositionRaw;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VideoCameraSettingsSchema, exposure, sensitivity, lensPosition, wbColorTemp, lensPositionRaw)

struct VideoRecordSchema {
    VersionSchema version{0, 0, 1};
    RecordType type = RecordType::Video;
    TimestampSchema timestamp;
    uint64_t sequenceNumber;
    uint64_t instanceNumber;
    uint32_t width;
    uint32_t height;
    VideoCameraSettingsSchema cameraSettings;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VideoRecordSchema, version, type, timestamp, sequenceNumber, instanceNumber, width, height, cameraSettings)

constexpr const char* DEFAULT_SHEMA = R"(
{
    "title": "Default record meta",
    "description": "Metadata for an unspecified record stream",
    "type": "object",
    "properties": {
        "version": {
            "type": "object",
            "description": "Metadata version",
            "properties": {
                "major": {
                    "type": "integer",
                    "description": "Major version"
                },
                "minor": {
                    "type": "integer",
                    "description": "Minor version"
                },
                "patch": {
                    "type": "integer",
                    "description": "Patch version"
                }
            }
        },
        "type": {
            "description": "Type of the record",
            "oneOf": [
                {
                    "title": "Other",
                    "const": 0
                },
                {
                    "title": "Video",
                    "const": 1
                },
                {
                    "title": "Imu",
                    "const": 2
                }
            ]
        },
        "timestamp": {
            "type": "object",
            "description": "Timestamp of the message",
            "properties": {
                "seconds": {
                    "type": "integer",
                    "description": "Seconds part of the timestamp"
                },
                "nanoseconds": {
                    "type": "integer",
                    "description": "Nanoseconds part of the timestamp"
                }
            }
        },
        "sequenceNumber": {
            "type": "integer",
            "description": "Sequence number of the message"
        },
        "data": {
            "type": "bytes",
            "description": "Buffer data"
        }
    }
}
)";

// TODO(asahtik): Add values, maybe covariance
constexpr const char* IMU_SHEMA = R"(
{
    "title": "IMU record meta",
    "description": "Metadata for an IMU record stream",
    "type": "object",
    "properties": {
        "version": {
            "type": "object",
            "description": "Metadata version",
            "properties": {
                "major": {
                    "type": "integer",
                    "description": "Major version"
                },
                "minor": {
                    "type": "integer",
                    "description": "Minor version"
                },
                "patch": {
                    "type": "integer",
                    "description": "Patch version"
                }
            }
        },
        "type": {
            "description": "Type of the record",
            "oneOf": [
                {
                    "title": "Other",
                    "const": 0
                },
                {
                    "title": "Video",
                    "const": 1
                },
                {
                    "title": "Imu",
                    "const": 2
                }
            ]
        },
        "packets": {
            "type": "array",
            "description": "IMU packets",
            "items": {
                "type": "object",
                "description": "IMU packet",
                "properties": {
                    "orientation": {
                        "type": "object",
                        "description": "Accelerometer quaternion",
                        "properties": {
                            "timestamp": {
                                "type": "object",
                                "description": "Timestamp of the message",
                                "properties": {
                                    "seconds": {
                                        "type": "integer",
                                        "description": "Seconds part of the timestamp"
                                    },
                                    "nanoseconds": {
                                        "type": "integer",
                                        "description": "Nanoseconds part of the timestamp"
                                    }
                                }
                            },
                            "sequenceNumber": {
                                "type": "integer",
                                "description": "Sequence number of the message"
                            },
                            "x": {
                                "type": "number",
                                "description": "X axis value"
                            },
                            "y": {
                                "type": "number",
                                "description": "Y axis value"
                            },
                            "z": {
                                "type": "number",
                                "description": "Z axis value"
                            },
                            "w": {
                                "type": "number",
                                "description": "Z axis value"
                            }
                        }
                    },
                    "acceleration": {
                        "type": "object",
                        "description": "Accelerometer acceleration",
                        "properties": {
                            "timestamp": {
                                "type": "object",
                                "description": "Timestamp of the message",
                                "properties": {
                                    "seconds": {
                                        "type": "integer",
                                        "description": "Seconds part of the timestamp"
                                    },
                                    "nanoseconds": {
                                        "type": "integer",
                                        "description": "Nanoseconds part of the timestamp"
                                    }
                                }
                            },
                            "sequenceNumber": {
                                "type": "integer",
                                "description": "Sequence number of the message"
                            },
                            "x": {
                                "type": "number",
                                "description": "X axis value"
                            },
                            "y": {
                                "type": "number",
                                "description": "Y axis value"
                            },
                            "z": {
                                "type": "number",
                                "description": "Z axis value"
                            }
                        }
                    }
                }
            }
        }
    }
}
)";

constexpr const char* VIDEO_SHEMA = R"(
{
    "title": "Default record meta",
    "description": "Metadata for an unspecified record stream",
    "type": "object",
    "properties": {
        "version": {
            "type": "object",
            "description": "Metadata version",
            "properties": {
                "major": {
                    "type": "integer",
                    "description": "Major version"
                },
                "minor": {
                    "type": "integer",
                    "description": "Minor version"
                },
                "patch": {
                    "type": "integer",
                    "description": "Patch version"
                }
            }
        },
        "type": {
            "description": "Type of the record",
            "oneOf": [
                {
                    "title": "Other",
                    "const": 0
                },
                {
                    "title": "Video",
                    "const": 1
                },
                {
                    "title": "Imu",
                    "const": 2
                }
            ]
        },
        "timestamp": {
            "type": "object",
            "description": "Timestamp of the message",
            "properties": {
                "seconds": {
                    "type": "integer",
                    "description": "Seconds part of the timestamp"
                },
                "nanoseconds": {
                    "type": "integer",
                    "description": "Nanoseconds part of the timestamp"
                }
            }
        },
        "sequenceNumber": {
            "type": "integer",
            "description": "Sequence number of the message"
        },
        "instanceNumber": {
            "type": "integer",
            "description": "Instance number of the source"
        },
        "width": {
            "type": "integer",
            "description": "Frame width"
        },
        "height": {
            "type": "integer",
            "description": "Frame height"
        },
        "cameraSettings": {
            "type": "object",
            "description": "Camera settings",
            "properties": {
                "exposure": {
                    "type": "number",
                    "description": "Exposure time in microseconds"
                },
                "sensitivity": {
                    "type": "number",
                    "description": "Sensitivity in ISO"
                },
                "lensPosition": {
                    "type": "number",
                    "description": "Lens position"
                },
                "wbColorTemp": {
                    "type": "number",
                    "description": "Lens position"
                },
                "lensPositionRaw": {
                    "type": "number",
                    "description": "Raw lens position"
                }
            }
        }
    }
}
)";

}  // namespace utility
}  // namespace dai
