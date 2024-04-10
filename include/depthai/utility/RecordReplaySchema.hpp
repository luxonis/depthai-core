#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace dai {
namespace utility {

enum class RecordType {
    Other = 0,
    Video = 1,
    Imu = 2
};

struct VersionSchema {
    uint16_t major;
    uint16_t minor;
    uint16_t patch;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VersionSchema, major, minor, patch)

// Make sure these structs are in sync with the JSON schema
struct DefaultRecordSchema {
    VersionSchema version {0, 0, 1};
    RecordType type = RecordType::Other;
    uint64_t timestamp;
    uint64_t sequenceNumber;
    std::vector<uint8_t> data;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DefaultRecordSchema, version, type, timestamp, sequenceNumber, data)

struct ImuOrientationSchema {
    uint64_t timestamp;
    uint64_t sequenceNumber;

    float x;
    float y;
    float z;
    float w;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImuOrientationSchema, x, y, z, w)

struct IMUAccelerationSchema {
    uint64_t timestamp;
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
    VersionSchema version {0, 0, 1};
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
    VersionSchema version {0, 0, 1};
    RecordType type = RecordType::Video;
    uint64_t timestamp;
    uint64_t sequenceNumber;
    uint64_t instanceNum;
    uint32_t width;
    uint32_t height;
    VideoCameraSettingsSchema cameraSettings;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VideoRecordSchema, version, type, timestamp, sequenceNumber, instanceNum, width, height, cameraSettings)

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
            "type": "integer",
            "description": "Timestamp of the message in nanoseconds"
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
                                "type": "integer",
                                "description": "Timestamp of the message in nanoseconds"
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
                                "type": "integer",
                                "description": "Timestamp of the message in nanoseconds"
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
            "type": "integer",
            "description": "Timestamp of the message in nanoseconds"
        },
        "sequenceNumber": {
            "type": "integer",
            "description": "Sequence number of the message"
        },
        "instanceNum": {
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

}
}
