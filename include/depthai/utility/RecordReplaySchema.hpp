#pragma once

#include <algorithm>
#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

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

struct IMUReportSchema {
    // TODO(asahtik): This does not seem like a lasting way
    // of representing the accuracy
    enum class Accuracy : std::uint8_t {
        UNRELIABLE = 0,
        LOW = 1,
        MEDIUM = 2,
        HIGH = 3,
        COVAR = 4,
    };

    TimestampSchema timestamp;
    uint64_t sequenceNumber;

    Accuracy accuracy;
    std::array<float, 9> covariance;
};

struct VectorSchema {
    float x;
    float y;
    float z;
};

struct QuaternionSchema {
    float i;
    float j;
    float k;
    float real;
};

struct IMUVectorSchema : public IMUReportSchema, public VectorSchema {};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IMUVectorSchema, x, y, z, timestamp, sequenceNumber, accuracy)

struct IMUQuaternionSchema : public IMUReportSchema, public QuaternionSchema {
    float rotationAccuracy;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IMUQuaternionSchema, i, j, k, real, rotationAccuracy, timestamp, sequenceNumber, covariance, accuracy)

struct IMUPacketSchema {
    IMUVectorSchema orientation;
    IMUVectorSchema acceleration;
    IMUVectorSchema magneticField;
    IMUQuaternionSchema rotationVector;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IMUPacketSchema, orientation, acceleration, magneticField, rotationVector)

struct IMURecordSchema {
    VersionSchema version{0, 0, 1};
    RecordType type = RecordType::Imu;
    std::vector<IMUPacketSchema> packets;

    dai::IMUData getMessage() {
        IMUData imuData;
        imuData.packets.reserve(packets.size());
        auto minTimestamp = packets.size() > 0 ? packets.front().acceleration.timestamp.get() : std::chrono::nanoseconds{0};
        for(const auto& packet : packets) {
            IMUPacket imuPacket;
            imuPacket.acceleroMeter.tsDevice.sec = packet.acceleration.timestamp.seconds;
            imuPacket.acceleroMeter.tsDevice.nsec = packet.acceleration.timestamp.nanoseconds;
            imuPacket.acceleroMeter.sequence = packet.acceleration.sequenceNumber;
            imuPacket.acceleroMeter.accuracy = (IMUReport::Accuracy)packet.acceleration.accuracy;
            imuPacket.acceleroMeter.x = packet.acceleration.x;
            imuPacket.acceleroMeter.y = packet.acceleration.y;
            imuPacket.acceleroMeter.z = packet.acceleration.z;

            imuPacket.gyroscope.tsDevice.sec = packet.orientation.timestamp.seconds;
            imuPacket.gyroscope.tsDevice.nsec = packet.orientation.timestamp.nanoseconds;
            imuPacket.gyroscope.sequence = packet.orientation.sequenceNumber;
            imuPacket.gyroscope.accuracy = (IMUReport::Accuracy)packet.orientation.accuracy;
            imuPacket.gyroscope.x = packet.orientation.x;
            imuPacket.gyroscope.y = packet.orientation.y;
            imuPacket.gyroscope.z = packet.orientation.z;

            imuPacket.magneticField.tsDevice.sec = packet.magneticField.timestamp.seconds;
            imuPacket.magneticField.tsDevice.nsec = packet.magneticField.timestamp.nanoseconds;
            imuPacket.magneticField.sequence = packet.magneticField.sequenceNumber;
            imuPacket.magneticField.accuracy = (IMUReport::Accuracy)packet.magneticField.accuracy;
            imuPacket.magneticField.x = packet.magneticField.x;
            imuPacket.magneticField.y = packet.magneticField.y;
            imuPacket.magneticField.z = packet.magneticField.z;

            imuPacket.rotationVector.tsDevice.sec = packet.rotationVector.timestamp.seconds;
            imuPacket.rotationVector.tsDevice.nsec = packet.rotationVector.timestamp.nanoseconds;
            imuPacket.rotationVector.sequence = packet.rotationVector.sequenceNumber;
            imuPacket.rotationVector.accuracy = (IMUReport::Accuracy)packet.rotationVector.accuracy;
            imuPacket.rotationVector.i = packet.rotationVector.i;
            imuPacket.rotationVector.j = packet.rotationVector.j;
            imuPacket.rotationVector.k = packet.rotationVector.k;
            imuPacket.rotationVector.real = packet.rotationVector.real;
            imuPacket.rotationVector.rotationVectorAccuracy = packet.rotationVector.rotationAccuracy;

            imuData.packets.push_back(imuPacket);
            std::min({minTimestamp,
                      packet.rotationVector.timestamp.get(),
                      packet.orientation.timestamp.get(),
                      packet.acceleration.timestamp.get(),
                      packet.magneticField.timestamp.get()});
        }
        imuData.setTimestampDevice(std::chrono::steady_clock::time_point(minTimestamp));
        return imuData;
    }
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IMURecordSchema, version, type, packets)

struct VideoCameraSettingsSchema {
    int32_t exposure;
    int32_t sensitivity;
    int32_t lensPosition;
    int32_t wbColorTemp;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VideoCameraSettingsSchema, exposure, sensitivity, lensPosition, wbColorTemp)

struct VideoRecordSchema {
    VersionSchema version{0, 0, 1};
    RecordType type = RecordType::Video;
    TimestampSchema timestamp;
    uint64_t sequenceNumber;
    uint64_t instanceNumber;
    uint32_t width;
    uint32_t height;
    VideoCameraSettingsSchema cameraSettings;

    dai::ImgFrame getMessage() {
        ImgFrame imgFrame;
        imgFrame.setWidth(width);
        imgFrame.setHeight(height);
        imgFrame.setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock>(timestamp.get()));
        imgFrame.setSequenceNum(sequenceNumber);
        imgFrame.setInstanceNum(instanceNumber);
        imgFrame.cam.wbColorTemp = cameraSettings.wbColorTemp;
        imgFrame.cam.lensPosition = cameraSettings.lensPosition;
        imgFrame.cam.exposureTimeUs = cameraSettings.exposure;
        imgFrame.cam.sensitivityIso = cameraSettings.sensitivity;
        return imgFrame;
    }
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
                        "description": "Gyroscope orientation",
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
                            "accuracy": {
                                "description": "Accuracy of the measurement",
                                "oneOf": [
                                    {
                                        "title": "Unreliable",
                                        "const": 0
                                    },
                                    {
                                        "title": "Low",
                                        "const": 1
                                    },
                                    {
                                        "title": "Medium",
                                        "const": 2
                                    },
                                    {
                                        "title": "High",
                                        "const": 3
                                    },
                                    {
                                        "title": "Covariance",
                                        "const": 4
                                    }

                                ]
                            },
                            "covariance": {
                                "type": "array",
                                "description": "Covariance matrix",
                                "items": {
                                    "type": "number"
                                }
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
                    },
                    "acceleration": {
                        "type": "object",
                        "description": "Acceleration vector",
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
                            "accuracy": {
                                "description": "Accuracy of the measurement",
                                "oneOf": [
                                    {
                                        "title": "Unreliable",
                                        "const": 0
                                    },
                                    {
                                        "title": "Low",
                                        "const": 1
                                    },
                                    {
                                        "title": "Medium",
                                        "const": 2
                                    },
                                    {
                                        "title": "High",
                                        "const": 3
                                    }
                                ]
                            },
                            "covariance": {
                                "type": "array",
                                "description": "Covariance matrix",
                                "items": {
                                    "type": "number"
                                }
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
                    },
                    "magneticField": {
                        "type": "object",
                        "description": "Magnetic field orientation",
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
                            "accuracy": {
                                "description": "Accuracy of the measurement",
                                "oneOf": [
                                    {
                                        "title": "Unreliable",
                                        "const": 0
                                    },
                                    {
                                        "title": "Low",
                                        "const": 1
                                    },
                                    {
                                        "title": "Medium",
                                        "const": 2
                                    },
                                    {
                                        "title": "High",
                                        "const": 3
                                    }
                                ]
                            },
                            "covariance": {
                                "type": "array",
                                "description": "Covariance matrix",
                                "items": {
                                    "type": "number"
                                }
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
                    },
                    "rotationVector": {
                        "type": "object",
                        "description": "Rotation vector with accuracy",
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
                            "accuracy": {
                                "description": "Accuracy of the measurement",
                                "oneOf": [
                                    {
                                        "title": "Unreliable",
                                        "const": 0
                                    },
                                    {
                                        "title": "Low",
                                        "const": 1
                                    },
                                    {
                                        "title": "Medium",
                                        "const": 2
                                    },
                                    {
                                        "title": "High",
                                        "const": 3
                                    }
                                ]
                            },
                            "covariance": {
                                "type": "array",
                                "description": "Covariance matrix",
                                "items": {
                                    "type": "number"
                                }
                            },
                            "i": {
                                "type": "number",
                                "description": "i axis quaterion value"
                            },
                            "j": {
                                "type": "number",
                                "description": "j axis quaternion value"
                            },
                            "k": {
                                "type": "number",
                                "description": "k axis quaternion value"
                            },
                            "real": {
                                "type": "number",
                                "description": "Quaternion scale value"
                            },
                            "rotationAccuracy": {
                                "type": "number",
                                "description": "Accuracy of the rotation vector in radians"
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
                }
            }
        }
    }
}
)";

}  // namespace utility
}  // namespace dai
