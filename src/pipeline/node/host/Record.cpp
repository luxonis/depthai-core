#include "depthai/pipeline/node/host/Record.hpp"

#include <chrono>
#include <cstdint>
#include <memory>

#include "depthai/config/config.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "depthai/utility/span.hpp"
#include "utility/RecordReplaySchema.hpp"

namespace dai {
namespace node {

enum class StreamType { EncodedVideo, RawVideo, Imu, Byte, Unknown };

using VideoCodec = dai::utility::VideoRecorder::VideoCodec;

void Record::run() {
    std::unique_ptr<utility::VideoRecorder> videoRecorder;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    videoRecorder = std::make_unique<dai::utility::VideoRecorder>();
#else
    throw std::runtime_error("Record node requires OpenCV support");
#endif

    utility::ByteRecorder byteRecorder;

    if(recordFile.empty()) {
        throw std::runtime_error("Record recordFile must be set");
    }

    std::string recordFileVideo = recordFile + ".mp4";
    std::string recordFileBytes = recordFile + ".mcap";

    StreamType streamType = StreamType::Unknown;
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int fps = 0;
    unsigned int i = 0;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    while(isRunning()) {
        auto msg = input.get<dai::Buffer>();
        if(msg == nullptr) continue;
        if(streamType == StreamType::Unknown) {
            if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
                auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
                if(imgFrame->getType() == dai::ImgFrame::Type::BITSTREAM)
                    throw std::runtime_error(
                        "Record node does not support encoded ImgFrame messages. Use the `out` output of VideoEncoder to record encoded frames.");
                streamType = StreamType::RawVideo;
                width = imgFrame->getWidth();
                height = imgFrame->getHeight();
                byteRecorder.init(recordFileBytes, compressionLevel, utility::RecordType::Video);
            } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
                auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                if(encFrame->getProfile() == EncodedFrame::Profile::HEVC) {
                    throw std::runtime_error("Record node does not support H265 encoding");
                }
                streamType = StreamType::EncodedVideo;
                width = encFrame->getWidth();
                height = encFrame->getHeight();
                if(logger) logger->trace("Record node detected {}x{} resolution", width, height);
                byteRecorder.init(recordFileBytes, compressionLevel, utility::RecordType::Video);
            } else if(std::dynamic_pointer_cast<IMUData>(msg) != nullptr) {
                streamType = StreamType::Imu;
                byteRecorder.init(recordFileBytes, compressionLevel, utility::RecordType::Imu);
            } else {
                streamType = StreamType::Byte;
                byteRecorder.init(recordFileBytes, compressionLevel, utility::RecordType::Other);
                throw std::runtime_error("Record node does not support this type of message");
            }
            if(logger)
                logger->trace("Record node detected stream type {}",
                              streamType == StreamType::RawVideo       ? "RawVideo"
                              : streamType == StreamType::EncodedVideo ? "EncodedVideo"
                                                                       : "Byte");
        }
        if(streamType == StreamType::RawVideo || streamType == StreamType::EncodedVideo) {
            if(i == 0)
                start = msg->getTimestampDevice();
            else if(i == fpsInitLength - 1) {
                end = msg->getTimestampDevice();
                fps = roundf((fpsInitLength * 1e6f) / (float)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
                if(logger) logger->trace("Record node detected {} fps", fps);
                if(streamType == StreamType::EncodedVideo) {
                    auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                    videoRecorder->init(
                        recordFileVideo, width, height, fps, encFrame->getProfile() == EncodedFrame::Profile::JPEG ? VideoCodec::MJPEG : VideoCodec::H264);
                } else {
                    videoRecorder->init(recordFileVideo, width, height, fps, VideoCodec::RAW);
                }
            }
            if(i >= fpsInitLength - 1) {
                auto data = msg->getData();
                if(streamType == StreamType::RawVideo) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
                    auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(msg);
                    auto frame = imgFrame->getCvFrame();
                    bool isGrayscale = imgFrame->getType() == ImgFrame::Type::GRAY8 || imgFrame->getType() == ImgFrame::Type::GRAYF16
                                       || (ImgFrame::Type::RAW16 <= imgFrame->getType() && imgFrame->getType() <= ImgFrame::Type::RAW8);
                    if(isGrayscale) {
                        cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
                    }
                    assert(frame.isContinuous());
                    span cvData(frame.data, frame.total() * frame.elemSize());
                    videoRecorder->write(cvData);
                    utility::VideoRecordSchema record;
                    record.timestamp.set(std::chrono::duration_cast<std::chrono::nanoseconds>(imgFrame->getTimestampDevice().time_since_epoch()));
                    record.sequenceNumber = imgFrame->getSequenceNum();
                    record.instanceNumber = imgFrame->getInstanceNum();
                    record.width = imgFrame->getWidth();
                    record.height = imgFrame->getHeight();
                    record.cameraSettings.exposure = imgFrame->cam.exposureTimeUs;
                    record.cameraSettings.sensitivity = imgFrame->cam.sensitivityIso;
                    record.cameraSettings.wbColorTemp = imgFrame->cam.wbColorTemp;
                    record.cameraSettings.lensPosition = imgFrame->cam.lensPosition;
                    record.cameraSettings.lensPositionRaw = imgFrame->cam.lensPositionRaw;
                    byteRecorder.write(record);
#else
                    throw std::runtime_error("Record node requires OpenCV support");
#endif
                } else {
                    videoRecorder->write(data);
                    auto encFrame = std::dynamic_pointer_cast<EncodedFrame>(msg);
                    utility::VideoRecordSchema record;
                    record.timestamp.set(std::chrono::duration_cast<std::chrono::nanoseconds>(encFrame->getTimestampDevice().time_since_epoch()));
                    record.sequenceNumber = encFrame->getSequenceNum();
                    record.instanceNumber = encFrame->getInstanceNum();
                    record.width = encFrame->getWidth();
                    record.height = encFrame->getHeight();
                    record.cameraSettings.exposure = encFrame->cam.exposureTimeUs;
                    record.cameraSettings.sensitivity = encFrame->cam.sensitivityIso;
                    record.cameraSettings.wbColorTemp = encFrame->cam.wbColorTemp;
                    record.cameraSettings.lensPosition = encFrame->cam.lensPosition;
                    record.cameraSettings.lensPositionRaw = encFrame->cam.lensPositionRaw;
                    byteRecorder.write(record);
                }
            }
            if(i < fpsInitLength) ++i;
        } else if(streamType == StreamType::Imu) {
            auto imuData = std::dynamic_pointer_cast<IMUData>(msg);
            utility::IMURecordSchema record;
            record.packets.reserve(imuData->packets.size());
            for(const auto& packet : imuData->packets) {
                utility::IMUPacketSchema packetSchema;
                // Acceleration
                packetSchema.acceleration.timestamp.set(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(packet.acceleroMeter.getTimestampDevice().time_since_epoch()));
                packetSchema.acceleration.sequenceNumber = packet.acceleroMeter.sequence;
                packetSchema.acceleration.accuracy = (utility::IMUReportSchema::Accuracy)packet.gyroscope.accuracy;
                packetSchema.acceleration.x = packet.acceleroMeter.x;
                packetSchema.acceleration.y = packet.acceleroMeter.y;
                packetSchema.acceleration.z = packet.acceleroMeter.z;
                // Orientation
                packetSchema.orientation.timestamp.set(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(packet.gyroscope.getTimestampDevice().time_since_epoch()));
                packetSchema.orientation.sequenceNumber = packet.gyroscope.sequence;
                packetSchema.orientation.accuracy = (utility::IMUReportSchema::Accuracy)packet.gyroscope.accuracy;
                packetSchema.orientation.x = packet.gyroscope.x;
                packetSchema.orientation.y = packet.gyroscope.y;
                packetSchema.orientation.z = packet.gyroscope.z;
                // Magnetic field
                packetSchema.magneticField.timestamp.set(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(packet.magneticField.getTimestampDevice().time_since_epoch()));
                packetSchema.magneticField.sequenceNumber = packet.magneticField.sequence;
                packetSchema.magneticField.accuracy = (utility::IMUReportSchema::Accuracy)packet.magneticField.accuracy;
                packetSchema.magneticField.x = packet.magneticField.x;
                packetSchema.magneticField.y = packet.magneticField.y;
                packetSchema.magneticField.z = packet.magneticField.z;
                // Rotation with accuracy
                packetSchema.rotationVector.timestamp.set(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(packet.rotationVector.getTimestampDevice().time_since_epoch()));
                packetSchema.rotationVector.sequenceNumber = packet.rotationVector.sequence;
                packetSchema.rotationVector.accuracy = (utility::IMUReportSchema::Accuracy)packet.rotationVector.accuracy;
                packetSchema.rotationVector.i = packet.rotationVector.i;
                packetSchema.rotationVector.j = packet.rotationVector.j;
                packetSchema.rotationVector.k = packet.rotationVector.k;
                packetSchema.rotationVector.real = packet.rotationVector.real;
                packetSchema.rotationVector.rotationAccuracy = packet.rotationVector.rotationVectorAccuracy;

                record.packets.push_back(packetSchema);
            }
            byteRecorder.write(record);
        } else {
            throw std::runtime_error("You can only record IMU or Video data");
        }
    }

    videoRecorder->close();
}

std::string Record::getRecordFile() const {
    return recordFile;
}

Record::CompressionLevel Record::getCompressionLevel() const {
    return compressionLevel;
}

Record& Record::setRecordFile(const std::string& recordFile) {
    this->recordFile = recordFile;
    return *this;
}

Record& Record::setCompressionLevel(CompressionLevel compressionLevel) {
    this->compressionLevel = compressionLevel;
    return *this;
}

}  // namespace node
}  // namespace dai
