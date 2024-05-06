#include "depthai/pipeline/node/host/Replay.hpp"

#include <chrono>
#include <memory>

#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "depthai/utility/RecordReplaySchema.hpp"

namespace dai {
namespace node {

std::shared_ptr<Buffer> Replay::getMessage(utility::RecordType type, const nlohmann::json& metadata, std::vector<uint8_t>& frame) {
    // TODO(asahtik): Handle versions
    switch(type) {
        case utility::RecordType::Other:
            throw std::runtime_error("Unsupported record type");
        case utility::RecordType::Video: {
            utility::VideoRecordSchema recordSchema = metadata;
            ImgFrame imgFrame;
            imgFrame.setWidth(recordSchema.width);
            imgFrame.setHeight(recordSchema.height);
            imgFrame.setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock>(recordSchema.timestamp.get()));
            imgFrame.setSequenceNum(recordSchema.sequenceNumber);
            imgFrame.setInstanceNum(recordSchema.instanceNumber);
            imgFrame.cam.wbColorTemp = recordSchema.cameraSettings.wbColorTemp;
            imgFrame.cam.lensPosition = recordSchema.cameraSettings.lensPosition;
            imgFrame.cam.lensPositionRaw = recordSchema.cameraSettings.lensPositionRaw;
            imgFrame.cam.exposureTimeUs = recordSchema.cameraSettings.exposure;
            imgFrame.cam.sensitivityIso = recordSchema.cameraSettings.sensitivity;

            assert(frame.size() == recordSchema.width * recordSchema.height * 3);
            cv::Mat img(recordSchema.height, recordSchema.width, CV_8UC3, frame.data());
            imgFrame.setCvFrame(img, outFrameType);
            return std::dynamic_pointer_cast<Buffer>(std::make_shared<ImgFrame>(imgFrame));
        }
        case utility::RecordType::Imu: {
            utility::IMURecordSchema recordSchema = metadata;
            IMUData imuData;
            imuData.packets.reserve(recordSchema.packets.size());
            for(const auto& packet : recordSchema.packets) {
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
            }
            return std::dynamic_pointer_cast<Buffer>(std::make_shared<IMUData>(imuData));
        }
    }
    return {};
}

void Replay::run() {
    if(replayVideo.empty() && replayFile.empty()) {
        throw std::runtime_error("Replay node requires replayVideo or replayFile to be set");
    }
    utility::VideoPlayer videoPlayer;
    utility::BytePlayer bytePlayer;
    bool hasVideo = !replayVideo.empty();
    bool hasMetadata = !replayFile.empty();
    if(!replayVideo.empty()) try {
            videoPlayer.init(replayVideo);
            if(size.has_value()) {
                const auto& [width, height] = size.value();
                videoPlayer.setSize(width, height);
            }
        } catch(const std::exception& e) {
            hasVideo = false;
            if(logger) logger->warn("Video not replaying: {}", e.what());
        }
    if(!replayFile.empty()) try {
            bytePlayer.init(replayFile);
        } catch(const std::exception& e) {
            hasMetadata = false;
            if(logger) logger->warn("Metadata not replaying: {}", e.what());
        }
    utility::RecordType type = utility::RecordType::Other;
    if(hasVideo && !hasMetadata) {
        type = utility::RecordType::Video;
    }
    bool first = true;
    auto start = std::chrono::steady_clock::now();
    uint64_t index = 0;
    auto loopStart = std::chrono::steady_clock::now();
    while(isRunning()) {
        nlohmann::json metadata;
        std::vector<uint8_t> frame;
        if(hasMetadata) {
            auto msg = bytePlayer.next();
            if(msg.has_value()) {
                metadata = msg.value();
                if(first) {
                    type = metadata["type"].get<utility::RecordType>();
                }
            } else if(!first) {
                // End of file
                if(loop) {
                    bytePlayer.restart();
                    if(hasVideo && type == utility::RecordType::Video) {
                        videoPlayer.restart();
                    }
                    continue;
                }
                break;
            } else {
                hasMetadata = false;
            }
        }
        if(hasVideo && type == utility::RecordType::Video) {
            auto msg = videoPlayer.next();
            if(msg.has_value()) {
                frame = msg.value();
            } else if(!first) {
                // End of file
                if(loop){
                    bytePlayer.restart();
                    videoPlayer.restart();
                    continue;
                }
                break;
            } else {
                hasVideo = false;
            }
        }

        if(!hasMetadata && !hasVideo) {
            break;
        }

        if(!hasVideo && type == utility::RecordType::Video) {
            throw std::runtime_error("Video file not found");
        }

        if(!hasMetadata && type == utility::RecordType::Video) {
            utility::VideoRecordSchema recordSchema;
            auto time = std::chrono::steady_clock::now() - start;
            const auto& [width, height] = videoPlayer.size();
            recordSchema.type = utility::RecordType::Video;
            recordSchema.timestamp.set(time);
            recordSchema.sequenceNumber = index++;
            recordSchema.width = width;
            recordSchema.height = height;
            metadata = recordSchema;
        } else if(type == utility::RecordType::Video && size.has_value()) {
            utility::VideoRecordSchema recordSchema = metadata;
            recordSchema.width = std::get<0>(size.value());
            recordSchema.height = std::get<1>(size.value());
            metadata = recordSchema;
        }

        auto buffer = getMessage(type, metadata, frame);

        if(buffer) out.send(buffer);

        if(fps.has_value() && fps.value() > 0.1f) {
            std::this_thread::sleep_until(loopStart + std::chrono::milliseconds((uint32_t)roundf(1000.f / fps.value())));
        }

        loopStart = std::chrono::steady_clock::now();

        first = false;
    }

    stop();
}

Replay& Replay::setReplayFile(const std::string& replayFile) {
    this->replayFile = replayFile;
    return *this;
}

Replay& Replay::setReplayVideo(const std::string& replayVideo) {
    this->replayVideo = replayVideo;
    return *this;
}

Replay& Replay::setOutFrameType(ImgFrame::Type outFrameType) {
    this->outFrameType = outFrameType;
    return *this;
}

Replay& Replay::setSize(std::tuple<int, int> size) {
    this->size = size;
    return *this;
}
Replay& Replay::setSize(int width, int height) {
    return setSize(std::make_tuple(width, height));
}
Replay& Replay::setFps(float fps) {
    this->fps = fps;
    return *this;
}
Replay& Replay::setLoop(bool loop) {
    this->loop = loop;
    return *this;
}

}  // namespace node
}  // namespace dai
