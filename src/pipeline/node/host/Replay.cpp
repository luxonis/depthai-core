#define _USE_MATH_DEFINES
#include "depthai/pipeline/node/host/Replay.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>

#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/RecordReplaySchema.hpp"
#include "utility/RecordReplayImpl.hpp"

namespace dai {
namespace node {

// Video Message
std::shared_ptr<Buffer> getVideoMessage(const nlohmann::json& metadata, ImgFrame::Type outFrameType, std::vector<uint8_t>& frame) {
    // TODO(asahtik): Handle versions
    utility::VideoRecordSchema recordSchema = metadata;
    ImgFrame imgFrame = recordSchema.getMessage();

    assert(frame.size() == recordSchema.width * recordSchema.height * 3);
    cv::Mat img(recordSchema.height, recordSchema.width, CV_8UC3, frame.data());
    imgFrame.setCvFrame(img, outFrameType);
    return std::dynamic_pointer_cast<Buffer>(std::make_shared<ImgFrame>(imgFrame));
}

std::shared_ptr<Buffer> getMessage(const nlohmann::json& metadata, utility::RecordType type) {
    switch(type) {
        case utility::RecordType::Other:
        case utility::RecordType::Video:
            throw std::runtime_error("Invalid message type");
            break;
        case utility::RecordType::Imu: {
            utility::IMURecordSchema recordSchema = metadata;
            IMUData imuData = recordSchema.getMessage();
            return std::dynamic_pointer_cast<Buffer>(std::make_shared<IMUData>(imuData));
        }
    }
    return {};
}

void ReplayVideo::run() {
    if(replayVideo.empty() && replayFile.empty()) {
        throw std::runtime_error("ReplayVideo node requires replayVideo or replayFile to be set");
    }
    utility::VideoPlayer videoPlayer;
    utility::BytePlayer bytePlayer;
    bool hasVideo = !replayVideo.empty();
    bool hasMetadata = !replayFile.empty();
    if(!replayVideo.empty()) try {
            videoPlayer.init(replayVideo.string());
            if(size.has_value()) {
                const auto& [width, height] = size.value();
                videoPlayer.setSize(width, height);
            }
        } catch(const std::exception& e) {
            hasVideo = false;
            if(logger) logger->warn("Video not replaying: {}", e.what());
        }
    if(!replayFile.empty()) try {
            bytePlayer.init(replayFile.string());
        } catch(const std::exception& e) {
            hasMetadata = false;
            if(logger) logger->warn("Metadata not replaying: {}", e.what());
        }
    utility::RecordType type = utility::RecordType::Other;
    if(hasVideo && !hasMetadata) {
        type = utility::RecordType::Video;
    }
    if(!hasVideo) {
        throw std::runtime_error("Video file not found or could not be opened");
    }
    bool first = true;
    auto start = std::chrono::steady_clock::now();
    uint64_t index = 0;
    auto loopStart = std::chrono::steady_clock::now();
    auto prevMsgTs = loopStart;
    while(isRunning()) {
        nlohmann::json metadata;
        std::vector<uint8_t> frame;
        if(hasMetadata) {
            auto msg = bytePlayer.next();
            if(msg.has_value()) {
                metadata = msg.value();
                if(first) {
                    type = metadata["type"].get<utility::RecordType>();
                    if(type != utility::RecordType::Video) {
                        throw std::runtime_error("Invalid message type, expected a video stream");
                    }
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
                if(loop) {
                    if(hasMetadata) {
                        bytePlayer.restart();
                    }
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

        auto buffer = getVideoMessage(metadata, outFrameType, frame);

        if(first) prevMsgTs = buffer->getTimestampDevice();

        if(hasMetadata && !(fps.has_value() && fps.value() > 0.1f)) {
            std::this_thread::sleep_until(loopStart + (buffer->getTimestampDevice() - prevMsgTs));
        }

        if(buffer) out.send(buffer);

        if(fps.has_value() && fps.value() > 0.1f) {
            std::this_thread::sleep_until(loopStart + std::chrono::milliseconds((uint32_t)roundf(1000.f / fps.value())));
        } else if(!hasMetadata) {
            std::this_thread::sleep_until(loopStart + std::chrono::milliseconds(1000 / 30));
        }

        loopStart = std::chrono::steady_clock::now();
        prevMsgTs = buffer->getTimestampDevice();

        first = false;
    }
    logger->info("Replay finished - stopping the pipeline!");
    stopPipeline();
}

void ReplayMetadataOnly::run() {
    if(replayFile.empty()) {
        throw std::runtime_error("ReplayMetadataOnly node requires replayFile to be set");
    }
    utility::BytePlayer bytePlayer;
    bool hasMetadata = !replayFile.empty();
    if(!replayFile.empty()) try {
            bytePlayer.init(replayFile.string());
        } catch(const std::exception& e) {
            hasMetadata = false;
            if(logger) logger->warn("Metadata not replaying: {}", e.what());
        }
    if(!hasMetadata) {
        throw std::runtime_error("Metadata file not found");
    }
    utility::RecordType type = utility::RecordType::Other;
    bool first = true;
    auto loopStart = std::chrono::steady_clock::now();
    auto prevMsgTs = loopStart;
    while(isRunning()) {
        nlohmann::json metadata;
        std::vector<uint8_t> frame;
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
                continue;
            }
            break;
        } else {
            throw std::runtime_error("Metadata file contains no messages");
        }
        auto buffer = getMessage(metadata, type);
        if(first) prevMsgTs = buffer->getTimestampDevice();

        if(!(fps.has_value() && fps.value() > 0.1f)) {
            std::this_thread::sleep_until(loopStart + (buffer->getTimestampDevice() - prevMsgTs));
        }

        if(buffer) out.send(buffer);

        if(fps.has_value() && fps.value() > 0.1f) {
            std::this_thread::sleep_until(loopStart + std::chrono::milliseconds((uint32_t)roundf(1000.f / fps.value())));
        }

        loopStart = std::chrono::steady_clock::now();
        prevMsgTs = buffer->getTimestampDevice();

        first = false;
    }
    stopPipeline();
}

std::filesystem::path ReplayVideo::getReplayMetadataFile() const {
    return replayFile;
}

std::filesystem::path ReplayVideo::getReplayVideoFile() const {
    return replayVideo;
}

ImgFrame::Type ReplayVideo::getOutFrameType() const {
    return outFrameType;
}

std::tuple<int, int> ReplayVideo::getSize() const {
    return size.value_or(std::make_tuple(0, 0));
}

float ReplayVideo::getFps() const {
    return fps.value_or(0.0f);
}

bool ReplayVideo::getLoop() const {
    return loop;
}

ReplayVideo& ReplayVideo::setReplayMetadataFile(const std::filesystem::path& replayFile) {
    this->replayFile = replayFile;
    return *this;
}

ReplayVideo& ReplayVideo::setReplayVideoFile(const std::filesystem::path& replayVideo) {
    this->replayVideo = replayVideo;
    return *this;
}

ReplayVideo& ReplayVideo::setOutFrameType(ImgFrame::Type outFrameType) {
    this->outFrameType = outFrameType;
    return *this;
}

ReplayVideo& ReplayVideo::setSize(std::tuple<int, int> size) {
    this->size = size;
    return *this;
}
ReplayVideo& ReplayVideo::setSize(int width, int height) {
    return setSize(std::make_tuple(width, height));
}
ReplayVideo& ReplayVideo::setFps(float fps) {
    this->fps = fps;
    return *this;
}
ReplayVideo& ReplayVideo::setLoop(bool loop) {
    this->loop = loop;
    return *this;
}

std::filesystem::path ReplayMetadataOnly::getReplayFile() const {
    return replayFile;
}
float ReplayMetadataOnly::getFps() const {
    return fps.value_or(0.0f);
}
bool ReplayMetadataOnly::getLoop() const {
    return loop;
}

ReplayMetadataOnly& ReplayMetadataOnly::setReplayFile(const std::filesystem::path& replayFile) {
    this->replayFile = replayFile;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setFps(float fps) {
    this->fps = fps;
    return *this;
}
ReplayMetadataOnly& ReplayMetadataOnly::setLoop(bool loop) {
    this->loop = loop;
    return *this;
}

}  // namespace node
}  // namespace dai
