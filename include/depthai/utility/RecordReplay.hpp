#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <stdexcept>

#include "depthai/properties/VideoEncoderProperties.hpp"
#include "nlohmann/json.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

#ifndef MCAP_COMPRESSION_NO_ZSTD
    #define MCAP_COMPRESSION_NO_ZSTD
#endif

namespace dai {

/**
 * RecordVideo container. VIDEO writes a video file and metadata. RAW_FRAMES writes one .dai file per frame.
 */
enum class RecordFormat : std::int32_t { VIDEO = 0, RAW_FRAMES = 1 };

/**
 * Configuration for recording and replaying messages
 */
struct RecordConfig {
    using Profile = dai::VideoEncoderProperties::Profile;
    enum class RecordReplayState { RECORD, REPLAY, NONE };
    enum class CompressionLevel : uint8_t { NONE, FASTEST, FAST, DEFAULT, SLOW, SLOWEST };

    /**
     * Fps of one holistic stream, stored in record_config.json under streams.
     */
    struct StreamConfig {
        float fps = 30.f;
    };

    struct VideoEncoding {
        bool enabled = true;
        int bitrate = 0;
        Profile profile = Profile::H264_MAIN;
        bool lossless = false;
        int quality = 80;
    };

    RecordReplayState state = RecordReplayState::NONE;

    std::filesystem::path outputDir;
    VideoEncoding videoEncoding;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
    bool syncCameraOutputs = true;
    /** VIDEO keeps the encoder path. RAW_FRAMES records the camera frames directly. */
    RecordFormat format = RecordFormat::VIDEO;
    /** Keep the device copy after a holistic or record-node transfer. */
    bool keepOnDevice = false;
    /** Per-stream fps written for holistic replay. Optional when a JSON config is read. */
    std::map<std::string, StreamConfig> streams;
};

struct NodeRecordParams {
    bool video;
    std::string name;
};

}  // namespace dai

using json = nlohmann::json;

namespace nlohmann {
template <>
struct adl_serializer<dai::RecordConfig> {
    static void to_json(json& j, const dai::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        std::string profile;
        switch(p.videoEncoding.profile) {
            case dai::RecordConfig::Profile::H264_BASELINE:
                profile = "H264_BASELINE";
                break;
            case dai::RecordConfig::Profile::H264_HIGH:
                profile = "H264_HIGH";
                break;
            case dai::RecordConfig::Profile::H264_MAIN:
                profile = "H264_MAIN";
                break;
            case dai::RecordConfig::Profile::H265_MAIN:
                profile = "H265_MAIN";
                break;
            case dai::RecordConfig::Profile::MJPEG:
                profile = "MJPEG";
                break;
        }
        auto vidEnc = json{{"enabled", p.videoEncoding.enabled},
                           {"bitrate", p.videoEncoding.bitrate},
                           {"profile", profile},
                           {"lossless", p.videoEncoding.lossless},
                           {"quality", p.videoEncoding.quality}};
        j = json{{"outputDir", p.outputDir}, {"videoEncoding", vidEnc}, {"compressionLevel", p.compressionLevel}, {"syncCameraOutputs", p.syncCameraOutputs}};
        j["format"] = p.format == dai::RecordFormat::RAW_FRAMES ? "RAW_FRAMES" : "VIDEO";
        j["keepOnDevice"] = p.keepOnDevice;
        json streams = json::object();
        for(const auto& stream : p.streams) {
            streams[stream.first] = json{{"fps", stream.second.fps}};
        }
        j["streams"] = streams;
    }

    static void from_json(const json& j, dai::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        std::string profile;
        j.at("videoEncoding").at("enabled").get_to(p.videoEncoding.enabled);
        if(p.videoEncoding.enabled) {
            j.at("videoEncoding").at("bitrate").get_to(p.videoEncoding.bitrate);
            j.at("videoEncoding").at("profile").get_to(profile);
            j.at("videoEncoding").at("lossless").get_to(p.videoEncoding.lossless);
            j.at("videoEncoding").at("quality").get_to(p.videoEncoding.quality);
            std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);

            p.videoEncoding.profile = dai::RecordConfig::Profile::MJPEG;
            if(profile == "h264_baseline" || profile == "avc_baseline") {
                p.videoEncoding.profile = dai::RecordConfig::Profile::H264_BASELINE;
            } else if(profile == "h264_high" || profile == "avc_high") {
                p.videoEncoding.profile = dai::RecordConfig::Profile::H264_HIGH;
            } else if(profile == "h264_main" || profile == "avc_main" || profile == "h264" || profile == "avc") {
                p.videoEncoding.profile = dai::RecordConfig::Profile::H264_MAIN;
            } else if(profile == "h265_main" || profile == "hevc_main" || profile == "h265" || profile == "hevc") {
                p.videoEncoding.profile = dai::RecordConfig::Profile::H265_MAIN;
            } else if(profile == "mjpeg") {
                p.videoEncoding.profile = dai::RecordConfig::Profile::MJPEG;
            }
        }

        j.at("compressionLevel").get_to(p.compressionLevel);
        if(j.contains("syncCameraOutputs")) {
            j.at("syncCameraOutputs").get_to(p.syncCameraOutputs);
        }

        j.at("outputDir").get_to(p.outputDir);
        if(j.contains("format")) {
            std::string format;
            j.at("format").get_to(format);
            if(format == "VIDEO") {
                p.format = dai::RecordFormat::VIDEO;
            } else if(format == "RAW_FRAMES") {
                p.format = dai::RecordFormat::RAW_FRAMES;
            } else {
                throw std::invalid_argument("Unknown record format: " + format);
            }
        }
        if(j.contains("keepOnDevice")) {
            j.at("keepOnDevice").get_to(p.keepOnDevice);
        }
        if(j.contains("streams")) {
            p.streams.clear();
            for(const auto& stream : j.at("streams").items()) {
                dai::RecordConfig::StreamConfig streamConfig;
                stream.value().at("fps").get_to(streamConfig.fps);
                p.streams.emplace(stream.key(), streamConfig);
            }
        }
    }
};
}  // namespace nlohmann
