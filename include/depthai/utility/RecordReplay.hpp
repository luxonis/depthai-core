#pragma once

#include <mp4v2/mp4v2.h>
#include <spdlog/logger.h>

#include <cstdint>
#include <fstream>
#include <functional>

#include "depthai/properties/VideoEncoderProperties.hpp"
#include "nlohmann/json.hpp"
#include "span.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

#ifndef MCAP_COMPRESSION_NO_ZSTD
    #define MCAP_COMPRESSION_NO_ZSTD
#endif
#include "RecordReplaySchema.hpp"

namespace dai {

/**
 * Configuration for recording and replaying messages
 */
struct RecordConfig {
    using Profile = dai::VideoEncoderProperties::Profile;
    enum class RecordReplayState { RECORD, REPLAY, NONE };
    enum class CompressionLevel : uint8_t { NONE, FASTEST, FAST, DEFAULT, SLOW, SLOWEST };

    struct VideoEncoding {
        bool enabled = true;
        int bitrate = 0;
        Profile profile = Profile::H264_MAIN;
        bool lossless = false;
        int quality = 80;
    };

    RecordReplayState state = RecordReplayState::NONE;

    std::string outputDir;
    VideoEncoding videoEncoding;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
};

struct NodeRecordParams {
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
        j = json{{"outputDir", p.outputDir}, {"videoEncoding", vidEnc}, {"compressionLevel", p.compressionLevel}};
    }

    static void from_json(const json& j, dai::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        std::string profile;
        j.at("videoEncoding").at("enabled").get_to(p.videoEncoding.enabled);
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

        j.at("compressionLevel").get_to(p.compressionLevel);

        j.at("outputDir").get_to(p.outputDir);
    }
};
}  // namespace nlohmann
