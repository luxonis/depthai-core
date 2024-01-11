#pragma once

#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "nlohmann/json.hpp"

using Profile = dai::VideoEncoderProperties::Profile;

namespace dai {
namespace utility {

struct RecordConfig {
    struct VideoEncoding {
        bool enabled = false;
        int bitrate = 0;
        Profile profile = Profile::H264_MAIN;
        bool lossless = true;
        int quality = 80;
    };
    struct ByteEncoding {
        bool enabled = true;
        int compressionLevel = 6;
    };

    std::string outputDir;
    VideoEncoding videoEncoding;
    ByteEncoding byteEncoding;
};

}  // namespace utility
}  // namespace dai

using json = nlohmann::json;

namespace nlohmann {
void to_json(json& j, const dai::utility::RecordConfig::VideoEncoding& p) {  // NOLINT
    std::string profile;
    switch(p.profile) {
        case Profile::H264_BASELINE:
            profile = "H264_BASELINE";
            break;
        case Profile::H264_HIGH:
            profile = "H264_HIGH";
            break;
        case Profile::H264_MAIN:
            profile = "H264_MAIN";
            break;
        case Profile::H265_MAIN:
            profile = "H265_MAIN";
            break;
        case Profile::MJPEG:
            profile = "MJPEG";
            break;
    }
    j = json{{"enabled", p.enabled}, {"bitrate", p.bitrate}, {"profile", profile}, {"lossless", p.lossless}, {"quality", p.quality}};
}
void to_json(json& j, const dai::utility::RecordConfig::ByteEncoding& p) {  // NOLINT
    j = json{{{"enabled", p.enabled}, {"compressionLevel", p.compressionLevel}}};
}
void to_json(json& j, const dai::utility::RecordConfig& p) {  // NOLINT
    j = json{{"outputDir", p.outputDir}, {"videoEncoding", p.videoEncoding}, {"byteEncoding", p.byteEncoding}};
}

void from_json(const json& j, dai::utility::RecordConfig::VideoEncoding& p) {  // NOLINT
    std::string profile;
    j.at("enabled").get_to(p.enabled);
    j.at("bitrate").get_to(p.bitrate);
    j.at("profile").get_to(profile);
    j.at("lossless").get_to(p.lossless);
    j.at("quality").get_to(p.quality);
    std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);

    p.profile = Profile::MJPEG;
    if(profile == "h264_baseline" || profile == "avc_baseline") {
        p.profile = Profile::H264_BASELINE;
    } else if(profile == "h264_high" || profile == "avc_high") {
        p.profile = Profile::H264_HIGH;
    } else if(profile == "h264_main" || profile == "avc_main" || profile == "h264" || profile == "avc") {
        p.profile = Profile::H264_MAIN;
    } else if(profile == "h265_main" || profile == "hevc_main" || profile == "h265" || profile == "hevc") {
        p.profile = Profile::H265_MAIN;
    } else if(profile == "mjpeg") {
        p.profile = Profile::MJPEG;
    }
}
void from_json(const json& j, dai::utility::RecordConfig::ByteEncoding& p) {  // NOLINT
    std::string algorithm;
    j.at("enabled").get_to(p.enabled);
    j.at("compressionLevel").get_to(p.compressionLevel);
    p.compressionLevel = std::max(0, std::min(9, p.compressionLevel));
}
void from_json(const json& j, dai::utility::RecordConfig& p) {  // NOLINT
    j.at("outputDir").get_to(p.outputDir);
    j.at("videoEncoding").get_to(p.videoEncoding);
    j.at("byteEncoding").get_to(p.byteEncoding);
}
}  // namespace ns
