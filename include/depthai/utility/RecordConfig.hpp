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
template <>
struct adl_serializer<dai::utility::RecordConfig> {
    static void to_json(json& j, const dai::utility::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        std::string profile;
        switch(p.videoEncoding.profile) {
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
        auto vidEnc = json{{"enabled", p.videoEncoding.enabled},
                           {"bitrate", p.videoEncoding.bitrate},
                           {"profile", profile},
                           {"lossless", p.videoEncoding.lossless},
                           {"quality", p.videoEncoding.quality}};
        auto byteEnc = json{{{"enabled", p.byteEncoding.enabled}, {"compressionLevel", p.byteEncoding.compressionLevel}}};
        j = json{{"outputDir", p.outputDir}, {"videoEncoding", vidEnc}, {"byteEncoding", byteEnc}};
    }

    static void from_json(const json& j, dai::utility::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        std::string profile;
        j.at("videoEncoding").at("enabled").get_to(p.videoEncoding.enabled);
        j.at("videoEncoding").at("bitrate").get_to(p.videoEncoding.bitrate);
        j.at("videoEncoding").at("profile").get_to(profile);
        j.at("videoEncoding").at("lossless").get_to(p.videoEncoding.lossless);
        j.at("videoEncoding").at("quality").get_to(p.videoEncoding.quality);
        std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);

        p.videoEncoding.profile = Profile::MJPEG;
        if(profile == "h264_baseline" || profile == "avc_baseline") {
            p.videoEncoding.profile = Profile::H264_BASELINE;
        } else if(profile == "h264_high" || profile == "avc_high") {
            p.videoEncoding.profile = Profile::H264_HIGH;
        } else if(profile == "h264_main" || profile == "avc_main" || profile == "h264" || profile == "avc") {
            p.videoEncoding.profile = Profile::H264_MAIN;
        } else if(profile == "h265_main" || profile == "hevc_main" || profile == "h265" || profile == "hevc") {
            p.videoEncoding.profile = Profile::H265_MAIN;
        } else if(profile == "mjpeg") {
            p.videoEncoding.profile = Profile::MJPEG;
        }

        j.at("byteEncoding").at("enabled").get_to(p.byteEncoding.enabled);
        j.at("byteEncoding").at("compressionLevel").get_to(p.byteEncoding.compressionLevel);
        p.byteEncoding.compressionLevel = std::max(0, std::min(9, p.byteEncoding.compressionLevel));

        j.at("outputDir").get_to(p.outputDir);
    }
};
}  // namespace nlohmann
