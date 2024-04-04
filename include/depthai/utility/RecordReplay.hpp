#pragma once

#include <mp4v2/mp4v2.h>
#include <spdlog/logger.h>

#include <cstdint>

#include "nlohmann/json.hpp"
#include "span.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

namespace dai {
namespace utility {

class VideoRecorder {
   public:
    enum class VideoCodec { H264, MJPEG, RAW };

    ~VideoRecorder();
    void init(const std::string& filePath, unsigned int width, unsigned int height, unsigned int fps, VideoCodec codec);
    void write(span<const uint8_t>&);
    void close();
    bool isInitialized() const {
        return initialized;
    }

   private:
    bool initialized = false;
    bool mp4v2Initialized = false;
    unsigned int fps = 0;
    unsigned int width = 0;
    unsigned int height = 0;
    VideoCodec codec = VideoCodec::RAW;
    MP4FileHandle mp4Writer = MP4_INVALID_FILE_HANDLE;
    MP4TrackId mp4Track = MP4_INVALID_TRACK_ID;
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<cv::VideoWriter> cvWriter;
#else
    std::unique_ptr<int> cvWriter;
#endif
};

class VideoPlayer {
   public:
    ~VideoPlayer();
    void init(const std::string& filePath);
    std::vector<uint8_t> next();
    void close();
    bool isInitialized() const {
        return initialized;
    }

   private:
    bool initialized = false;
};

struct NodeRecordParams {
    std::string name;
};

struct RecordConfig {
    enum class RecordReplayState { RECORD, REPLAY, NONE };

    // struct VideoEncoding {
    //     bool enabled = false;
    //     int bitrate = 0;
    //     Profile profile = Profile::H264_MAIN;
    //     bool lossless = true;
    //     int quality = 80;
    // };
    struct ByteEncoding {
        bool enabled = true;
        int compressionLevel = 6;
    };

    RecordReplayState state = RecordReplayState::NONE;

    std::string outputDir;
    // VideoEncoding videoEncoding;
    ByteEncoding byteEncoding;
};

bool checkRecordConfig(std::string& recordPath, RecordConfig& config);

bool allMatch(const std::vector<std::string>& v1, const std::vector<std::string>& v2);

std::string matchTo(const std::vector<std::string>& mxIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames);

}  // namespace utility
}  // namespace dai

using json = nlohmann::json;

namespace nlohmann {
template <>
struct adl_serializer<dai::utility::RecordConfig> {
    static void to_json(json& j, const dai::utility::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        // std::string profile;
        // switch(p.videoEncoding.profile) {
        //     case Profile::H264_BASELINE:
        //         profile = "H264_BASELINE";
        //         break;
        //     case Profile::H264_HIGH:
        //         profile = "H264_HIGH";
        //         break;
        //     case Profile::H264_MAIN:
        //         profile = "H264_MAIN";
        //         break;
        //     case Profile::H265_MAIN:
        //         profile = "H265_MAIN";
        //         break;
        //     case Profile::MJPEG:
        //         profile = "MJPEG";
        //         break;
        // }
        // auto vidEnc = json{{"enabled", p.videoEncoding.enabled},
        //                    {"bitrate", p.videoEncoding.bitrate},
        //                    {"profile", profile},
        //                    {"lossless", p.videoEncoding.lossless},
        //                    {"quality", p.videoEncoding.quality}};
        auto byteEnc = json{{{"enabled", p.byteEncoding.enabled}, {"compressionLevel", p.byteEncoding.compressionLevel}}};
        j = json{{"outputDir", p.outputDir} /* , {"videoEncoding", vidEnc} */, {"byteEncoding", byteEnc}};
    }

    static void from_json(const json& j, dai::utility::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        // std::string profile;
        // j.at("videoEncoding").at("enabled").get_to(p.videoEncoding.enabled);
        // j.at("videoEncoding").at("bitrate").get_to(p.videoEncoding.bitrate);
        // j.at("videoEncoding").at("profile").get_to(profile);
        // j.at("videoEncoding").at("lossless").get_to(p.videoEncoding.lossless);
        // j.at("videoEncoding").at("quality").get_to(p.videoEncoding.quality);
        // std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);
        //
        // p.videoEncoding.profile = Profile::MJPEG;
        // if(profile == "h264_baseline" || profile == "avc_baseline") {
        //     p.videoEncoding.profile = Profile::H264_BASELINE;
        // } else if(profile == "h264_high" || profile == "avc_high") {
        //     p.videoEncoding.profile = Profile::H264_HIGH;
        // } else if(profile == "h264_main" || profile == "avc_main" || profile == "h264" || profile == "avc") {
        //     p.videoEncoding.profile = Profile::H264_MAIN;
        // } else if(profile == "h265_main" || profile == "hevc_main" || profile == "h265" || profile == "hevc") {
        //     p.videoEncoding.profile = Profile::H265_MAIN;
        // } else if(profile == "mjpeg") {
        //     p.videoEncoding.profile = Profile::MJPEG;
        // }

        j.at("byteEncoding").at("enabled").get_to(p.byteEncoding.enabled);
        j.at("byteEncoding").at("compressionLevel").get_to(p.byteEncoding.compressionLevel);
        p.byteEncoding.compressionLevel = std::max(0, std::min(9, p.byteEncoding.compressionLevel));

        j.at("outputDir").get_to(p.outputDir);
    }
};
}  // namespace nlohmann
