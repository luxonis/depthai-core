#pragma once

#include <spdlog/logger.h>
#include <cstdint>

#include "span.hpp"

#include "nlohmann/json.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

#define DEPTHAI_RECORD_OPENCV 1

namespace dai {
namespace utility {

// TODO(asahtik): Record frame metadata
class VideoRecorder {
   public:
    VideoRecorder() = default;
    virtual ~VideoRecorder() = default;
    virtual bool init(std::string filePath, int width, int height, int fps) = 0;
    virtual bool write(span<const uint8_t>&) = 0;
    virtual bool close() = 0;
    bool isInitialized() const {
        return initialized;
    }

   private:
    bool initialized = false;
};

// TODO(asahtik): Add mp4v2
class VideoRecorderMp4v2 : public VideoRecorder {
   public:
    ~VideoRecorderMp4v2() override;
    bool init(std::string filePath, int width, int height, int fps) override;
    bool write(span<const uint8_t>&) override;
    bool close() override;
};

#if DEPTHAI_RECORD_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
class VideoRecorderOpenCV : public VideoRecorder {
   public:
    VideoRecorderOpenCV(const std::string& filename, int width, int height, int fps, int bitrate, int quality, bool lossless, int profile);
    ~VideoRecorderOpenCV() override;
    bool init(std::string filePath, int width, int heigh, int fps) override;
    bool write(span<const uint8_t>&) override;
    bool close() override;

   private:
    cv::VideoWriter writer;
    std::string filePath;
    int width, height, fps;
};
#endif

// TODO(asahtik): Replay frame metadata
class VideoPlayer {
   public:
    VideoPlayer() = default;
    virtual ~VideoPlayer() = default;
    virtual bool init(const std::string& filePath) = 0;
    virtual std::vector<uint8_t> next() = 0;
    virtual bool close() = 0;
    bool isInitialized() const {
        return initialized;
    }

   private:
    bool initialized = false;
};

class VideoPlayerMp4v2 : public VideoPlayer {
   public:
    ~VideoPlayerMp4v2() override;
    bool init(const std::string& filePath) override;
    std::vector<uint8_t> next() override;
    bool close() override;
};

// #if DEPTHAI_RECORD_OPENCV && defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
// class VideoRecorderOpenCV : public VideoRecorder {
//    public:
//     VideoRecorderOpenCV(const std::string& filename, int width, int height, int fps, int bitrate, int quality, bool lossless, int profile);
//     ~VideoRecorderOpenCV() override;
//     bool init(std::string filePath, int width, int heigh, int fps) override;
//     bool write(span<const uint8_t>&) override;
//     bool close() override;
//
//    private:
//     cv::VideoWriter writer;
//     std::string filePath;
//     int width, height, fps;
// };
// #endif

struct NodeRecordParams {
    std::string name;
};

struct RecordConfig {
    enum class RecordReplayState {RECORD, REPLAY, NONE};

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
