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
#include "mcap/mcap.hpp"

namespace dai {
namespace utility {

struct NodeRecordParams {
    std::string name;
};

class VideoRecorder {
   public:
    enum class VideoCodec { H264, MJPEG, RAW };

    ~VideoRecorder();
    void init(const std::string& filePath, unsigned int width, unsigned int height, unsigned int fps, VideoCodec codec);
    void write(span<uint8_t>&);
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

class ByteRecorder {
   public:
    enum class CompressionLevel : uint8_t { NONE, FASTEST, FAST, DEFAULT, SLOW, SLOWEST };

    ~ByteRecorder();
    void init(const std::string& filePath, CompressionLevel compressionLevel, RecordType recordingType);
    template <typename T>
    void write(const T& data) {
        mcap::Timestamp writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
        nlohmann::json j = data;
        std::string serialized = j.dump();
        mcap::Message msg;
        msg.channelId = channelId;
        msg.logTime = writeTime;
        msg.publishTime = writeTime;
        msg.sequence = index++;
        msg.data = reinterpret_cast<const std::byte*>(serialized.data());
        msg.dataSize = serialized.size();
        const auto res = writer.write(msg);
        if(!res.ok()) {
            writer.close();
            throw std::runtime_error("Failed to write video frame metadata: " + res.message);
        }
    }
    void close();
    bool isInitialized() const {
        return initialized;
    }

   private:
    bool initialized = false;
    std::ofstream file;

    uint64_t index = 0;
    mcap::McapWriter writer;
    mcap::ChannelId channelId;
};

class VideoPlayer {
   public:
    ~VideoPlayer();
    void init(const std::string& filePath);
    void setSize(uint32_t width, uint32_t height);
    std::optional<std::vector<uint8_t>> next();
    std::tuple<uint32_t, uint32_t> size();
    void close();
    bool isInitialized() const {
        return initialized;
    }

   private:
    uint32_t width = 0;
    uint32_t height = 0;
    bool initialized = false;
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<cv::VideoCapture> cvReader;
#else
    std::unique_ptr<int> cvReader;
#endif
};

class BytePlayer {
   public:
    ~BytePlayer();
    void init(const std::string& filePath);
    std::optional<nlohmann::json> next();
    void close();
    bool isInitialized() const {
        return initialized;
    }

   private:
    mcap::McapReader reader;
    std::unique_ptr<mcap::LinearMessageView> messageView;
    std::unique_ptr<mcap::LinearMessageView::Iterator> it;
    bool initialized = false;
};

struct RecordConfig {
    using Profile = dai::VideoEncoderProperties::Profile;
    enum class RecordReplayState { RECORD, REPLAY, NONE };

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
    uint8_t compressionLevel = 5;
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
        std::string profile;
        switch(p.videoEncoding.profile) {
            case dai::utility::RecordConfig::Profile::H264_BASELINE:
                profile = "H264_BASELINE";
                break;
            case dai::utility::RecordConfig::Profile::H264_HIGH:
                profile = "H264_HIGH";
                break;
            case dai::utility::RecordConfig::Profile::H264_MAIN:
                profile = "H264_MAIN";
                break;
            case dai::utility::RecordConfig::Profile::H265_MAIN:
                profile = "H265_MAIN";
                break;
            case dai::utility::RecordConfig::Profile::MJPEG:
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

    static void from_json(const json& j, dai::utility::RecordConfig& p) {  // NOLINT this is a specialization, naming conventions don't apply
        std::string profile;
        j.at("videoEncoding").at("enabled").get_to(p.videoEncoding.enabled);
        j.at("videoEncoding").at("bitrate").get_to(p.videoEncoding.bitrate);
        j.at("videoEncoding").at("profile").get_to(profile);
        j.at("videoEncoding").at("lossless").get_to(p.videoEncoding.lossless);
        j.at("videoEncoding").at("quality").get_to(p.videoEncoding.quality);
        std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);

        p.videoEncoding.profile = dai::utility::RecordConfig::Profile::MJPEG;
        if(profile == "h264_baseline" || profile == "avc_baseline") {
            p.videoEncoding.profile = dai::utility::RecordConfig::Profile::H264_BASELINE;
        } else if(profile == "h264_high" || profile == "avc_high") {
            p.videoEncoding.profile = dai::utility::RecordConfig::Profile::H264_HIGH;
        } else if(profile == "h264_main" || profile == "avc_main" || profile == "h264" || profile == "avc") {
            p.videoEncoding.profile = dai::utility::RecordConfig::Profile::H264_MAIN;
        } else if(profile == "h265_main" || profile == "hevc_main" || profile == "h265" || profile == "hevc") {
            p.videoEncoding.profile = dai::utility::RecordConfig::Profile::H265_MAIN;
        } else if(profile == "mjpeg") {
            p.videoEncoding.profile = dai::utility::RecordConfig::Profile::MJPEG;
        }

        j.at("compressionLevel").get_to(p.compressionLevel);
        p.compressionLevel = std::max((uint8_t)0, std::min((uint8_t)5, p.compressionLevel));

        j.at("outputDir").get_to(p.outputDir);
    }
};
}  // namespace nlohmann
