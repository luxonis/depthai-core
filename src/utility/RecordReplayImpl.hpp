#include "depthai/utility/RecordReplay.hpp"
#include "mcap/mcap.hpp"

namespace dai {
namespace utility {
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
    ~ByteRecorder();
    void init(const std::string& filePath, RecordConfig::CompressionLevel compressionLevel, RecordType recordingType);
    template <typename T>
    void write(const T& data) {
        mcap::Timestamp writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
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
    void restart();
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
    void restart();
    void close();
    static std::optional<std::tuple<uint32_t, uint32_t>> getVideoSize(const std::string& filePath);
    bool isInitialized() const {
        return initialized;
    }

   private:
    mcap::McapReader reader;
    std::unique_ptr<mcap::LinearMessageView> messageView;
    std::unique_ptr<mcap::LinearMessageView::Iterator> it;
    bool initialized = false;
};

bool checkRecordConfig(std::string& recordPath, RecordConfig& config);

bool allMatch(const std::vector<std::string>& v1, const std::vector<std::string>& v2);

std::string matchTo(const std::vector<std::string>& mxIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames);
}  // namespace utility
}  // namespace dai
