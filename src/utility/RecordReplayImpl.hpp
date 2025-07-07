#include "depthai/utility/RecordReplay.hpp"
#include "mcap/mcap.hpp"
#ifdef DEPTHAI_ENABLE_MP4V2
    #include <mp4v2.h>
#endif
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include <google/protobuf/descriptor.h>
#endif
#include "utility/span.hpp"
namespace dai {
namespace utility {

#ifdef DEPTHAI_ENABLE_PROTOBUF
mcap::Schema createSchema(const google::protobuf::Descriptor* d);
#endif

class VideoRecorder {
   public:
    enum class VideoCodec { H264, MJPEG, RAW };

    ~VideoRecorder();
    void init(const std::string& filePath, unsigned int width, unsigned int height, unsigned int fps, VideoCodec codec);
    void write(span<uint8_t>&, const uint32_t stride = 0);
    void close();
    bool isInitialized() const {
        return initialized;
    }

   private:
    bool initialized = false;
    unsigned int fps = 0;
    unsigned int width = 0;
    unsigned int height = 0;
    VideoCodec codec = VideoCodec::RAW;
#ifdef DEPTHAI_ENABLE_MP4V2
    MP4FileHandle mp4Writer = MP4_INVALID_FILE_HANDLE;
    MP4TrackId mp4Track = MP4_INVALID_TRACK_ID;
#endif
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<cv::VideoWriter> cvWriter;
#endif
};

class ByteRecorder {
   private:
    void setWriter(const std::string& filePath, RecordConfig::CompressionLevel compressionLevel);

   public:
    ~ByteRecorder();
    template <typename T>
    void init(const std::string& filePath, RecordConfig::CompressionLevel compressionLevel, const std::string& channelName) {
#ifdef DEPTHAI_ENABLE_PROTOBUF
        if(initialized) {
            throw std::runtime_error("ByteRecorder already initialized");
        }
        if(filePath.empty()) {
            throw std::runtime_error("ByteRecorder file path is empty");
        }
        setWriter(filePath, compressionLevel);
        {
            mcap::Schema schema = createSchema(T::descriptor());
            writer.addSchema(schema);
            mcap::Channel channel(channelName, "protobuf", schema.id);
            writer.addChannel(channel);
            channelId = channel.id;
        }

        initialized = true;
#else
        throw std::runtime_error("ByteRecorder not supported without protobuf support");
#endif
    }
    void write(std::vector<uint8_t> data) {
        mcap::Timestamp writeTime = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        mcap::Message msg;
        msg.channelId = channelId;
        msg.logTime = writeTime;
        msg.publishTime = writeTime;
        msg.sequence = index++;
        msg.data = reinterpret_cast<const std::byte*>(data.data());
        msg.dataSize = data.size();
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
    std::string init(const std::string& filePath);
    template <typename T>
    std::optional<T> next() {
        if(!initialized) {
            throw std::runtime_error("BytePlayer not initialized");
        }
        if(*it == messageView->end()) return std::nullopt;
        if((*it)->channel->messageEncoding != "protobuf") {
            throw std::runtime_error("Unsupported message encoding: " + (*it)->channel->messageEncoding);
        }

        T data;
        if(!data.ParseFromArray(reinterpret_cast<const char*>((*it)->message.data), (*it)->message.dataSize)) {
            throw std::runtime_error("Failed to parse protobuf message");
        }

        ++(*it);

        return data;
    }
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

bool checkRecordConfig(std::filesystem::path& recordPath, RecordConfig& config);

bool allMatch(const std::vector<std::string>& v1, const std::vector<std::string>& v2);

std::string matchTo(const std::vector<std::string>& deviceIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames);

std::tuple<size_t, size_t> getVideoSize(const std::string& filePath);

}  // namespace utility
}  // namespace dai
