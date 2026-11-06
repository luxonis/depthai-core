#include <algorithm>
#include <cstring>
#include <fstream>
#include <magic_enum/magic_enum.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <set>
#include <stdexcept>

#include "ReplayInternal.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "utility/RecordReplayImpl.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/common.pb.h"
#endif

#ifdef DEPTHAI_HAVE_LIBAV
extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libavutil/avutil.h>
    #include <libavutil/imgutils.h>
}
#endif

namespace dai {
namespace replay {
namespace {

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error(message);
}

std::string lowerExt(const std::filesystem::path& path) {
    auto ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    const auto name = path.filename().string();
    auto lowered = name;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    if(lowered.size() >= 7 && lowered.substr(lowered.size() - 7) == ".tar.gz") return ".tar.gz";
    if(lowered.size() >= 7 && lowered.substr(lowered.size() - 7) == ".tar.xz") return ".tar.xz";
    return ext;
}

bool isArchiveName(const std::filesystem::path& path) {
    const auto ext = lowerExt(path);
    return ext == ".tar" || ext == ".tgz" || ext == ".tar.gz" || ext == ".tar.xz";
}

bool isVideoName(const std::filesystem::path& path) {
    const auto ext = lowerExt(path);
    return ext == ".mp4" || ext == ".avi";
}

std::uint64_t fileSizeOf(const std::filesystem::path& path) {
    std::error_code error;
    const auto size = std::filesystem::file_size(path, error);
    if(error) return 0;
    return static_cast<std::uint64_t>(size);
}

int clamp8(int value) {
    return std::max(0, std::min(255, value));
}

void limitedRgbToYuv(int r, int g, int b, std::uint8_t& y, std::uint8_t& u, std::uint8_t& v) {
    y = static_cast<std::uint8_t>(clamp8(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16));
    u = static_cast<std::uint8_t>(clamp8(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128));
    v = static_cast<std::uint8_t>(clamp8(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128));
}

void fullRgbToYuv(int r, int g, int b, std::uint8_t& y, std::uint8_t& u, std::uint8_t& v) {
    y = static_cast<std::uint8_t>(clamp8((299 * r + 587 * g + 114 * b + 500) / 1000));
    u = static_cast<std::uint8_t>(clamp8(((-169 * r - 331 * g + 500 * b + 500) / 1000) + 128));
    v = static_cast<std::uint8_t>(clamp8(((500 * r - 419 * g - 81 * b + 500) / 1000) + 128));
}

void yuvToRgb(int y, int u, int v, int& r, int& g, int& b) {
    const int c = y - 16;
    const int d = u - 128;
    const int e = v - 128;
    r = clamp8((1164 * c + 1596 * e + 500) / 1000);
    g = clamp8((1164 * c - 391 * d - 813 * e + 500) / 1000);
    b = clamp8((1164 * c + 2018 * d + 500) / 1000);
}

void checkPngHeader(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    unsigned char header[26] = {};
    in.read(reinterpret_cast<char*>(header), 26);
    static const unsigned char kSig[8] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
    if(!in || std::memcmp(header, kSig, 8) != 0) fail("ReplayVideo png header is not valid: " + path.string());
    const int bitDepth = header[24];
    const int colorType = header[25];
    if(bitDepth == 16) fail("ReplayVideo png bit depth 16 is not supported: " + path.string());
    if(colorType == 3) fail("ReplayVideo png palette is not supported: " + path.string());
}

std::pair<int, int> jpegSize(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    unsigned char marker[2] = {};
    in.read(reinterpret_cast<char*>(marker), 2);
    if(!in || marker[0] != 0xff || marker[1] != 0xd8) fail("ReplayVideo jpeg header is not valid: " + path.string());
    while(in) {
        unsigned char byte = 0;
        do {
            in.read(reinterpret_cast<char*>(&byte), 1);
        } while(in && byte != 0xff);
        do {
            in.read(reinterpret_cast<char*>(&byte), 1);
        } while(in && byte == 0xff);
        if(!in) break;
        if(byte == 0xd9 || byte == 0xda) break;
        unsigned char lengthBytes[2] = {};
        in.read(reinterpret_cast<char*>(lengthBytes), 2);
        const int length = (lengthBytes[0] << 8) | lengthBytes[1];
        if(length < 2) fail("ReplayVideo jpeg header is not valid: " + path.string());
        if((byte >= 0xc0 && byte <= 0xc3) || (byte >= 0xc5 && byte <= 0xc7) || (byte >= 0xc9 && byte <= 0xcb) || (byte >= 0xcd && byte <= 0xcf)) {
            unsigned char sof[5] = {};
            in.read(reinterpret_cast<char*>(sof), 5);
            const int height = (sof[1] << 8) | sof[2];
            const int width = (sof[3] << 8) | sof[4];
            return {width, height};
        }
        in.seekg(length - 2, std::ios::cur);
    }
    fail("ReplayVideo jpeg header is not valid: " + path.string());
}

struct DaiInfo {
    bool encoded = false;
    bool mjpeg = false;
    ImgFrame::Type type = ImgFrame::Type::NONE;
    int width = 0;
    int height = 0;
    DecodedPixels pixels;
};

DaiInfo readDai(const std::filesystem::path& path) {
#ifndef DEPTHAI_ENABLE_PROTOBUF
    (void)path;
    fail("ReplayVideo node requires protobuf support");
#else
    std::ifstream file(path, std::ios::binary);
    if(!file) fail("ReplayVideo source cannot be read: " + path.string());
    proto::common::ProtoSerializableMessage envelope;
    if(!envelope.ParseFromIstream(&file)) fail("ReplayVideo dai envelope is not valid: " + path.string());
    ImgFrame imgProbe;
    EncodedFrame encProbe;
    DaiInfo info;
    if(envelope.schema_name() == imgProbe.serializeSchema().schemaName) {
        ImgFrame frame;
        frame.load(path);
        if(frame.getType() == ImgFrame::Type::BITSTREAM) fail("ReplayVideo BITSTREAM is not supported: " + path.string());
        info.type = frame.getType();
        info.width = static_cast<int>(frame.getWidth());
        info.height = static_cast<int>(frame.getHeight());
        info.pixels.width = info.width;
        info.pixels.height = info.height;
        info.pixels.type = info.type;
        info.pixels.passthrough = !isReplayOutType(info.type);
        info.pixels.fb = frame.fb;
        info.pixels.sourceFb = frame.sourceFb;
        info.pixels.cam = frame.cam;
        info.pixels.transformation = frame.transformation;
        info.pixels.instanceNum = frame.instanceNum;
        info.pixels.category = frame.category;
        info.pixels.hasTiming = true;
        info.pixels.sequenceNum = frame.getSequenceNum();
        info.pixels.timestamp = frame.getTimestamp();
        info.pixels.timestampDevice = frame.getTimestampDevice();
        info.pixels.timestampSystem = frame.getTimestampSystem();
        const auto span = frame.getData();
        info.pixels.data.assign(span.begin(), span.end());
        if(info.type == ImgFrame::Type::YUV420p || info.type == ImgFrame::Type::NV12 || info.type == ImgFrame::Type::GRAY8
           || info.type == ImgFrame::Type::RGB888i || info.type == ImgFrame::Type::BGR888i) {
            info.pixels.passthrough = false;
        }
        return info;
    }
    if(envelope.schema_name() == encProbe.serializeSchema().schemaName) {
        EncodedFrame encoded;
        encoded.load(path);
        if(encoded.profile == EncodedFrame::Profile::AVC || encoded.profile == EncodedFrame::Profile::HEVC) {
            fail("ReplayVideo H264 or H265 EncodedFrame is not supported in .dai: " + path.string());
        }
        info.encoded = true;
        info.mjpeg = true;
        info.pixels.fullRange = true;
        const auto span = encoded.getData();
        std::vector<std::uint8_t> bytes(span.begin(), span.end());
        cv::Mat buffer(1, static_cast<int>(bytes.size()), CV_8UC1, bytes.data());
        cv::Mat image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED | cv::IMREAD_IGNORE_ORIENTATION);
        if(image.empty()) fail("ReplayVideo jpeg data is not valid: " + path.string());
        info.width = image.cols;
        info.height = image.rows;
        info.pixels.width = info.width;
        info.pixels.height = info.height;
        info.type = ImgFrame::Type::NV12;
        info.pixels.type = ImgFrame::Type::NV12;
        if(image.channels() == 1) {
            info.pixels.data.assign(image.datastart, image.dataend);
            info.pixels.type = ImgFrame::Type::GRAY8;
            info.type = ImgFrame::Type::GRAY8;
        } else {
            if(image.channels() == 4) cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
            info.pixels.data.assign(image.datastart, image.dataend);
            info.pixels.type = ImgFrame::Type::BGR888i;
            info.type = ImgFrame::Type::BGR888i;
        }
        info.pixels.fullRange = true;
        info.pixels.hasTiming = true;
        info.pixels.sequenceNum = encoded.getSequenceNum();
        info.pixels.timestamp = encoded.getTimestamp();
        info.pixels.timestampDevice = encoded.getTimestampDevice();
        info.pixels.timestampSystem = encoded.getTimestampSystem();
        return info;
    }
    fail("ReplayVideo dai message type is not supported: " + path.string());
#endif
}

ListedSource listFrameDirectory(const std::filesystem::path& directory) {
    std::vector<std::pair<std::filesystem::path, std::int64_t>> frames;
    std::set<std::string> prefixes;
    std::set<std::string> extensions;
    std::set<std::int64_t> seen;
    for(const auto& entry : std::filesystem::directory_iterator(directory)) {
        if(!entry.is_regular_file()) continue;
        const auto filename = entry.path().filename().string();
        if(filename.empty() || filename.front() == '.') continue;
        auto ext = lowerExt(entry.path());
        if(ext == ".jpeg") ext = ".jpg";
        if(ext != ".dai" && ext != ".jpg" && ext != ".png") continue;
        const auto stem = entry.path().stem().string();
        const auto split = stem.rfind('_');
        if(split == std::string::npos || split == 0 || split + 1 >= stem.size()) continue;
        const auto seqText = stem.substr(split + 1);
        if(!std::all_of(seqText.begin(), seqText.end(), [](unsigned char character) { return std::isdigit(character) != 0; })) continue;
        const auto sequence = std::stoll(seqText);
        if(!seen.insert(sequence).second) fail("ReplayVideo duplicate frame number " + std::to_string(sequence) + " in " + directory.string());
        prefixes.insert(stem.substr(0, split));
        extensions.insert(ext);
        frames.emplace_back(entry.path(), sequence);
    }
    if(frames.empty()) fail("ReplayVideo source is empty: " + directory.string());
    if(prefixes.size() != 1) {
        std::string listed;
        for(const auto& prefix : prefixes) {
            if(!listed.empty()) listed += ", ";
            listed += prefix;
        }
        fail("ReplayVideo directory has more than one stream: " + listed);
    }
    if(extensions.size() != 1) fail("ReplayVideo directory has more than one extension: " + directory.string());
    std::sort(frames.begin(), frames.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
    ListedSource source;
    source.kind = ListedSource::Kind::Frames;
    source.path = directory;
    source.stream = *prefixes.begin();
    source.dai = *extensions.begin() == ".dai";
    source.jpegOrPng = !source.dai;
    const auto metadata = directory / (source.stream + ".mcap");
    if(std::filesystem::exists(metadata)) source.autoMetadata = metadata;
    if(source.dai) {
        const auto first = readDai(frames.front().first);
        source.width = first.width;
        source.height = first.height;
        source.sourceType = first.type;
        source.passthrough = first.pixels.passthrough && !first.mjpeg;
        source.mjpegDai = first.mjpeg;
        if(first.mjpeg) {
            source.sourceType = ImgFrame::Type::NV12;
            source.passthrough = false;
        }
        for(std::size_t i = 1; i < frames.size(); i++) {
            const auto other = readDai(frames[i].first);
            if(other.width != source.width || other.height != source.height) {
                fail("ReplayVideo frame size does not match: " + frames[i].first.string() + " is " + std::to_string(other.width) + "x"
                     + std::to_string(other.height));
            }
            if(other.type != source.sourceType && !first.mjpeg) {
                fail("ReplayVideo frame type does not match: " + frames[i].first.string());
            }
        }
    } else {
        const bool png = *extensions.begin() == ".png";
        if(png) checkPngHeader(frames.front().first);
        const auto size = png ? [](const std::filesystem::path& path) {
            checkPngHeader(path);
            const auto image = cv::imread(path.string(), cv::IMREAD_UNCHANGED | cv::IMREAD_IGNORE_ORIENTATION);
            if(image.empty()) fail("ReplayVideo png header is not valid: " + path.string());
            return std::pair<int, int>{image.cols, image.rows};
        }(frames.front().first)
                              : jpegSize(frames.front().first);
        source.width = size.first;
        source.height = size.second;
        source.sourceType = ImgFrame::Type::NV12;
        for(std::size_t i = 1; i < frames.size(); i++) {
            const auto other = png ? [&]() {
                checkPngHeader(frames[i].first);
                const auto image = cv::imread(frames[i].first.string(), cv::IMREAD_UNCHANGED | cv::IMREAD_IGNORE_ORIENTATION);
                if(image.empty()) fail("ReplayVideo png header is not valid: " + frames[i].first.string());
                return std::pair<int, int>{image.cols, image.rows};
            }()
                                   : jpegSize(frames[i].first);
            if(other.first != source.width || other.second != source.height) {
                fail("ReplayVideo frame size does not match: " + frames[i].first.string() + " is " + std::to_string(other.first) + "x"
                     + std::to_string(other.second));
            }
        }
    }
    source.frameCount = static_cast<int>(frames.size());
    source.bytes = 0;
    for(const auto& frame : frames) {
        source.frames.push_back(ListedFrame{frame.first, frame.second});
        source.bytes += fileSizeOf(frame.first);
    }
    for(const auto& entry : std::filesystem::directory_iterator(directory)) {
        if(entry.is_regular_file()) source.bytes += 0;
    }
    source.bytes = 0;
    for(const auto& entry : std::filesystem::directory_iterator(directory)) {
        if(!entry.is_regular_file()) continue;
        const auto name = entry.path().filename().string();
        if(name.empty() || name.front() == '.') continue;
        source.bytes += fileSizeOf(entry.path());
    }
    return source;
}

#ifdef DEPTHAI_HAVE_LIBAV
struct LibavVideo {
    AVFormatContext* format = nullptr;
    AVCodecContext* codec = nullptr;
    AVPacket* packet = nullptr;
    AVFrame* frame = nullptr;
    int streamIndex = -1;
    int next = 0;
    ~LibavVideo() {
        if(frame) av_frame_free(&frame);
        if(packet) av_packet_free(&packet);
        if(codec) avcodec_free_context(&codec);
        if(format) avformat_close_input(&format);
    }
};

std::string codecName(AVCodecID id) {
    const char* name = avcodec_get_name(id);
    return name ? name : "unknown";
}

void openLibav(LibavVideo& video, const std::filesystem::path& path) {
    if(avformat_open_input(&video.format, path.string().c_str(), nullptr, nullptr) < 0) {
        fail("ReplayVideo video file cannot be opened: " + path.string());
    }
    if(avformat_find_stream_info(video.format, nullptr) < 0) fail("ReplayVideo video file cannot be opened: " + path.string());
    video.streamIndex = av_find_best_stream(video.format, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if(video.streamIndex < 0) fail("ReplayVideo video file has no video track: " + path.string());
    AVStream* stream = video.format->streams[video.streamIndex];
    const AVCodec* decoder = avcodec_find_decoder(stream->codecpar->codec_id);
    if(decoder == nullptr) fail("ReplayVideo missing decoder support for " + codecName(stream->codecpar->codec_id));
    video.codec = avcodec_alloc_context3(decoder);
    if(avcodec_parameters_to_context(video.codec, stream->codecpar) < 0 || avcodec_open2(video.codec, decoder, nullptr) < 0) {
        fail("ReplayVideo missing decoder support for " + codecName(stream->codecpar->codec_id));
    }
    video.packet = av_packet_alloc();
    video.frame = av_frame_alloc();
}

ListedSource probeLibav(const std::filesystem::path& path) {
    LibavVideo video;
    openLibav(video, path);
    AVStream* stream = video.format->streams[video.streamIndex];
    ListedSource source;
    source.kind = ListedSource::Kind::Video;
    source.path = path;
    source.stream = path.stem().string();
    source.width = video.codec->width;
    source.height = video.codec->height;
    source.codec = codecName(stream->codecpar->codec_id);
    if(const char* profile = avcodec_profile_name(stream->codecpar->codec_id, stream->codecpar->profile)) source.profile = profile;
    if(stream->avg_frame_rate.den != 0) source.fps = av_q2d(stream->avg_frame_rate);
    const auto format = video.codec->pix_fmt;
    const auto* descriptor = av_pix_fmt_desc_get(format);
    const int bitDepth = descriptor ? descriptor->comp[0].depth : 8;
    const bool yuv420 = format == AV_PIX_FMT_YUV420P || format == AV_PIX_FMT_YUVJ420P;
    if(source.codec != "h264" && source.codec != "hevc" && source.codec != "mjpeg" && source.codec != "ffv1") {
        fail("ReplayVideo codec " + source.codec + " is not supported");
    }
    if(!yuv420 || bitDepth != 8) {
        const auto name = source.profile.empty() ? source.codec : source.profile;
        fail("ReplayVideo profile " + name + " is not supported");
    }
    if(stream->nb_frames > 0)
        source.frameCount = static_cast<int>(stream->nb_frames);
    else {
        int packets = 0;
        while(av_read_frame(video.format, video.packet) >= 0) {
            if(video.packet->stream_index == video.streamIndex) packets++;
            av_packet_unref(video.packet);
        }
        source.frameCount = packets;
    }
    if(source.frameCount <= 0 || source.width <= 0 || source.height <= 0) fail("ReplayVideo source is empty: " + path.string());
    source.sourceType = ImgFrame::Type::YUV420p;
    source.bytes = fileSizeOf(path);
    const auto metadata = path.parent_path() / (source.stream + ".mcap");
    if(std::filesystem::exists(metadata)) source.autoMetadata = metadata;
    return source;
}

DecodedPixels readLibavFrame(LibavVideo& video) {
    bool flushed = false;
    while(true) {
        const int received = avcodec_receive_frame(video.codec, video.frame);
        if(received == 0) {
            if(video.frame->format != AV_PIX_FMT_YUV420P && video.frame->format != AV_PIX_FMT_YUVJ420P) {
                fail("ReplayVideo profile " + std::string(av_get_pix_fmt_name(static_cast<AVPixelFormat>(video.frame->format))) + " is not supported");
            }
            const int width = video.frame->width;
            const int height = video.frame->height;
            DecodedPixels pixels;
            pixels.width = width;
            pixels.height = height;
            pixels.type = ImgFrame::Type::YUV420p;
            pixels.fullRange = video.frame->format == AV_PIX_FMT_YUVJ420P;
            const int ySize = width * height;
            const int cSize = ySize / 4;
            pixels.data.resize(static_cast<std::size_t>(ySize + 2 * cSize));
            for(int row = 0; row < height; row++) {
                std::memcpy(pixels.data.data() + static_cast<std::size_t>(row * width), video.frame->data[0] + row * video.frame->linesize[0], width);
            }
            for(int row = 0; row < height / 2; row++) {
                std::memcpy(
                    pixels.data.data() + static_cast<std::size_t>(ySize + row * (width / 2)), video.frame->data[1] + row * video.frame->linesize[1], width / 2);
                std::memcpy(pixels.data.data() + static_cast<std::size_t>(ySize + cSize + row * (width / 2)),
                            video.frame->data[2] + row * video.frame->linesize[2],
                            width / 2);
            }
            video.next++;
            return pixels;
        }
        if(received != AVERROR(EAGAIN)) fail("ReplayVideo frame decode failed");
        if(av_read_frame(video.format, video.packet) < 0) {
            if(flushed) fail("ReplayVideo frame decode failed");
            avcodec_send_packet(video.codec, nullptr);
            flushed = true;
            continue;
        }
        if(video.packet->stream_index == video.streamIndex) avcodec_send_packet(video.codec, video.packet);
        av_packet_unref(video.packet);
    }
}
#endif

ListedSource probeOpenCv(const std::filesystem::path& path) {
    cv::VideoCapture capture(path.string(), cv::CAP_FFMPEG);
    if(!capture.isOpened()) fail("ReplayVideo missing decoder support for " + path.string());
    ListedSource source;
    source.kind = ListedSource::Kind::Video;
    source.path = path;
    source.stream = path.stem().string();
    source.width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    source.height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    source.fps = capture.get(cv::CAP_PROP_FPS);
    source.frameCount = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_COUNT));
    const int fourcc = static_cast<int>(capture.get(cv::CAP_PROP_FOURCC));
    char tag[5] = {static_cast<char>(fourcc & 0xff),
                   static_cast<char>((fourcc >> 8) & 0xff),
                   static_cast<char>((fourcc >> 16) & 0xff),
                   static_cast<char>((fourcc >> 24) & 0xff),
                   0};
    source.codec = tag;
    if(source.frameCount <= 0 || source.width <= 0 || source.height <= 0) fail("ReplayVideo source is empty: " + path.string());
    source.sourceType = ImgFrame::Type::YUV420p;
    source.bytes = fileSizeOf(path);
    const auto metadata = path.parent_path() / (source.stream + ".mcap");
    if(std::filesystem::exists(metadata)) source.autoMetadata = metadata;
    return source;
}

cv::Mat resizeMat(const cv::Mat& image, int width, int height) {
    if(image.cols == width && image.rows == height) return image;
    const bool down = static_cast<std::int64_t>(width) * height < static_cast<std::int64_t>(image.cols) * image.rows;
    cv::Mat out;
    cv::resize(image, out, cv::Size(width, height), 0, 0, down ? cv::INTER_AREA : cv::INTER_LINEAR);
    return out;
}

void packYuv(DecodedPixels& pixels, const cv::Mat& y, const cv::Mat& u, const cv::Mat& v, ImgFrame::Type type) {
    const int width = y.cols;
    const int height = y.rows;
    pixels.width = width;
    pixels.height = height;
    pixels.type = type;
    if(type == ImgFrame::Type::GRAY8) {
        pixels.data.assign(y.datastart, y.dataend);
        return;
    }
    const int ySize = width * height;
    const int cSize = ySize / 4;
    pixels.data.assign(static_cast<std::size_t>(ySize + (type == ImgFrame::Type::GRAY8 ? 0 : 2 * cSize)), 0);
    std::memcpy(pixels.data.data(), y.data, static_cast<std::size_t>(ySize));
    if(type == ImgFrame::Type::YUV420p) {
        std::memcpy(pixels.data.data() + ySize, u.data, static_cast<std::size_t>(cSize));
        std::memcpy(pixels.data.data() + ySize + cSize, v.data, static_cast<std::size_t>(cSize));
    } else if(type == ImgFrame::Type::NV12) {
        for(int i = 0; i < cSize; i++) {
            pixels.data[static_cast<std::size_t>(ySize + i * 2)] = u.data[i];
            pixels.data[static_cast<std::size_t>(ySize + i * 2 + 1)] = v.data[i];
        }
    }
}

void bgrToPlanes(const cv::Mat& bgr, bool fullRange, cv::Mat& y, cv::Mat& u, cv::Mat& v) {
    y.create(bgr.rows, bgr.cols, CV_8UC1);
    const int chromaRows = bgr.rows / 2 + bgr.rows % 2;
    const int chromaCols = bgr.cols / 2 + bgr.cols % 2;
    u.create(chromaRows, chromaCols, CV_8UC1);
    v.create(chromaRows, chromaCols, CV_8UC1);
    std::vector<int> uAcc(static_cast<std::size_t>(chromaRows) * chromaCols, 0);
    std::vector<int> vAcc = uAcc;
    for(int row = 0; row < bgr.rows; row++) {
        for(int col = 0; col < bgr.cols; col++) {
            const auto pixel = bgr.at<cv::Vec3b>(row, col);
            std::uint8_t yv = 0;
            std::uint8_t uv = 0;
            std::uint8_t vv = 0;
            if(fullRange)
                fullRgbToYuv(pixel[2], pixel[1], pixel[0], yv, uv, vv);
            else
                limitedRgbToYuv(pixel[2], pixel[1], pixel[0], yv, uv, vv);
            y.at<std::uint8_t>(row, col) = yv;
            if(row / 2 >= u.rows || col / 2 >= u.cols) continue;
            const auto index = static_cast<std::size_t>((row / 2) * u.cols + col / 2);
            uAcc[index] += uv;
            vAcc[index] += vv;
        }
    }
    for(int row = 0; row < u.rows; row++) {
        for(int col = 0; col < u.cols; col++) {
            const int count = std::min(2, bgr.rows - row * 2) * std::min(2, bgr.cols - col * 2);
            const auto index = static_cast<std::size_t>(row * u.cols + col);
            u.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>((uAcc[index] + count / 2) / count);
            v.at<std::uint8_t>(row, col) = static_cast<std::uint8_t>((vAcc[index] + count / 2) / count);
        }
    }
}

}  // namespace

bool isReplayOutType(ImgFrame::Type type) {
    return type == ImgFrame::Type::NV12 || type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RGB888i
           || type == ImgFrame::Type::BGR888i;
}

std::string frameTypeName(ImgFrame::Type type) {
    const auto name = magic_enum::enum_name(type);
    return name.empty() ? std::string("unsupported") : std::string(name);
}

ListedSource inspectReplaySource(const std::filesystem::path& path) {
    if(!std::filesystem::exists(path)) fail("ReplayVideo source is missing: " + path.string());
    if(std::filesystem::is_directory(path)) return listFrameDirectory(path);
    if(isArchiveName(path)) fail("ReplayVideo archive must be extracted before inspection: " + path.string());
    if(lowerExt(path) == ".mcap") {
        ListedSource source;
        source.kind = ListedSource::Kind::Mcap;
        source.path = path;
        source.stream = path.stem().string();
        source.bytes = fileSizeOf(path);
        if(source.bytes == 0) fail("ReplayVideo source is empty: " + path.string());
        return source;
    }
    if(!isVideoName(path)) fail("ReplayVideo unknown extension: " + path.extension().string());
    if(fileSizeOf(path) == 0) fail("ReplayVideo source is empty: " + path.string());
#ifdef DEPTHAI_HAVE_LIBAV
    return probeLibav(path);
#else
    return probeOpenCv(path);
#endif
}

std::filesystem::path resolveArchiveEntry(const std::filesystem::path& root, const std::string& entry, bool metadataOnly) {
    if(entry.empty()) return root;
    const auto direct = root / entry;
    if(!metadataOnly && std::filesystem::is_directory(direct)) return direct;
    if(metadataOnly) {
        const auto mcap = root / (entry + ".mcap");
        if(std::filesystem::exists(mcap)) return mcap;
    } else {
        for(const char* ext : {".mp4", ".avi"}) {
            const auto video = root / (entry + ext);
            if(std::filesystem::exists(video)) return video;
        }
        const auto mcap = root / (entry + ".mcap");
        if(std::filesystem::exists(mcap)) return mcap;
    }
    fail(std::string(metadataOnly ? "ReplayMetadataOnly" : "ReplayVideo") + " archive entry is missing: " + entry);
}

std::tuple<int, int, double> probeMediaSize(const std::filesystem::path& path) {
    if(std::filesystem::is_directory(path)) {
        const auto listed = listFrameDirectory(path);
        return {listed.width, listed.height, listed.fps};
    }
    const auto listed = inspectReplaySource(path);
    return {listed.width, listed.height, listed.fps};
}

struct FrameReader::State {
    ListedSource source;
    int next = 0;
#ifdef DEPTHAI_HAVE_LIBAV
    std::unique_ptr<LibavVideo> libav;
#endif
    cv::VideoCapture capture;
    bool opencv = false;
};

FrameReader::FrameReader() = default;
FrameReader::~FrameReader() = default;
FrameReader::FrameReader(FrameReader&&) noexcept = default;
FrameReader& FrameReader::operator=(FrameReader&&) noexcept = default;

void FrameReader::open(const ListedSource& source) {
    // Copy first. A loop reopens with state->source, and replacing state frees that object.
    ListedSource kept = source;
    state = std::make_unique<State>();
    state->source = std::move(kept);
    if(state->source.kind == ListedSource::Kind::Video) {
#ifdef DEPTHAI_HAVE_LIBAV
        state->libav = std::make_unique<LibavVideo>();
        openLibav(*state->libav, state->source.path);
#else
        state->opencv = true;
        state->capture.open(state->source.path.string(), cv::CAP_FFMPEG);
        if(!state->capture.isOpened()) fail("ReplayVideo missing decoder support for " + state->source.path.string());
#endif
    }
}

int FrameReader::count() const {
    return state ? state->source.frameCount : 0;
}

DecodedPixels FrameReader::read(int index) {
    if(!state) fail("ReplayVideo reader is not open");
    if(index < state->next) open(state->source);
    if(state->source.kind == ListedSource::Kind::Frames) {
        if(index < 0 || index >= static_cast<int>(state->source.frames.size())) fail("ReplayVideo frame is missing");
        const auto& path = state->source.frames[static_cast<std::size_t>(index)].path;
        const auto ext = lowerExt(path);
        DecodedPixels pixels;
        if(ext == ".dai") {
            auto dai = readDai(path);
            pixels = std::move(dai.pixels);
            if(dai.mjpeg && pixels.type == ImgFrame::Type::BGR888i) pixels.fullRange = true;
        } else {
            if(ext == ".png") checkPngHeader(path);
            cv::Mat image = cv::imread(path.string(), cv::IMREAD_UNCHANGED | cv::IMREAD_IGNORE_ORIENTATION);
            if(image.empty()) fail("ReplayVideo image cannot be read: " + path.string());
            pixels.width = image.cols;
            pixels.height = image.rows;
            pixels.fullRange = ext != ".png";
            if(image.channels() == 4) cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
            if(image.channels() == 1) {
                pixels.type = ImgFrame::Type::GRAY8;
                pixels.data.assign(image.datastart, image.dataend);
                pixels.fullRange = false;
            } else {
                pixels.type = ImgFrame::Type::BGR888i;
                if(!image.isContinuous()) image = image.clone();
                pixels.data.assign(image.datastart, image.dataend);
            }
        }
        state->next = index + 1;
        const auto expected = state->source.frames[static_cast<std::size_t>(index)].path;
        if(!std::filesystem::exists(expected)) fail("ReplayVideo frame changed during replay: " + expected.string());
        return pixels;
    }
#ifdef DEPTHAI_HAVE_LIBAV
    if(state->libav) {
        while(state->libav->next <= index) {
            auto pixels = readLibavFrame(*state->libav);
            if(state->libav->next - 1 == index) {
                state->next = index + 1;
                return pixels;
            }
        }
    }
#endif
    if(state->opencv) {
        while(state->next <= index) {
            cv::Mat frame;
            if(!state->capture.read(frame) || frame.empty()) fail("ReplayVideo frame decode failed");
            DecodedPixels pixels;
            pixels.width = frame.cols;
            pixels.height = frame.rows;
            pixels.type = ImgFrame::Type::BGR888i;
            pixels.fullRange = false;
            if(!frame.isContinuous()) frame = frame.clone();
            pixels.data.assign(frame.datastart, frame.dataend);
            state->next++;
            if(state->next - 1 == index) return pixels;
        }
    }
    fail("ReplayVideo frame decode failed");
}

DecodedPixels convertPixels(const DecodedPixels& pixels, ImgFrame::Type outType, int outW, int outH) {
    if(pixels.passthrough) return pixels;
    if((outW % 2) != 0 || (outH % 2) != 0) {
        fail("ReplayVideo size " + std::to_string(outW) + "x" + std::to_string(outH) + " is not supported");
    }
    cv::Mat y;
    cv::Mat u;
    cv::Mat v;
    cv::Mat bgr;
    const int width = pixels.width;
    const int height = pixels.height;
    if(width <= 0 || height <= 0 || outW <= 0 || outH <= 0) fail("ReplayVideo frame dimensions are invalid");
    const bool explicitOffsets = pixels.fb.p1Offset != 0 || pixels.fb.p2Offset != 0 || pixels.fb.p3Offset != 0;
    const auto firstOffset = static_cast<std::size_t>(pixels.fb.p1Offset);
    const auto rowStride = pixels.fb.stride != 0
                               ? static_cast<std::size_t>(pixels.fb.stride)
                               : static_cast<std::size_t>(width) * ((pixels.type == ImgFrame::Type::RGB888i || pixels.type == ImgFrame::Type::BGR888i) ? 3 : 1);
    const auto plane = [&](int rows, int cols, int type, std::size_t offset, std::size_t stride) {
        const auto rowBytes = static_cast<std::size_t>(cols) * CV_ELEM_SIZE(type);
        if(rows <= 0 || cols <= 0 || stride < rowBytes || offset > pixels.data.size() || pixels.data.size() - offset < rowBytes
           || static_cast<std::size_t>(rows - 1) > (pixels.data.size() - offset - rowBytes) / stride) {
            fail("ReplayVideo frame payload is too small or has an invalid plane layout");
        }
        return cv::Mat(rows, cols, type, const_cast<std::uint8_t*>(pixels.data.data() + offset), stride).clone();
    };
    const int chromaWidth = width / 2 + width % 2;
    const int chromaHeight = height / 2 + height % 2;
    if(pixels.type == ImgFrame::Type::GRAY8) {
        y = plane(height, width, CV_8UC1, firstOffset, rowStride);
        u = cv::Mat(chromaHeight, chromaWidth, CV_8UC1, cv::Scalar(128));
        v = u.clone();
    } else if(pixels.type == ImgFrame::Type::BGR888i || pixels.type == ImgFrame::Type::RGB888i) {
        bgr = plane(height, width, CV_8UC3, firstOffset, rowStride);
        if(pixels.type == ImgFrame::Type::RGB888i) cv::cvtColor(bgr, bgr, cv::COLOR_RGB2BGR);
        bgrToPlanes(bgr, pixels.fullRange, y, u, v);
    } else if(pixels.type == ImgFrame::Type::YUV420p) {
        const auto chromaStride = (rowStride + 1) / 2;
        const auto secondOffset = explicitOffsets ? static_cast<std::size_t>(pixels.fb.p2Offset) : rowStride * height;
        const auto thirdOffset = explicitOffsets ? static_cast<std::size_t>(pixels.fb.p3Offset) : secondOffset + chromaStride * chromaHeight;
        y = plane(height, width, CV_8UC1, firstOffset, rowStride);
        u = plane(chromaHeight, chromaWidth, CV_8UC1, secondOffset, chromaStride);
        v = plane(chromaHeight, chromaWidth, CV_8UC1, thirdOffset, chromaStride);
    } else if(pixels.type == ImgFrame::Type::NV12) {
        const auto secondOffset = explicitOffsets ? static_cast<std::size_t>(pixels.fb.p2Offset) : rowStride * height;
        y = plane(height, width, CV_8UC1, firstOffset, rowStride);
        const auto uv = plane(chromaHeight, chromaWidth, CV_8UC2, secondOffset, rowStride);
        std::vector<cv::Mat> planes;
        cv::split(uv, planes);
        u = planes[0];
        v = planes[1];
    } else {
        fail(std::string("ReplayVideo output type is not supported: ") + frameTypeName(pixels.type));
    }
    if(width != outW || height != outH) {
        y = resizeMat(y, outW, outH);
        u = resizeMat(u, outW / 2, outH / 2);
        v = resizeMat(v, outW / 2, outH / 2);
    }
    DecodedPixels out;
    out.fullRange = pixels.fullRange;
    out.width = outW;
    out.height = outH;
    out.type = outType;
    if(outType == ImgFrame::Type::GRAY8 || outType == ImgFrame::Type::YUV420p || outType == ImgFrame::Type::NV12) {
        packYuv(out, y, u, v, outType);
        return out;
    }
    if(outType != ImgFrame::Type::RGB888i && outType != ImgFrame::Type::BGR888i) {
        fail(std::string("ReplayVideo output type is not supported: ") + frameTypeName(outType));
    }
    out.data.resize(static_cast<std::size_t>(outW * outH * 3));
    for(int row = 0; row < outH; row++) {
        for(int col = 0; col < outW; col++) {
            const int uvRow = row / 2;
            const int uvCol = col / 2;
            int r = 0;
            int g = 0;
            int b = 0;
            yuvToRgb(y.at<std::uint8_t>(row, col), u.at<std::uint8_t>(uvRow, uvCol), v.at<std::uint8_t>(uvRow, uvCol), r, g, b);
            const auto offset = static_cast<std::size_t>((row * outW + col) * 3);
            if(outType == ImgFrame::Type::BGR888i) {
                out.data[offset] = static_cast<std::uint8_t>(b);
                out.data[offset + 1] = static_cast<std::uint8_t>(g);
                out.data[offset + 2] = static_cast<std::uint8_t>(r);
            } else {
                out.data[offset] = static_cast<std::uint8_t>(r);
                out.data[offset + 1] = static_cast<std::uint8_t>(g);
                out.data[offset + 2] = static_cast<std::uint8_t>(b);
            }
        }
    }
    return out;
}

std::shared_ptr<ImgFrame> pixelsToFrame(const DecodedPixels& pixels) {
    auto frame = std::make_shared<ImgFrame>();
    frame->setType(pixels.type);
    frame->setWidth(static_cast<unsigned int>(pixels.width));
    frame->setHeight(static_cast<unsigned int>(pixels.height));
    frame->fb = pixels.fb;
    frame->sourceFb = pixels.sourceFb;
    frame->cam = pixels.cam;
    frame->instanceNum = pixels.instanceNum;
    frame->category = pixels.category;
    frame->transformation = pixels.transformation;
    if(pixels.passthrough) {
        frame->setData(pixels.data);
        return frame;
    }
    const unsigned int width = static_cast<unsigned int>(pixels.width);
    const unsigned int height = static_cast<unsigned int>(pixels.height);
    const unsigned int ySize = width * height;
    frame->fb.type = pixels.type;
    frame->fb.width = width;
    frame->fb.height = height;
    frame->sourceFb.type = pixels.type;
    frame->sourceFb.width = width;
    frame->sourceFb.height = height;
    if(pixels.type == ImgFrame::Type::GRAY8) {
        frame->fb.stride = width;
        frame->fb.bytesPP = 1;
        frame->fb.p1Offset = 0;
        frame->fb.p2Offset = 0;
        frame->fb.p3Offset = 0;
    } else if(pixels.type == ImgFrame::Type::YUV420p) {
        frame->fb.stride = width;
        frame->fb.bytesPP = 1;
        frame->fb.p1Offset = 0;
        frame->fb.p2Offset = ySize;
        frame->fb.p3Offset = ySize + ySize / 4;
    } else if(pixels.type == ImgFrame::Type::NV12) {
        frame->fb.stride = width;
        frame->fb.bytesPP = 1;
        frame->fb.p1Offset = 0;
        frame->fb.p2Offset = ySize;
        frame->fb.p3Offset = ySize;
    } else {
        frame->fb.stride = width * 3;
        frame->fb.bytesPP = 3;
        frame->fb.p1Offset = 0;
        frame->fb.p2Offset = 0;
        frame->fb.p3Offset = 0;
    }
    frame->sourceFb = frame->fb;
    frame->setData(pixels.data);
    if(pixels.transformation.getSize().first == 0) frame->transformation = ImgTransformation(width, height);
    return frame;
}

}  // namespace replay
}  // namespace dai
