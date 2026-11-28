#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <magic_enum/magic_enum.hpp>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <set>
#include <stdexcept>

#include "ReplayInternal.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "utility/RecordReplayImpl.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/EncodedFrame.pb.h"
    #include "depthai/schemas/ImgFrame.pb.h"
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
    return ext == ".mp4" || ext == ".avi" || ext == ".mkv" || ext == ".mov" || ext == ".m4v";
}

std::uint64_t fileSizeOf(const std::filesystem::path& path) {
    std::error_code error;
    const auto size = std::filesystem::file_size(path, error);
    if(error) return 0;
    return static_cast<std::uint64_t>(size);
}

std::uint32_t pngBigEndian(const unsigned char* bytes) {
    return (static_cast<std::uint32_t>(bytes[0]) << 24) | (static_cast<std::uint32_t>(bytes[1]) << 16) | (static_cast<std::uint32_t>(bytes[2]) << 8)
           | static_cast<std::uint32_t>(bytes[3]);
}

std::pair<int, int> pngSize(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    unsigned char header[26] = {};
    in.read(reinterpret_cast<char*>(header), 26);
    static const unsigned char kSig[8] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
    if(!in || std::memcmp(header, kSig, 8) != 0 || std::memcmp(header + 12, "IHDR", 4) != 0) fail("ReplayVideo png header is not valid: " + path.string());
    const int bitDepth = header[24];
    const int colorType = header[25];
    if(bitDepth == 16) fail("ReplayVideo png bit depth 16 is not supported: " + path.string());
    if(colorType == 3) fail("ReplayVideo png palette is not supported: " + path.string());
    const auto width = pngBigEndian(header + 16);
    const auto height = pngBigEndian(header + 20);
    if(width == 0 || height == 0 || width > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
       || height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        fail("ReplayVideo png header is not valid: " + path.string());
    }
    return {static_cast<int>(width), static_cast<int>(height)};
}

void checkPngHeader(const std::filesystem::path& path) {
    pngSize(path);
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

class ProtoCursor {
    std::ifstream& in;

   public:
    explicit ProtoCursor(std::ifstream& input) : in(input) {}

    bool readVarint(std::uint64_t& budget, std::uint64_t& value) {
        value = 0;
        for(int shift = 0; shift < 64; shift += 7) {
            if(budget == 0) return false;
            char byte = 0;
            if(!in.read(&byte, 1)) return false;
            --budget;
            const auto bits = static_cast<std::uint64_t>(static_cast<unsigned char>(byte));
            value |= (bits & 0x7fu) << shift;
            if((bits & 0x80u) == 0) return true;
        }
        return false;
    }

    bool skip(std::uint64_t& budget, std::uint64_t count) {
        if(count > budget) return false;
        if(count == 0) return true;
        in.seekg(static_cast<std::streamoff>(count), std::ios::cur);
        if(!in) return false;
        budget -= count;
        return true;
    }

    bool readString(std::uint64_t& budget, std::string& out, std::uint64_t count) {
        if(count > budget || count > 512) return false;
        out.assign(static_cast<std::size_t>(count), '\0');
        if(count > 0 && !in.read(out.data(), static_cast<std::streamsize>(count))) return false;
        budget -= count;
        return static_cast<bool>(in);
    }

    bool skipField(std::uint64_t& budget, std::uint64_t wire) {
        if(wire == 0) {
            std::uint64_t ignored = 0;
            return readVarint(budget, ignored);
        }
        if(wire == 1) return skip(budget, 8);
        if(wire == 5) return skip(budget, 4);
        if(wire != 2) return false;
        std::uint64_t length = 0;
        return readVarint(budget, length) && skip(budget, length);
    }

    bool takeLength(std::uint64_t& budget, std::uint64_t& length) {
        if(!readVarint(budget, length) || length > budget) return false;
        budget -= length;
        return true;
    }
};

struct ProtoKey {
    std::uint64_t field = 0;
    std::uint64_t wire = 0;
};

bool readKey(ProtoCursor& cursor, std::uint64_t& budget, ProtoKey& key) {
    std::uint64_t raw = 0;
    if(!cursor.readVarint(budget, raw)) return false;
    key.field = raw >> 3;
    key.wire = raw & 7u;
    return key.field != 0;
}

struct DaiHeader {
    int width = 0;
    int height = 0;
    ImgFrame::Type type = ImgFrame::Type::NONE;
    bool mjpeg = false;
    bool passthrough = false;
    std::int64_t sequenceNum = 0;
    std::chrono::steady_clock::time_point timestamp{};
    std::chrono::steady_clock::time_point timestampDevice{};
    std::optional<std::chrono::system_clock::time_point> timestampSystem;
};

struct ClockParts {
    bool present = false;
    std::int64_t sec = 0;
    std::int64_t nsec = 0;
};

bool readClock(ProtoCursor& cursor, std::uint64_t budget, ClockParts& clock) {
    while(budget > 0) {
        ProtoKey key;
        if(!readKey(cursor, budget, key)) return false;
        if(key.wire == 0 && (key.field == 1 || key.field == 2)) {
            std::uint64_t value = 0;
            if(!cursor.readVarint(budget, value)) return false;
            if(key.field == 1)
                clock.sec = static_cast<std::int64_t>(value);
            else
                clock.nsec = static_cast<std::int64_t>(value);
            clock.present = true;
        } else if(!cursor.skipField(budget, key.wire)) {
            return false;
        }
    }
    return true;
}

std::chrono::steady_clock::time_point toSteady(const ClockParts& clock) {
    if(!clock.present) return {};
    const auto total = std::chrono::seconds(clock.sec) + std::chrono::nanoseconds(clock.nsec);
    return std::chrono::time_point<std::chrono::steady_clock>(std::chrono::duration_cast<std::chrono::steady_clock::duration>(total));
}

std::optional<std::chrono::system_clock::time_point> toSystem(const ClockParts& clock) {
    if(!clock.present) return std::nullopt;
    const auto total = std::chrono::seconds(clock.sec) + std::chrono::nanoseconds(clock.nsec);
    return std::chrono::time_point<std::chrono::system_clock>(std::chrono::duration_cast<std::chrono::system_clock::duration>(total));
}

bool fitUint32(std::uint64_t value, std::uint32_t& out) {
    if(value > std::numeric_limits<std::uint32_t>::max()) return false;
    out = static_cast<std::uint32_t>(value);
    return true;
}

bool readSpecs(ProtoCursor& cursor, std::uint64_t budget, int& type, std::uint32_t& width, std::uint32_t& height) {
    while(budget > 0) {
        ProtoKey key;
        if(!readKey(cursor, budget, key)) return false;
        if(key.wire == 0 && key.field >= 1 && key.field <= 3) {
            std::uint64_t value = 0;
            if(!cursor.readVarint(budget, value)) return false;
            if(key.field == 1) {
                if(value > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) return false;
                type = static_cast<int>(value);
            } else if(key.field == 2) {
                if(!fitUint32(value, width)) return false;
            } else if(!fitUint32(value, height)) {
                return false;
            }
        } else if(!cursor.skipField(budget, key.wire)) {
            return false;
        }
    }
    return true;
}

bool dimension(std::uint32_t value, int& out) {
    if(value == 0 || value > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) return false;
    out = static_cast<int>(value);
    return true;
}

DaiHeader readDaiHeader(const std::filesystem::path& path) {
#ifndef DEPTHAI_ENABLE_PROTOBUF
    (void)path;
    fail("ReplayVideo node requires protobuf support");
#else
    const auto bytes = fileSizeOf(path);
    std::ifstream file(path, std::ios::binary);
    if(!file || bytes == 0) fail("ReplayVideo source cannot be read: " + path.string());
    ProtoCursor cursor(file);
    std::uint64_t budget = bytes;
    std::string schema;
    bool sawMessage = false;
    DaiHeader header;
    const auto imageSchema = proto::img_frame::ImgFrame::descriptor()->full_name();
    const auto encodedSchema = proto::encoded_frame::EncodedFrame::descriptor()->full_name();
    while(budget > 0) {
        ProtoKey key;
        if(!readKey(cursor, budget, key)) fail("ReplayVideo dai envelope is not valid: " + path.string());
        if(key.wire == 2 && (key.field == 1 || key.field == 3)) {
            std::uint64_t length = 0;
            if(!cursor.takeLength(budget, length)) fail("ReplayVideo dai envelope is not valid: " + path.string());
            if(key.field == 1) {
                if(!cursor.readString(length, schema, length)) fail("ReplayVideo dai envelope is not valid: " + path.string());
                continue;
            }
            if(schema != imageSchema && schema != encodedSchema) fail("ReplayVideo dai message type is not supported: " + path.string());
            const bool image = schema == imageSchema;
            std::uint64_t inner = length;
            int rawType = 0;
            std::uint32_t width = 0;
            std::uint32_t height = 0;
            int profile = 0;
            ClockParts timestamp;
            ClockParts timestampDevice;
            ClockParts timestampSystem;
            bool sawSequence = false;
            while(inner > 0) {
                ProtoKey field;
                if(!readKey(cursor, inner, field)) fail("ReplayVideo dai envelope is not valid: " + path.string());
                const bool clockField = field.wire == 2
                                        && ((image && (field.field == 1 || field.field == 2 || field.field == 12))
                                            || (!image && (field.field == 13 || field.field == 14 || field.field == 17)));
                const bool specsField = image && field.wire == 2 && field.field == 4;
                const bool dataField = field.wire == 2 && ((image && field.field == 11) || (!image && field.field == 16));
                if(specsField || clockField) {
                    std::uint64_t nested = 0;
                    if(!cursor.takeLength(inner, nested)) fail("ReplayVideo dai envelope is not valid: " + path.string());
                    bool ok = false;
                    if(specsField) {
                        ok = readSpecs(cursor, nested, rawType, width, height);
                    } else if((image && field.field == 1) || (!image && field.field == 13)) {
                        ok = readClock(cursor, nested, timestamp);
                    } else if((image && field.field == 2) || (!image && field.field == 14)) {
                        ok = readClock(cursor, nested, timestampDevice);
                    } else {
                        ok = readClock(cursor, nested, timestampSystem);
                    }
                    if(!ok) fail("ReplayVideo dai envelope is not valid: " + path.string());
                } else if(field.wire == 0
                          && ((image && field.field == 3) || (!image && (field.field == 3 || field.field == 4 || field.field == 7 || field.field == 12)))) {
                    std::uint64_t value = 0;
                    if(!cursor.readVarint(inner, value)) fail("ReplayVideo dai envelope is not valid: " + path.string());
                    if(image && field.field == 3) {
                        header.sequenceNum = static_cast<std::int64_t>(value);
                        sawSequence = true;
                    } else if(!image && field.field == 3) {
                        if(!fitUint32(value, width)) fail("ReplayVideo dai envelope is not valid: " + path.string());
                    } else if(!image && field.field == 4) {
                        if(!fitUint32(value, height)) fail("ReplayVideo dai envelope is not valid: " + path.string());
                    } else if(!image && field.field == 7) {
                        if(value > 2) fail("ReplayVideo dai message type is not supported: " + path.string());
                        profile = static_cast<int>(value);
                    } else if(!image && field.field == 12) {
                        header.sequenceNum = static_cast<std::int64_t>(value);
                        sawSequence = true;
                    }
                } else if(dataField) {
                    std::uint64_t payload = 0;
                    if(!cursor.takeLength(inner, payload) || !cursor.skip(payload, payload)) fail("ReplayVideo dai envelope is not valid: " + path.string());
                } else if(!cursor.skipField(inner, field.wire)) {
                    fail("ReplayVideo dai envelope is not valid: " + path.string());
                }
            }
            if(!dimension(width, header.width) || !dimension(height, header.height)) fail("ReplayVideo dai envelope is not valid: " + path.string());
            if(image) {
                if(!magic_enum::enum_contains(static_cast<ImgFrame::Type>(rawType))) fail("ReplayVideo dai envelope is not valid: " + path.string());
                header.type = static_cast<ImgFrame::Type>(rawType);
                if(header.type == ImgFrame::Type::BITSTREAM) fail("ReplayVideo BITSTREAM is not supported: " + path.string());
                header.passthrough = !isReplayOutType(header.type);
            } else {
                if(profile == static_cast<int>(EncodedFrame::Profile::AVC) || profile == static_cast<int>(EncodedFrame::Profile::HEVC)) {
                    fail("ReplayVideo H264 or H265 EncodedFrame is not supported in .dai: " + path.string());
                }
                if(profile != static_cast<int>(EncodedFrame::Profile::JPEG)) fail("ReplayVideo dai message type is not supported: " + path.string());
                header.mjpeg = true;
                header.type = ImgFrame::Type::NV12;
                header.passthrough = false;
            }
            if(!sawSequence) header.sequenceNum = 0;
            header.timestamp = toSteady(timestamp);
            header.timestampDevice = toSteady(timestampDevice);
            header.timestampSystem = toSystem(timestampSystem);
            sawMessage = true;
            break;
        }
        if(!cursor.skipField(budget, key.wire)) fail("ReplayVideo dai envelope is not valid: " + path.string());
    }
    if(!sawMessage) fail("ReplayVideo dai envelope is not valid: " + path.string());
    if(!file) fail("ReplayVideo dai envelope is not valid: " + path.string());
    return header;
#endif
}

void setPackedLayout(ImgFrame& frame, ImgFrame::Type type, unsigned width, unsigned height) {
    frame.setType(type);
    frame.setSize(width, height);
    const unsigned ySize = width * height;
    frame.fb.p1Offset = 0;
    if(type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RAW8) {
        frame.fb.stride = width;
        frame.fb.p2Offset = 0;
        frame.fb.p3Offset = 0;
    } else if(type == ImgFrame::Type::RGB888p || type == ImgFrame::Type::BGR888p) {
        frame.fb.stride = width;
        frame.fb.p2Offset = ySize;
        frame.fb.p3Offset = ySize * 2;
    } else if(type == ImgFrame::Type::YUV420p) {
        frame.fb.stride = width;
        frame.fb.p2Offset = ySize;
        frame.fb.p3Offset = ySize + ySize / 4;
    } else if(type == ImgFrame::Type::NV12) {
        frame.fb.stride = width;
        frame.fb.p2Offset = ySize;
        frame.fb.p3Offset = ySize;
    } else {
        frame.fb.stride = width * 3;
        frame.fb.p2Offset = 0;
        frame.fb.p3Offset = 0;
    }
}

std::shared_ptr<ImgFrame> frameFromPacked(ImgFrame::Type type, int width, int height, std::vector<std::uint8_t> data) {
    auto frame = std::make_shared<ImgFrame>();
    setPackedLayout(*frame, type, static_cast<unsigned>(width), static_cast<unsigned>(height));
    frame->setData(std::move(data));
    return frame;
}

std::shared_ptr<ImgFrame> frameFromMat(const cv::Mat& image, ImgFrame::Type type) {
    auto frame = std::make_shared<ImgFrame>();
    frame->setCvFrame(image, type);
    return frame;
}

void copyPlaybackInfo(ImgFrame& dst, const ImgFrame& src) {
    dst.sourceFb = src.sourceFb;
    dst.cam = src.cam;
    dst.instanceNum = src.instanceNum;
    dst.category = src.category;
    dst.transformation = src.transformation;
    dst.setSequenceNum(src.getSequenceNum());
    dst.setTimestamp(src.getTimestamp());
    dst.setTimestampDevice(src.getTimestampDevice());
    dst.setTimestampSystem(src.getTimestampSystem());
}

std::shared_ptr<ImgFrame> readDai(const std::filesystem::path& path) {
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
    if(envelope.schema_name() == imgProbe.serializeSchema().schemaName) {
        auto frame = std::make_shared<ImgFrame>();
        frame->load(path);
        if(frame->getType() == ImgFrame::Type::BITSTREAM) fail("ReplayVideo BITSTREAM is not supported: " + path.string());
        return frame;
    }
    if(envelope.schema_name() == encProbe.serializeSchema().schemaName) {
        EncodedFrame encoded;
        encoded.load(path);
        if(encoded.profile == EncodedFrame::Profile::AVC || encoded.profile == EncodedFrame::Profile::HEVC) {
            fail("ReplayVideo H264 or H265 EncodedFrame is not supported in .dai: " + path.string());
        }
        const auto span = encoded.getData();
        std::vector<std::uint8_t> bytes(span.begin(), span.end());
        cv::Mat buffer(1, static_cast<int>(bytes.size()), CV_8UC1, bytes.data());
        cv::Mat image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED | cv::IMREAD_IGNORE_ORIENTATION);
        if(image.empty()) fail("ReplayVideo jpeg data is not valid: " + path.string());
        if(image.channels() == 4) cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
        const auto type = image.channels() == 1 ? ImgFrame::Type::GRAY8 : ImgFrame::Type::BGR888i;
        auto frame = frameFromMat(image, type);
        frame->setSequenceNum(encoded.getSequenceNum());
        frame->setTimestamp(encoded.getTimestamp());
        frame->setTimestampDevice(encoded.getTimestampDevice());
        frame->setTimestampSystem(encoded.getTimestampSystem());
        return frame;
    }
    fail("ReplayVideo dai message type is not supported: " + path.string());
#endif
}

ListedSource listFrameDirectory(const std::filesystem::path& directory) {
    struct Item {
        std::filesystem::path path;
        std::int64_t sequence = 0;
        std::string prefix;
        std::string ext;
    };
    std::vector<Item> frames;
    std::uint64_t directoryBytes = 0;
    for(const auto& entry : std::filesystem::directory_iterator(directory)) {
        if(!entry.is_regular_file()) continue;
        const auto filename = entry.path().filename().string();
        if(filename.empty() || filename.front() == '.') continue;
        directoryBytes += fileSizeOf(entry.path());
        auto ext = lowerExt(entry.path());
        if(ext == ".jpeg") ext = ".jpg";
        if(ext != ".dai" && ext != ".jpg" && ext != ".png") continue;
        const auto stem = entry.path().stem().string();
        const auto split = stem.rfind('_');
        if(split == std::string::npos || split == 0 || split + 1 >= stem.size()) continue;
        const auto seqText = stem.substr(split + 1);
        if(!std::all_of(seqText.begin(), seqText.end(), [](unsigned char character) { return std::isdigit(character) != 0; })) continue;
        Item item;
        item.path = entry.path();
        item.sequence = std::stoll(seqText);
        item.prefix = stem.substr(0, split);
        item.ext = ext;
        frames.push_back(std::move(item));
    }
    if(frames.empty()) fail("ReplayVideo source is empty: " + directory.string());
    std::set<std::string> prefixes;
    std::set<std::string> extensions;
    for(const auto& frame : frames) {
        prefixes.insert(frame.prefix);
        extensions.insert(frame.ext);
    }
    if(prefixes.size() != 1) {
        std::string listed;
        for(const auto& prefix : prefixes) {
            if(!listed.empty()) listed += ", ";
            listed += prefix;
        }
        fail("ReplayVideo directory has more than one stream: " + listed);
    }
    if(extensions.size() != 1) fail("ReplayVideo directory has more than one extension: " + directory.string());
    std::map<std::int64_t, std::string> seen;
    for(const auto& frame : frames) {
        if(!seen.emplace(frame.sequence, frame.path.filename().string()).second) {
            fail("ReplayVideo duplicate frame number " + std::to_string(frame.sequence) + " in " + frame.path.filename().string());
        }
    }
    std::sort(frames.begin(), frames.end(), [](const auto& a, const auto& b) { return a.sequence < b.sequence; });
    ListedSource source;
    source.kind = ListedSource::Kind::Frames;
    source.path = directory;
    source.stream = *prefixes.begin();
    source.dai = *extensions.begin() == ".dai";
    source.jpegOrPng = !source.dai;
    const auto metadata = directory / (source.stream + ".mcap");
    if(std::filesystem::exists(metadata)) source.autoMetadata = metadata;
    if(source.dai) {
        const auto first = readDaiHeader(frames.front().path);
        source.width = first.width;
        source.height = first.height;
        source.sourceType = first.type;
        source.passthrough = first.passthrough && !first.mjpeg;
        source.mjpegDai = first.mjpeg;
        if(first.mjpeg) {
            source.sourceType = ImgFrame::Type::NV12;
            source.passthrough = false;
        }
        for(std::size_t i = 1; i < frames.size(); i++) {
            const auto other = readDaiHeader(frames[i].path);
            if(other.width != source.width || other.height != source.height) {
                fail("ReplayVideo frame size does not match: " + frames[i].path.string() + " is " + std::to_string(other.width) + "x"
                     + std::to_string(other.height));
            }
            if(other.type != source.sourceType && !first.mjpeg) {
                fail("ReplayVideo frame type does not match: " + frames[i].path.string());
            }
        }
    } else {
        const bool png = *extensions.begin() == ".png";
        const auto size = png ? pngSize(frames.front().path) : jpegSize(frames.front().path);
        source.width = size.first;
        source.height = size.second;
        source.sourceType = ImgFrame::Type::NV12;
        for(std::size_t i = 1; i < frames.size(); i++) {
            const auto other = png ? pngSize(frames[i].path) : jpegSize(frames[i].path);
            if(other.first != source.width || other.second != source.height) {
                fail("ReplayVideo frame size does not match: " + frames[i].path.string() + " is " + std::to_string(other.first) + "x"
                     + std::to_string(other.second));
            }
        }
    }
    source.frameCount = static_cast<int>(frames.size());
    source.bytes = directoryBytes;
    for(const auto& frame : frames) source.frames.push_back(ListedFrame{frame.path, frame.sequence});
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
    video.codec->thread_count = 0;
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
        if(bitDepth != 8) fail("ReplayVideo bit depth " + std::to_string(bitDepth) + " is not supported: " + name);
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

std::shared_ptr<ImgFrame> readLibavFrame(LibavVideo& video) {
    bool flushed = false;
    while(true) {
        const int received = avcodec_receive_frame(video.codec, video.frame);
        if(received == 0) {
            if(video.frame->format != AV_PIX_FMT_YUV420P && video.frame->format != AV_PIX_FMT_YUVJ420P) {
                fail("ReplayVideo profile " + std::string(av_get_pix_fmt_name(static_cast<AVPixelFormat>(video.frame->format))) + " is not supported");
            }
            const int width = video.frame->width;
            const int height = video.frame->height;
            const int ySize = width * height;
            const int cSize = ySize / 4;
            std::vector<std::uint8_t> data(static_cast<std::size_t>(ySize + 2 * cSize));
            for(int row = 0; row < height; row++) {
                std::memcpy(data.data() + static_cast<std::size_t>(row * width), video.frame->data[0] + row * video.frame->linesize[0], width);
            }
            for(int row = 0; row < height / 2; row++) {
                std::memcpy(
                    data.data() + static_cast<std::size_t>(ySize + row * (width / 2)), video.frame->data[1] + row * video.frame->linesize[1], width / 2);
                std::memcpy(data.data() + static_cast<std::size_t>(ySize + cSize + row * (width / 2)),
                            video.frame->data[2] + row * video.frame->linesize[2],
                            width / 2);
            }
            video.next++;
            return frameFromPacked(ImgFrame::Type::YUV420p, width, height, std::move(data));
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

std::string lowerCodecTag(const std::string& tag) {
    std::string codec = tag;
    for(char& character : codec) {
        const auto value = static_cast<unsigned char>(character);
        if(value < 32 || value > 126)
            character = ' ';
        else
            character = static_cast<char>(std::tolower(value));
    }
    return codec;
}

bool bytesAt(const std::vector<char>& data, std::size_t index, const char* key, std::size_t length) {
    if(index + length > data.size()) return false;
    return std::memcmp(data.data() + index, key, length) == 0;
}

int openCvBitDepthMinus8(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if(!in) return 0;
    std::vector<char> data(4 << 20);
    in.read(data.data(), static_cast<std::streamsize>(data.size()));
    data.resize(static_cast<std::size_t>(std::max<std::streamsize>(0, in.gcount())));
    for(std::size_t index = 0; index + 22 < data.size(); index++) {
        if(!bytesAt(data, index, "hvcC", 4)) continue;
        if(static_cast<unsigned char>(data[index + 4]) != 1) continue;
        const int luma = static_cast<unsigned char>(data[index + 21]) & 0x7;
        if(luma != 0) return luma;
    }
    for(std::size_t index = 0; index + 6 < data.size(); index++) {
        if(!bytesAt(data, index, "avcC", 4)) continue;
        if(static_cast<unsigned char>(data[index + 4]) != 1) continue;
        const auto profile = static_cast<unsigned char>(data[index + 5]);
        if(profile == 110 || profile == 122) return 2;
    }
    return 0;
}

void rejectOpenCvCodec(const std::filesystem::path& path, const std::string& codec) {
    const bool vp9 = codec.find("vp9") != std::string::npos || codec.find("vp09") != std::string::npos || codec.find("vp90") != std::string::npos;
    const bool vp8 = codec.find("vp8") != std::string::npos || codec.find("vp08") != std::string::npos || codec.find("vp80") != std::string::npos;
    if(vp9) fail("ReplayVideo codec vp9 is not supported");
    if(vp8) fail("ReplayVideo codec vp8 is not supported");
    const int minus8 = openCvBitDepthMinus8(path);
    if(minus8 > 0) {
        const auto name = codec.find_first_not_of(' ') == std::string::npos ? std::string("hevc") : codec;
        fail("ReplayVideo bit depth " + std::to_string(8 + minus8) + " is not supported: " + name);
    }
}

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
    rejectOpenCvCodec(path, lowerCodecTag(tag));
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

int rowBytesOf(ImgFrame::Type type, int width) {
    if(type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i) return width * 3;
    return width;
}

std::size_t packedBytes(ImgFrame::Type type, int width, int height) {
    const auto area = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if(type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RAW8) return area;
    if(type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12) return area + area / 2;
    if(type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i || type == ImgFrame::Type::RGB888p || type == ImgFrame::Type::BGR888p)
        return area * 3;
    return 0;
}

void copyRows(std::vector<std::uint8_t>& out, const std::uint8_t* data, std::size_t size, int rows, int rowBytes, std::size_t offset, std::size_t stride) {
    if(rows <= 0 || rowBytes <= 0 || stride < static_cast<std::size_t>(rowBytes) || offset > size || size - offset < static_cast<std::size_t>(rowBytes)
       || static_cast<std::size_t>(rows - 1) > (size - offset - static_cast<std::size_t>(rowBytes)) / stride) {
        fail("ReplayVideo frame payload is too small or has an invalid plane layout");
    }
    const auto start = out.size();
    out.resize(start + static_cast<std::size_t>(rows) * static_cast<std::size_t>(rowBytes));
    for(int row = 0; row < rows; row++) {
        std::memcpy(out.data() + start + static_cast<std::size_t>(row) * static_cast<std::size_t>(rowBytes),
                    data + offset + static_cast<std::size_t>(row) * stride,
                    static_cast<std::size_t>(rowBytes));
    }
}

bool canonicalLayout(const ImgFrame& frame) {
    const auto type = frame.getType();
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const auto need = packedBytes(type, width, height);
    if(width <= 0 || height <= 0 || need == 0 || frame.getData().size() < need) return false;
    const int rowBytes = rowBytesOf(type, width);
    if(frame.fb.stride != 0 && static_cast<int>(frame.fb.stride) != rowBytes) return false;
    if(frame.fb.p1Offset != 0) return false;
    const auto area = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if(type == ImgFrame::Type::YUV420p) {
        if(frame.fb.p2Offset != 0 && frame.fb.p2Offset != area) return false;
        if(frame.fb.p3Offset != 0 && frame.fb.p3Offset != area + area / 4) return false;
    } else if(type == ImgFrame::Type::NV12) {
        if(frame.fb.p2Offset != 0 && frame.fb.p2Offset != area) return false;
        if(frame.fb.p3Offset != 0 && frame.fb.p3Offset != area) return false;
    } else if(type == ImgFrame::Type::RGB888p || type == ImgFrame::Type::BGR888p) {
        if(frame.fb.p2Offset != 0 && frame.fb.p2Offset != area) return false;
        if(frame.fb.p3Offset != 0 && frame.fb.p3Offset != area * 2) return false;
    } else if(frame.fb.p2Offset != 0 || frame.fb.p3Offset != 0) {
        return false;
    }
    return true;
}

std::shared_ptr<ImgFrame> repack(const ImgFrame& frame) {
    const auto type = frame.getType();
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const auto bytes = frame.getData();
    const auto* data = bytes.data();
    const auto size = bytes.size();
    const bool explicitOffsets = frame.fb.p1Offset != 0 || frame.fb.p2Offset != 0 || frame.fb.p3Offset != 0;
    const auto rowStride = frame.fb.stride != 0 ? static_cast<std::size_t>(frame.fb.stride) : static_cast<std::size_t>(rowBytesOf(type, width));
    const auto first = static_cast<std::size_t>(frame.fb.p1Offset);
    std::vector<std::uint8_t> packed;
    const auto need = packedBytes(type, width, height);
    if(need != 0) packed.reserve(need);
    if(type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RAW8) {
        copyRows(packed, data, size, height, width, first, rowStride);
    } else if(type == ImgFrame::Type::BGR888i || type == ImgFrame::Type::RGB888i) {
        copyRows(packed, data, size, height, width * 3, first, rowStride);
    } else if(type == ImgFrame::Type::BGR888p || type == ImgFrame::Type::RGB888p) {
        const auto area = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        const auto stride = frame.fb.stride != 0 ? static_cast<std::size_t>(frame.fb.stride) : static_cast<std::size_t>(width);
        const auto plane0 = explicitOffsets ? static_cast<std::size_t>(frame.fb.p1Offset) : 0;
        const auto plane1 = explicitOffsets ? static_cast<std::size_t>(frame.fb.p2Offset) : area;
        const auto plane2 = explicitOffsets ? static_cast<std::size_t>(frame.fb.p3Offset) : area * 2;
        copyRows(packed, data, size, height, width, plane0, stride);
        copyRows(packed, data, size, height, width, plane1, stride);
        copyRows(packed, data, size, height, width, plane2, stride);
    } else if(type == ImgFrame::Type::YUV420p) {
        const int chromaWidth = width / 2 + width % 2;
        const int chromaHeight = height / 2 + height % 2;
        const auto chromaStride = (rowStride + 1) / 2;
        const auto second = explicitOffsets ? static_cast<std::size_t>(frame.fb.p2Offset) : rowStride * static_cast<std::size_t>(height);
        const auto third = explicitOffsets ? static_cast<std::size_t>(frame.fb.p3Offset) : second + chromaStride * static_cast<std::size_t>(chromaHeight);
        copyRows(packed, data, size, height, width, first, rowStride);
        copyRows(packed, data, size, chromaHeight, chromaWidth, second, chromaStride);
        copyRows(packed, data, size, chromaHeight, chromaWidth, third, chromaStride);
    } else if(type == ImgFrame::Type::NV12) {
        const int chromaHeight = height / 2 + height % 2;
        const int chromaWidth = width / 2 + width % 2;
        const auto second = explicitOffsets ? static_cast<std::size_t>(frame.fb.p2Offset) : rowStride * static_cast<std::size_t>(height);
        copyRows(packed, data, size, height, width, first, rowStride);
        copyRows(packed, data, size, chromaHeight, chromaWidth * 2, second, rowStride);
    } else {
        fail(std::string("ReplayVideo output type is not supported: ") + frameTypeName(type));
    }
    auto out = frameFromPacked(type, width, height, std::move(packed));
    copyPlaybackInfo(*out, frame);
    return out;
}

bool isLuma(ImgFrame::Type type) {
    return type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RAW8 || type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12;
}

std::shared_ptr<ImgFrame> finishPacked(ImgFrame::Type type, int width, int height, std::vector<std::uint8_t> data, const ImgFrame& source) {
    auto out = frameFromPacked(type, width, height, std::move(data));
    copyPlaybackInfo(*out, source);
    return out;
}

void interleaveChroma(std::uint8_t* dst, const std::uint8_t* u, const std::uint8_t* v, int count) {
    for(int index = 0; index < count; ++index) {
        dst[static_cast<std::size_t>(index) * 2] = u[index];
        dst[static_cast<std::size_t>(index) * 2 + 1] = v[index];
    }
}

void splitChroma(std::uint8_t* u, std::uint8_t* v, const std::uint8_t* src, int count) {
    for(int index = 0; index < count; ++index) {
        u[index] = src[static_cast<std::size_t>(index) * 2];
        v[index] = src[static_cast<std::size_t>(index) * 2 + 1];
    }
}

void placePlane(const cv::Mat& src, cv::Mat& dst) {
    if(src.cols == dst.cols && src.rows == dst.rows) {
        if(src.data == dst.data) return;
        src.copyTo(dst);
        return;
    }
    const bool down = static_cast<std::int64_t>(dst.cols) * dst.rows < static_cast<std::int64_t>(src.cols) * src.rows;
    cv::resize(src, dst, dst.size(), 0, 0, down ? cv::INTER_AREA : cv::INTER_LINEAR);
}

// YUV420p, NV12, gray, and raw share luma. Copy that plane and rearrange chroma. A BGR round-trip is lossy and slower.
std::shared_ptr<ImgFrame> convertLuma(const ImgFrame& frame, ImgFrame::Type outType, int outW, int outH) {
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const auto bytes = frame.getData();
    const auto* src = bytes.data();
    const int ySize = width * height;
    const int outY = outW * outH;
    const auto srcType = frame.getType();
    const bool srcChroma = srcType == ImgFrame::Type::YUV420p || srcType == ImgFrame::Type::NV12;
    const bool outChroma = outType == ImgFrame::Type::YUV420p || outType == ImgFrame::Type::NV12;
    std::vector<std::uint8_t> packed(packedBytes(outType, outW, outH));

    if(srcChroma && srcType == ImgFrame::Type::YUV420p && outType == ImgFrame::Type::NV12 && width == outW && height == outH) {
        std::memcpy(packed.data(), src, static_cast<std::size_t>(ySize));
        const int chroma = ySize / 4;
        interleaveChroma(packed.data() + ySize, src + ySize, src + ySize + chroma, chroma);
        return finishPacked(outType, outW, outH, std::move(packed), frame);
    }
    if(srcChroma && srcType == ImgFrame::Type::NV12 && outType == ImgFrame::Type::YUV420p && width == outW && height == outH) {
        std::memcpy(packed.data(), src, static_cast<std::size_t>(ySize));
        const int chroma = ySize / 4;
        splitChroma(packed.data() + ySize, packed.data() + ySize + chroma, src + ySize, chroma);
        return finishPacked(outType, outW, outH, std::move(packed), frame);
    }

    cv::Mat ySrc(height, width, CV_8UC1, const_cast<std::uint8_t*>(src));
    cv::Mat yDst(outH, outW, CV_8UC1, packed.data());
    placePlane(ySrc, yDst);
    if(!outChroma) return finishPacked(outType, outW, outH, std::move(packed), frame);

    const int chromaRows = outH / 2;
    const int chromaCols = outW / 2;
    const int chroma = chromaRows * chromaCols;
    if(!srcChroma) {
        std::memset(packed.data() + outY, 128, packed.size() - static_cast<std::size_t>(outY));
        return finishPacked(outType, outW, outH, std::move(packed), frame);
    }
    if((width % 2) != 0 || (height % 2) != 0) fail("ReplayVideo frame dimensions are invalid");

    if(srcType == ImgFrame::Type::YUV420p && outType == ImgFrame::Type::YUV420p) {
        cv::Mat uSrc(height / 2, width / 2, CV_8UC1, const_cast<std::uint8_t*>(src + ySize));
        cv::Mat vSrc(height / 2, width / 2, CV_8UC1, const_cast<std::uint8_t*>(src + ySize + ySize / 4));
        cv::Mat uDst(chromaRows, chromaCols, CV_8UC1, packed.data() + outY);
        cv::Mat vDst(chromaRows, chromaCols, CV_8UC1, packed.data() + outY + chroma);
        placePlane(uSrc, uDst);
        placePlane(vSrc, vDst);
    } else if(srcType == ImgFrame::Type::NV12 && outType == ImgFrame::Type::NV12) {
        cv::Mat uvSrc(height / 2, width / 2, CV_8UC2, const_cast<std::uint8_t*>(src + ySize));
        cv::Mat uvDst(chromaRows, chromaCols, CV_8UC2, packed.data() + outY);
        placePlane(uvSrc, uvDst);
    } else if(srcType == ImgFrame::Type::YUV420p) {
        cv::Mat uSrc(height / 2, width / 2, CV_8UC1, const_cast<std::uint8_t*>(src + ySize));
        cv::Mat vSrc(height / 2, width / 2, CV_8UC1, const_cast<std::uint8_t*>(src + ySize + ySize / 4));
        cv::Mat u(chromaRows, chromaCols, CV_8UC1);
        cv::Mat v(chromaRows, chromaCols, CV_8UC1);
        placePlane(uSrc, u);
        placePlane(vSrc, v);
        interleaveChroma(packed.data() + outY, u.data, v.data, chroma);
    } else {
        cv::Mat uvSrc(height / 2, width / 2, CV_8UC2, const_cast<std::uint8_t*>(src + ySize));
        cv::Mat uv(chromaRows, chromaCols, CV_8UC2);
        placePlane(uvSrc, uv);
        splitChroma(packed.data() + outY, packed.data() + outY + chroma, uv.data, chroma);
    }
    return finishPacked(outType, outW, outH, std::move(packed), frame);
}

cv::Mat bgrFromColor(const ImgFrame& frame) {
    const auto bytes = frame.getData();
    auto* data = const_cast<std::uint8_t*>(bytes.data());
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const auto area = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if(frame.getType() == ImgFrame::Type::BGR888i) return cv::Mat(height, width, CV_8UC3, data);
    if(frame.getType() == ImgFrame::Type::RGB888i) {
        cv::Mat bgr;
        cv::cvtColor(cv::Mat(height, width, CV_8UC3, data), bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }
    cv::Mat first(height, width, CV_8UC1, data);
    cv::Mat second(height, width, CV_8UC1, data + area);
    cv::Mat third(height, width, CV_8UC1, data + area * 2);
    cv::Mat bgr;
    if(frame.getType() == ImgFrame::Type::BGR888p)
        cv::merge(std::vector<cv::Mat>{first, second, third}, bgr);
    else
        cv::merge(std::vector<cv::Mat>{third, second, first}, bgr);
    return bgr;
}

std::shared_ptr<ImgFrame> packFromBgr(const cv::Mat& bgr, ImgFrame::Type outType, const ImgFrame& source) {
    const int width = bgr.cols;
    const int height = bgr.rows;
    if(outType == ImgFrame::Type::BGR888i) {
        std::vector<std::uint8_t> packed(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);
        cv::Mat dst(height, width, CV_8UC3, packed.data());
        bgr.copyTo(dst);
        return finishPacked(outType, width, height, std::move(packed), source);
    }
    if(outType == ImgFrame::Type::RGB888i) {
        std::vector<std::uint8_t> packed(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3);
        cv::Mat dst(height, width, CV_8UC3, packed.data());
        cv::cvtColor(bgr, dst, cv::COLOR_BGR2RGB);
        return finishPacked(outType, width, height, std::move(packed), source);
    }
    if(outType == ImgFrame::Type::BGR888p || outType == ImgFrame::Type::RGB888p) {
        const auto area = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        std::vector<std::uint8_t> packed(area * 3);
        cv::Mat channel0(height, width, CV_8UC1, packed.data());
        cv::Mat channel1(height, width, CV_8UC1, packed.data() + area);
        cv::Mat channel2(height, width, CV_8UC1, packed.data() + area * 2);
        if(outType == ImgFrame::Type::BGR888p)
            cv::split(bgr, std::vector<cv::Mat>{channel0, channel1, channel2});
        else
            cv::split(bgr, std::vector<cv::Mat>{channel2, channel1, channel0});
        return finishPacked(outType, width, height, std::move(packed), source);
    }
    if(outType == ImgFrame::Type::GRAY8 || outType == ImgFrame::Type::RAW8) {
        std::vector<std::uint8_t> packed(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
        cv::Mat dst(height, width, CV_8UC1, packed.data());
        if(bgr.channels() == 1)
            bgr.copyTo(dst);
        else
            cv::cvtColor(bgr, dst, cv::COLOR_BGR2GRAY);
        return finishPacked(outType, width, height, std::move(packed), source);
    }
    const int ySize = width * height;
    const int chroma = ySize / 4;
    if(outType == ImgFrame::Type::YUV420p) {
        std::vector<std::uint8_t> packed(static_cast<std::size_t>(ySize + 2 * chroma));
        cv::Mat dst(height * 3 / 2, width, CV_8UC1, packed.data());
        cv::cvtColor(bgr, dst, cv::COLOR_BGR2YUV_I420);
        return finishPacked(outType, width, height, std::move(packed), source);
    }
    std::vector<std::uint8_t> i420(static_cast<std::size_t>(ySize + 2 * chroma));
    cv::Mat dst(height * 3 / 2, width, CV_8UC1, i420.data());
    cv::cvtColor(bgr, dst, cv::COLOR_BGR2YUV_I420);
    std::vector<std::uint8_t> packed(i420.size());
    std::memcpy(packed.data(), i420.data(), static_cast<std::size_t>(ySize));
    interleaveChroma(packed.data() + ySize, i420.data() + ySize, i420.data() + ySize + chroma, chroma);
    return finishPacked(outType, width, height, std::move(packed), source);
}

std::shared_ptr<ImgFrame> convertYuvToColor(const ImgFrame& frame, ImgFrame::Type outType, int outW, int outH) {
    const int width = static_cast<int>(frame.getWidth());
    const int height = static_cast<int>(frame.getHeight());
    const auto bytes = frame.getData();
    cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<std::uint8_t*>(bytes.data()));
    const bool nv12 = frame.getType() == ImgFrame::Type::NV12;
    if((outType == ImgFrame::Type::BGR888i || outType == ImgFrame::Type::RGB888i) && width == outW && height == outH) {
        std::vector<std::uint8_t> packed(static_cast<std::size_t>(outW) * static_cast<std::size_t>(outH) * 3);
        cv::Mat dst(outH, outW, CV_8UC3, packed.data());
        const bool rgb = outType == ImgFrame::Type::RGB888i;
        const auto code = nv12 ? (rgb ? cv::COLOR_YUV2RGB_NV12 : cv::COLOR_YUV2BGR_NV12) : (rgb ? cv::COLOR_YUV2RGB_IYUV : cv::COLOR_YUV2BGR_IYUV);
        cv::cvtColor(yuv, dst, code);
        return finishPacked(outType, outW, outH, std::move(packed), frame);
    }
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, nv12 ? cv::COLOR_YUV2BGR_NV12 : cv::COLOR_YUV2BGR_IYUV);
    if(bgr.cols != outW || bgr.rows != outH) bgr = resizeMat(bgr, outW, outH);
    return packFromBgr(bgr, outType, frame);
}

std::shared_ptr<ImgFrame> convertColor(const ImgFrame& frame, ImgFrame::Type outType, int outW, int outH) {
    if(frame.getType() == ImgFrame::Type::GRAY8 || frame.getType() == ImgFrame::Type::RAW8) {
        const auto bytes = frame.getData();
        cv::Mat gray(static_cast<int>(frame.getHeight()), static_cast<int>(frame.getWidth()), CV_8UC1, const_cast<std::uint8_t*>(bytes.data()));
        if(gray.cols != outW || gray.rows != outH) gray = resizeMat(gray, outW, outH);
        if(outType == ImgFrame::Type::BGR888i || outType == ImgFrame::Type::RGB888i) {
            std::vector<std::uint8_t> packed(static_cast<std::size_t>(outW) * static_cast<std::size_t>(outH) * 3);
            cv::Mat dst(outH, outW, CV_8UC3, packed.data());
            cv::cvtColor(gray, dst, cv::COLOR_GRAY2BGR);
            return finishPacked(outType, outW, outH, std::move(packed), frame);
        }
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return packFromBgr(bgr, outType, frame);
    }
    if(frame.getType() == ImgFrame::Type::YUV420p || frame.getType() == ImgFrame::Type::NV12) return convertYuvToColor(frame, outType, outW, outH);
    cv::Mat bgr = bgrFromColor(frame);
    if(bgr.cols != outW || bgr.rows != outH) bgr = resizeMat(bgr, outW, outH);
    return packFromBgr(bgr, outType, frame);
}

}  // namespace

std::shared_ptr<ImgFrame> probeDaiHeader(const std::filesystem::path& path) {
    const DaiHeader header = readDaiHeader(path);
    auto frame = std::make_shared<ImgFrame>();
    frame->setType(header.type);
    frame->setSize(static_cast<unsigned>(header.width), static_cast<unsigned>(header.height));
    frame->setSequenceNum(header.sequenceNum);
    frame->setTimestamp(header.timestamp);
    frame->setTimestampDevice(header.timestampDevice);
    frame->setTimestampSystem(header.timestampSystem);
    return frame;
}

bool isReplayOutType(ImgFrame::Type type) {
    return type == ImgFrame::Type::NV12 || type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RGB888i
           || type == ImgFrame::Type::BGR888i || type == ImgFrame::Type::RGB888p || type == ImgFrame::Type::BGR888p || type == ImgFrame::Type::RAW8;
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
    bool rawYuv = false;
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
        std::string codec = state->source.codec;
        std::transform(codec.begin(), codec.end(), codec.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
        state->rawYuv = codec.find("ffv1") != std::string::npos;
        if(state->rawYuv) state->capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
        state->capture.open(state->source.path.string(), cv::CAP_FFMPEG);
        if(state->rawYuv) state->capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
        if(!state->capture.isOpened()) fail("ReplayVideo missing decoder support for " + state->source.path.string());
#endif
    }
}

int FrameReader::count() const {
    return state ? state->source.frameCount : 0;
}

std::shared_ptr<ImgFrame> FrameReader::read(int index) {
    if(!state) fail("ReplayVideo reader is not open");
    if(index < state->next) open(state->source);
    if(state->source.kind == ListedSource::Kind::Frames) {
        if(index < 0 || index >= static_cast<int>(state->source.frames.size())) fail("ReplayVideo frame is missing");
        const auto& path = state->source.frames[static_cast<std::size_t>(index)].path;
        const auto ext = lowerExt(path);
        std::shared_ptr<ImgFrame> frame;
        if(ext == ".dai") {
            frame = readDai(path);
        } else {
            if(ext == ".png") checkPngHeader(path);
            cv::Mat image = cv::imread(path.string(), cv::IMREAD_UNCHANGED | cv::IMREAD_IGNORE_ORIENTATION);
            if(image.empty()) fail("ReplayVideo image cannot be read: " + path.string());
            if(image.channels() == 4) cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
            const auto type = image.channels() == 1 ? ImgFrame::Type::GRAY8 : ImgFrame::Type::BGR888i;
            frame = frameFromMat(image, type);
        }
        state->next = index + 1;
        const auto expected = state->source.frames[static_cast<std::size_t>(index)].path;
        if(!std::filesystem::exists(expected)) fail("ReplayVideo frame changed during replay: " + expected.string());
        return frame;
    }
#ifdef DEPTHAI_HAVE_LIBAV
    if(state->libav) {
        while(state->libav->next <= index) {
            auto decoded = readLibavFrame(*state->libav);
            if(state->libav->next - 1 == index) {
                state->next = index + 1;
                return decoded;
            }
        }
    }
#endif
    if(state->opencv) {
        while(state->next <= index) {
            cv::Mat image;
            if(!state->capture.read(image) || image.empty()) fail("ReplayVideo frame decode failed");
            std::shared_ptr<ImgFrame> frame;
            const bool i420 = state->rawYuv && image.type() == CV_8UC1 && image.cols == state->source.width && state->source.height > 0
                              && image.rows == state->source.height * 3 / 2;
            if(i420) {
                if(!image.isContinuous()) image = image.clone();
                std::vector<std::uint8_t> bytes(image.datastart, image.dataend);
                frame = frameFromPacked(ImgFrame::Type::YUV420p, state->source.width, state->source.height, std::move(bytes));
            } else {
                if(image.channels() == 4) cv::cvtColor(image, image, cv::COLOR_BGRA2BGR);
                if(image.channels() == 1) cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
                frame = frameFromMat(image, ImgFrame::Type::BGR888i);
            }
            state->next++;
            if(state->next - 1 == index) return frame;
        }
    }
    fail("ReplayVideo frame decode failed");
}

std::shared_ptr<ImgFrame> convertPixels(const std::shared_ptr<ImgFrame>& frame, ImgFrame::Type outType, int outW, int outH) {
    if(!frame) fail("ReplayVideo frame is missing");
    if(!isReplayOutType(frame->getType())) return frame;
    if((outW % 2) != 0 || (outH % 2) != 0) {
        fail("ReplayVideo size " + std::to_string(outW) + "x" + std::to_string(outH) + " is not supported");
    }
    const int width = static_cast<int>(frame->getWidth());
    const int height = static_cast<int>(frame->getHeight());
    if(width <= 0 || height <= 0 || outW <= 0 || outH <= 0) fail("ReplayVideo frame dimensions are invalid");
    if(!isReplayOutType(outType)) fail(std::string("ReplayVideo output type is not supported: ") + frameTypeName(outType));

    const bool same = frame->getType() == outType && width == outW && height == outH;
    if(same && canonicalLayout(*frame)) return frame;
    auto tight = canonicalLayout(*frame) ? frame : repack(*frame);
    if(same) return tight;

    if(isLuma(tight->getType()) && isLuma(outType)) return convertLuma(*tight, outType, outW, outH);
    return convertColor(*tight, outType, outW, outH);
}

}  // namespace replay
}  // namespace dai
