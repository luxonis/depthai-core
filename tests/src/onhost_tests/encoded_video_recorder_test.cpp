#include <catch2/catch_all.hpp>
#include <chrono>
#include <filesystem>
#include <mp4v2.h>
#include <vector>

#include "utility/EncodedVideoRecorder.hpp"

namespace {

struct RecordingFile {
    const std::filesystem::path path = std::filesystem::temp_directory_path()
                                       / ("dai-encoded-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".mp4");
    ~RecordingFile() {
        std::error_code error;
        std::filesystem::remove(path, error);
    }
};

}  // namespace

TEST_CASE("Encoded recorder retains H264 samples before the next SPS") {
    RecordingFile file;
    dai::utility::EncodedVideoRecorder writer;
    writer.init(file.path.string(), 64, 48, 29.97, false);
    // The first access unit of a triggered segment can be a P slice. Its sample
    // must stay in the container even though decoding needs the later keyframe.
    const std::vector<std::uint8_t> predicted{0, 0, 0, 1, 0x41, 0x9a, 0x20};
    const std::vector<std::uint8_t> keyframe{
        0, 0, 0, 1, 0x67, 0x42, 0, 0x1e, 0xe9, 0x01, 0x40, 0x7b, 0x20,
        0, 0, 0, 1, 0x68, 0xce, 0x06, 0xe2,
        0, 0, 0, 1, 0x65, 0x88, 0x84, 0x21};
    writer.write(predicted);
    writer.write(keyframe);
    writer.write(predicted);
    writer.close();
    const auto reader = MP4Read(file.path.string().c_str());
    REQUIRE(reader != MP4_INVALID_FILE_HANDLE);
    const auto track = MP4FindTrackId(reader, 0, MP4_VIDEO_TRACK_TYPE);
    const auto count = MP4GetTrackNumberOfSamples(reader, track);
    const auto duration = MP4GetTrackDuration(reader, track);
    const auto timescale = MP4GetTrackTimeScale(reader, track);
    MP4Close(reader);
    REQUIRE(count == 3);
    REQUIRE(static_cast<double>(duration) / timescale == Catch::Approx(3.0 / 29.97).epsilon(0.0001));
}

TEST_CASE("Encoded recorder writes every MJPEG sample and can reopen for a new segment") {
    RecordingFile first;
    RecordingFile second;
    dai::utility::EncodedVideoRecorder writer;
    // The muxer treats JPEG payload as opaque; decoding is tested with real camera input on RVC4.
    const std::vector<std::uint8_t> jpeg{0xff, 0xd8, 0xff, 0xd9};
    for(const auto& path : {first.path, second.path}) {
        writer.init(path.string(), 64, 48, 10, true);
        writer.write(jpeg);
        writer.write(jpeg);
        writer.close();
        const auto reader = MP4Read(path.string().c_str());
        REQUIRE(reader != MP4_INVALID_FILE_HANDLE);
        const auto track = MP4FindTrackId(reader, 0, MP4_VIDEO_TRACK_TYPE);
        const auto count = MP4GetTrackNumberOfSamples(reader, track);
        MP4Close(reader);
        REQUIRE(count == 2);
    }
}
