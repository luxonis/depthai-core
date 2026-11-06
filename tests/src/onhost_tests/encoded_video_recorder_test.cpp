#include <mp4v2.h>

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <filesystem>
#include <vector>

#include "utility/EncodedVideoRecorder.hpp"

namespace {

struct RecordingFile {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / ("dai-encoded-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".mp4");
    ~RecordingFile() {
        std::error_code error;
        std::filesystem::remove(path, error);
    }
};

const std::vector<std::uint8_t> kPredicted{0, 0, 0, 1, 0x41, 0x9a, 0x20};
const std::vector<std::uint8_t> kKeyframe{0, 0, 0,    1,    0x67, 0x42, 0, 0x1e, 0xe9, 0x01, 0x40, 0x7b, 0x20, 0,   0,
                                          0, 1, 0x68, 0xce, 0x06, 0xe2, 0, 0,    0,    1,    0x65, 0x88, 0x84, 0x21};
const std::vector<std::uint8_t> kIdrOnly{0, 0, 0, 1, 0x65, 0x88, 0x84, 0x21};

std::uint32_t sampleCount(const std::filesystem::path& path) {
    const auto reader = MP4Read(path.string().c_str());
    REQUIRE(reader != MP4_INVALID_FILE_HANDLE);
    const auto track = MP4FindTrackId(reader, 0, MP4_VIDEO_TRACK_TYPE);
    const auto count = MP4GetTrackNumberOfSamples(reader, track);
    MP4Close(reader);
    return count;
}

}  // namespace

TEST_CASE("Encoded recorder retains H264 samples before the next SPS") {
    RecordingFile file;
    dai::utility::EncodedVideoRecorder writer;
    writer.init(file.path.string(), 64, 48, 29.97, false);
    // The first access unit of a triggered segment can be a P slice. Its sample
    // must stay in the container even though decoding needs the later keyframe.
    writer.write(kPredicted);
    writer.write(kKeyframe);
    writer.write(kPredicted);
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

TEST_CASE("Encoded recorder reads H264 parameter sets without opening a file") {
    const auto sets = dai::utility::EncodedVideoRecorder::peekH264ParameterSets(kKeyframe);
    REQUIRE(sets.sps.size() >= 4);
    REQUIRE(sets.sps[0] == 0x67);
    REQUIRE(sets.pps.size() >= 1);
    REQUIRE(sets.pps[0] == 0x68);
    const auto predicted = dai::utility::EncodedVideoRecorder::peekH264ParameterSets(kPredicted);
    REQUIRE(predicted.sps.empty());
    REQUIRE(predicted.pps.empty());
}

TEST_CASE("Encoded recorder starts a later H264 segment on a cached keyframe") {
    RecordingFile first;
    dai::utility::EncodedVideoRecorder writer;
    writer.init(first.path.string(), 64, 48, 30, false);
    writer.write(kKeyframe);
    const auto sets = writer.copyH264ParameterSets();
    writer.close();
    REQUIRE(sets.sps.size() >= 4);
    REQUIRE(sets.pps.size() >= 1);
    REQUIRE(sampleCount(first.path) == 1);

    RecordingFile second;
    writer.init(second.path.string(), 64, 48, 30, false);
    writer.setWaitForKeyframe(true);
    writer.setH264ParameterSets(sets.sps, sets.pps);
    // A mid-GOP P slice must not become the first sample of the new file.
    REQUIRE(writer.write(kPredicted) == false);
    REQUIRE(writer.write(kIdrOnly) == true);
    REQUIRE(writer.write(kPredicted) == true);
    const auto copied = writer.copyH264ParameterSets();
    writer.close();
    REQUIRE(copied.sps == sets.sps);
    REQUIRE(copied.pps == sets.pps);
    REQUIRE(sampleCount(second.path) == 2);

    const auto reader = MP4Read(second.path.string().c_str());
    REQUIRE(reader != MP4_INVALID_FILE_HANDLE);
    const auto track = MP4FindTrackId(reader, 0, MP4_VIDEO_TRACK_TYPE);
    std::uint8_t** seq = nullptr;
    std::uint32_t* seqSize = nullptr;
    std::uint8_t** pict = nullptr;
    std::uint32_t* pictSize = nullptr;
    REQUIRE(MP4GetTrackH264SeqPictHeaders(reader, track, &seq, &seqSize, &pict, &pictSize));
    REQUIRE(seqSize != nullptr);
    REQUIRE(seqSize[0] == sets.sps.size());
    REQUIRE(std::equal(sets.sps.begin(), sets.sps.end(), seq[0]));
    REQUIRE(pictSize != nullptr);
    REQUIRE(pictSize[0] == sets.pps.size());
    REQUIRE(std::equal(sets.pps.begin(), sets.pps.end(), pict[0]));
    MP4FreeH264SeqPictHeaders(seq, seqSize, pict, pictSize);
    MP4Close(reader);
}
