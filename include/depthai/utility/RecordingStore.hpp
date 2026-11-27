#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace dai {

/**
 * One recording in the device store.
 */
struct RecordingInfo {
    std::string name;
    std::uint64_t sizeBytes = 0;
    std::int64_t createdMs = 0;
    bool complete = false;
};

/**
 * Reject a recording name that fails the record-node name rules.
 * @throws std::invalid_argument naming recordingName and the value.
 */
void validateRecordingName(const std::string& name);

/**
 * Reject a stream name that fails the record-node name rules.
 * @throws std::invalid_argument naming streamName and the value.
 */
void validateStreamName(const std::string& name);

/**
 * Default recording directory name from the host local clock, with milliseconds.
 */
std::string makeDefaultRecordingName();

/**
 * Host local time as YYYY-MM-DD_HH-MM-SS-mmm.
 */
std::string makeRecordingTimestamp();

/**
 * Insert _s<segment> before the extension. Segment 0 returns the path unchanged.
 */
std::filesystem::path pathWithSegment(const std::filesystem::path& file, int segment);

}  // namespace dai
