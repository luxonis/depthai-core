#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>

namespace dai {
namespace utility {

// How an H.264 recording turns maxFrames / maxDurationMs into the next file.
// The cut is armed this many written frames before maxFrames, and two frame
// periods before maxDurationMs. The frame that crosses the mark stays in the
// current segment. The next IDR opens the next one. RAW_FRAMES and MJPEG do
// not use these helpers.

inline std::int32_t h264ArmAfterFrames(std::int32_t maxFrames) {
    if(maxFrames <= 2) return 1;
    return maxFrames - 2;
}

inline bool h264FrameCutDue(std::int32_t written, std::int32_t maxFrames) {
    return maxFrames > 0 && written >= h264ArmAfterFrames(maxFrames);
}

inline bool h264DurationCutDue(std::chrono::steady_clock::time_point start,
                               std::chrono::steady_clock::time_point now,
                               std::int32_t maxDurationMs,
                               double fps) {
    if(maxDurationMs <= 0) return false;
    double frameMs = 33.0;
    if(std::isfinite(fps) && fps > 0.0) frameMs = 1000.0 / fps;
    if(frameMs < 1.0) frameMs = 1.0;
    auto lead = static_cast<std::int32_t>(std::llround(frameMs * 2.0));
    if(lead < 1) lead = 1;
    if(lead > maxDurationMs) lead = maxDurationMs;
    const auto mark = start + std::chrono::milliseconds(static_cast<std::int32_t>(maxDurationMs - lead));
    return now >= mark;
}

}  // namespace utility
}  // namespace dai
