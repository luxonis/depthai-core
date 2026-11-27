#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "depthai/utility/span.hpp"

namespace dai {
namespace utility {

struct H264ParameterSets {
    std::vector<std::uint8_t> sps;
    std::vector<std::uint8_t> pps;
};

// The encoded MP4 backend is independent of OpenCV and is shared with RVC4.
class EncodedVideoRecorder {
   public:
    EncodedVideoRecorder();
    ~EncodedVideoRecorder();
    void init(const std::string& filePath, unsigned int width, unsigned int height, double fps, bool mjpeg);
    // Copies SPS and PPS into this file before the first sample. A later segment
    // can then be decoded without waiting for the encoder to repeat them in-band.
    void setH264ParameterSets(span<const std::uint8_t> sps, span<const std::uint8_t> pps);
    H264ParameterSets copyH264ParameterSets() const;
    // When true, samples before the first IDR are not written. Parameter sets are still kept.
    void setWaitForKeyframe(bool wait);
    // Reads SPS and PPS out of an Annex-B access unit. Absent sets are empty.
    // Does not open a file, so a recorder can remember them before the first segment.
    static H264ParameterSets peekH264ParameterSets(span<const std::uint8_t> data);
    // True when the access unit contains an IDR slice (NAL type 5). A non-IDR
    // intra slice is not a segment start: later P slices can still reference
    // pictures from before it.
    static bool containsIdr(span<const std::uint8_t> data);
    // False when the frame was dropped while waiting for an IDR.
    bool write(span<const uint8_t> data);
    void close();

   private:
    struct State;
    std::unique_ptr<State> state;
};

}  // namespace utility
}  // namespace dai
