#pragma once

#include <memory>
#include <string>

#include "depthai/utility/span.hpp"

namespace dai {
namespace utility {

// The encoded MP4 backend is independent of OpenCV and is shared with RVC4.
class EncodedVideoRecorder {
   public:
    EncodedVideoRecorder();
    ~EncodedVideoRecorder();
    void init(const std::string& filePath, unsigned int width, unsigned int height, double fps, bool mjpeg);
    void write(span<const uint8_t> data);
    void close();

   private:
    struct State;
    std::unique_ptr<State> state;
};

}  // namespace utility
}  // namespace dai
