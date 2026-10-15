#pragma once

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <atomic>

    #include "depthai/pipeline/ThreadedHostNode.hpp"
    #include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace node {

/**
 * @brief Decode VideoEncoder EncodedFrames on the host. Requires OpenCV support.
 *
 * Currently supports JPEG (MJPEG) only. Unsupported profiles or invalid frames
 * stop the pipeline with an error. Each input must contain one complete encoded image.
 * A nonzero frameSize selects that many bytes at frameOffset; zero uses the remaining payload.
 */
class VideoDecoder : public NodeCRTP<ThreadedHostNode, VideoDecoder> {
   public:
    constexpr static const char* NAME = "VideoDecoder";

    /** Encoded frame input. Default queue is blocking with size 4. */
    Input input{*this, {"input", DEFAULT_GROUP, true, 4, {{DatatypeEnum::EncodedFrame, false}}}};
    /** Decoded image output, preserving input timestamps and camera metadata. */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Set the output image type (default BGR888i). May be changed at runtime.
     * Supports BGR888i, RGB888i, BGR888p, RGB888p, GRAY8, RAW8, YUV420p, NV12 and NV21.
     * YUV output requires even image dimensions.
     * @param type Output image type. Unsupported types throw std::invalid_argument.
     */
    VideoDecoder& setOutFrameType(ImgFrame::Type type);
    /** Get the configured output image type. */
    ImgFrame::Type getOutFrameType() const;
    /** Run the host decoding loop. */
    void run() override;

   private:
    std::atomic<ImgFrame::Type> outFrameType{ImgFrame::Type::BGR888i};
};

}  // namespace node
}  // namespace dai

#endif
