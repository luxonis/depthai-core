#include "depthai/pipeline/node/host/VideoDecoder.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <algorithm>
    #include <array>
    #include <limits>
    #include <opencv2/imgcodecs.hpp>
    #include <stdexcept>
    #include <string>

    #include "depthai/pipeline/datatype/EncodedFrame.hpp"

namespace dai {
namespace node {

VideoDecoder& VideoDecoder::setOutFrameType(ImgFrame::Type type) {
    constexpr std::array<ImgFrame::Type, 9> supportedTypes = {ImgFrame::Type::BGR888i,
                                                              ImgFrame::Type::RGB888i,
                                                              ImgFrame::Type::BGR888p,
                                                              ImgFrame::Type::RGB888p,
                                                              ImgFrame::Type::GRAY8,
                                                              ImgFrame::Type::RAW8,
                                                              ImgFrame::Type::YUV420p,
                                                              ImgFrame::Type::NV12,
                                                              ImgFrame::Type::NV21};
    if(std::find(supportedTypes.begin(), supportedTypes.end(), type) == supportedTypes.end()) {
        throw std::invalid_argument("VideoDecoder: unsupported output image type");
    }
    outFrameType.store(type);
    return *this;
}

ImgFrame::Type VideoDecoder::getOutFrameType() const {
    return outFrameType.load();
}

void VideoDecoder::run() {
    try {
        while(mainLoop()) {
            std::shared_ptr<EncodedFrame> encoded;
            {
                auto blockEvent = inputBlockEvent();
                encoded = input.get<EncodedFrame>();
            }
            if(!encoded) continue;

            cv::Mat decoded;
            switch(encoded->getProfile()) {
                case EncodedFrame::Profile::JPEG: {
                    auto data = encoded->getData();
                    const size_t offset = encoded->frameOffset;
                    if(offset > data.size()) throw std::runtime_error("VideoDecoder: invalid encoded frame offset");
                    const size_t size = encoded->frameSize == 0 ? data.size() - offset : encoded->frameSize;
                    if(size > data.size() - offset || size < 2 || size > static_cast<size_t>(std::numeric_limits<int>::max())) {
                        throw std::runtime_error("VideoDecoder: invalid encoded frame size");
                    }
                    // imdecode also accepts other image formats; require a JPEG start marker.
                    if(data[offset] != 0xff || data[offset + 1] != 0xd8) {
                        throw std::runtime_error("VideoDecoder: expected a JPEG image");
                    }
                    cv::Mat bytes(1, static_cast<int>(size), CV_8UC1, data.data() + offset);
                    decoded = cv::imdecode(bytes, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
                    break;
                }
                case EncodedFrame::Profile::AVC:
                case EncodedFrame::Profile::HEVC:
                default:
                    throw std::runtime_error("VideoDecoder: unsupported encoding profile; only JPEG (MJPEG) is supported");
            }
            if(decoded.empty()) throw std::runtime_error("VideoDecoder: failed to decode JPEG image");

            const auto type = getOutFrameType();
            if((type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12 || type == ImgFrame::Type::NV21)
               && (decoded.cols % 2 != 0 || decoded.rows % 2 != 0)) {
                throw std::runtime_error("VideoDecoder: YUV output requires even image dimensions");
            }
            auto frame = std::make_shared<ImgFrame>(encoded->getImgFrameMeta());
            frame->setCvFrame(decoded, type);
            {
                auto blockEvent = outputBlockEvent();
                out.send(frame);
            }
        }
    } catch(const cv::Exception& ex) {
        throw std::runtime_error(std::string("VideoDecoder: ") + ex.what());
    }
}

}  // namespace node
}  // namespace dai

#endif
