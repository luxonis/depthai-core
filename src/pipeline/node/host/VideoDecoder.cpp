#include "depthai/pipeline/node/host/VideoDecoder.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <algorithm>
    #include <array>
    #include <limits>
    #include <map>
    #include <opencv2/imgcodecs.hpp>
    #include <opencv2/imgproc.hpp>
    #include <stdexcept>
    #include <string>

    #include "depthai/pipeline/datatype/EncodedFrame.hpp"

    #ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
        #include <wels/codec_api.h>
    #endif

namespace dai {
namespace node {

namespace {
span<uint8_t> getEncodedData(EncodedFrame& frame) {
    auto data = frame.getData();
    const size_t offset = frame.frameOffset;
    if(offset > data.size()) throw std::runtime_error("VideoDecoder: invalid encoded frame offset");
    const size_t size = frame.frameSize == 0 ? data.size() - offset : frame.frameSize;
    if(size > data.size() - offset || size < 2 || size > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error("VideoDecoder: invalid encoded frame size");
    }
    return {data.data() + offset, size};
}

    #ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
struct DecoderDeleter {
    void operator()(ISVCDecoder* decoder) const {
        decoder->Uninitialize();
        WelsDestroyDecoder(decoder);
    }
};

// One decoder per run, so references and pending metadata never survive a restart.
class H264Decoder {
    std::unique_ptr<ISVCDecoder, DecoderDeleter> decoder;
    std::map<uint64_t, ImgFrame> metadata;
    uint64_t nextId = 0;

    static cv::Mat toBgr(const std::array<unsigned char*, 3>& planes, const SBufferInfo& info) {
        const auto& image = info.UsrData.sSystemBuffer;
        const int width = image.iWidth;
        const int height = image.iHeight;
        if(width <= 0 || height <= 0 || width % 2 != 0 || height % 2 != 0 || height > std::numeric_limits<int>::max() / 3 * 2) {
            throw std::runtime_error("VideoDecoder: H.264 requires positive, even image dimensions");
        }
        cv::Mat i420(height + height / 2, width, CV_8UC1);
        size_t offset = 0;
        for(size_t plane = 0; plane < planes.size(); ++plane) {
            const int rows = plane == 0 ? height : height / 2;
            const int cols = plane == 0 ? width : width / 2;
            const int stride = image.iStride[plane == 0 ? 0 : 1];
            if(!planes[plane] || stride < cols) throw std::runtime_error("VideoDecoder: invalid H.264 output plane");
            cv::Mat source(rows, cols, CV_8UC1, planes[plane], stride);
            cv::Mat destination(rows, cols, CV_8UC1, i420.data + offset);
            source.copyTo(destination);
            offset += static_cast<size_t>(rows) * cols;
        }
        cv::Mat bgr;
        cv::cvtColor(i420, bgr, cv::COLOR_YUV2BGR_I420);
        return bgr;
    }

   public:
    H264Decoder() {
        ISVCDecoder* instance = nullptr;
        const auto result = WelsCreateDecoder(&instance);
        decoder.reset(instance);
        if(result != 0 || !decoder) throw std::runtime_error("VideoDecoder: could not create OpenH264 decoder");
        SDecodingParam params{};
        params.sVideoProperty.size = sizeof(params.sVideoProperty);
        params.sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_AVC;
        params.eEcActiveIdc = ERROR_CON_DISABLE;
        if(decoder->Initialize(&params) != 0) throw std::runtime_error("VideoDecoder: could not initialize OpenH264 decoder");
    }

    std::shared_ptr<ImgFrame> decode(EncodedFrame& encoded, span<uint8_t> data, ImgFrame::Type type) {
        SBufferInfo info{};
        info.uiInBsTimeStamp = ++nextId;
        metadata.emplace(nextId, encoded.getImgFrameMeta());
        // H.264 has at most 16 reference pictures. Allow extra entries for header-only packets,
        // but do not retain metadata indefinitely when such packets produce no output.
        constexpr size_t MAX_PENDING_METADATA = 64;
        if(metadata.size() > MAX_PENDING_METADATA) metadata.erase(metadata.begin());
        std::array<unsigned char*, 3> planes{};
        const auto status = decoder->DecodeFrameNoDelay(data.data(), static_cast<int>(data.size()), planes.data(), &info);
        if((status & ~dsFramePending) != dsErrorFree) {
            throw std::runtime_error("VideoDecoder: H.264 decoding failed (OpenH264 status " + std::to_string(status) + ")");
        }
        if(info.iBufferStatus == 0) return nullptr;
        auto entry = metadata.find(info.uiOutYuvTimeStamp);
        if(entry == metadata.end()) throw std::runtime_error("VideoDecoder: missing H.264 output metadata");
        auto frame = std::make_shared<ImgFrame>(entry->second);
        metadata.erase(entry);
        frame->setCvFrame(toBgr(planes, info), type);
        return frame;
    }
};
    #endif
}  // namespace

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
    #ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
    std::unique_ptr<H264Decoder> h264;
    #endif
    try {
        while(mainLoop()) {
            std::shared_ptr<EncodedFrame> encoded;
            {
                auto blockEvent = inputBlockEvent();
                encoded = input.get<EncodedFrame>();
            }
            if(!encoded) continue;
            auto data = getEncodedData(*encoded);
            const auto type = getOutFrameType();
            std::shared_ptr<ImgFrame> frame;
            switch(encoded->getProfile()) {
                case EncodedFrame::Profile::JPEG: {
    #ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
                    h264.reset();
    #endif
                    // imdecode also accepts other image formats; require a JPEG start marker.
                    if(data[0] != 0xff || data[1] != 0xd8) throw std::runtime_error("VideoDecoder: expected a JPEG image");
                    cv::Mat bytes(1, static_cast<int>(data.size()), CV_8UC1, data.data());
                    auto decoded = cv::imdecode(bytes, cv::IMREAD_COLOR | cv::IMREAD_IGNORE_ORIENTATION);
                    if(decoded.empty()) throw std::runtime_error("VideoDecoder: failed to decode JPEG image");
                    if((type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12 || type == ImgFrame::Type::NV21)
                       && (decoded.cols % 2 != 0 || decoded.rows % 2 != 0)) {
                        throw std::runtime_error("VideoDecoder: YUV output requires even image dimensions");
                    }
                    frame = std::make_shared<ImgFrame>(encoded->getImgFrameMeta());
                    frame->setCvFrame(decoded, type);
                    break;
                }
                case EncodedFrame::Profile::AVC:
    #ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
                    if(!h264) h264 = std::make_unique<H264Decoder>();
                    frame = h264->decode(*encoded, data, type);
                    break;
    #else
                    throw std::runtime_error("VideoDecoder: H.264 decoding requires DEPTHAI_ENABLE_OPENH264=ON");
    #endif
                case EncodedFrame::Profile::HEVC:
                default:
                    throw std::runtime_error("VideoDecoder: unsupported encoding profile (H.265 is not supported)");
            }
            if(frame) {
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
