#include <catch2/catch_all.hpp>
#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "depthai/depthai.hpp"

namespace {
std::shared_ptr<dai::EncodedFrame> makeFrame(int width = 32, int height = 24, bool padded = false) {
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(30, 90, 180));
    std::vector<uint8_t> jpeg;
    REQUIRE(cv::imencode(".jpg", image, jpeg));
    auto frame = std::make_shared<dai::EncodedFrame>();
    frame->setProfile(dai::EncodedFrame::Profile::JPEG);
    frame->setSize(width, height);
    frame->transformation = dai::ImgTransformation(width, height);
    frame->setSequenceNum(42);
    frame->setTimestamp(std::chrono::steady_clock::now());
    frame->setTimestampSystem(std::chrono::system_clock::now());
    frame->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::seconds(10)));
    frame->setInstanceNum(3);
    frame->cam.sensitivityIso = 200;
    if(padded) {
        frame->frameOffset = 7;
        frame->frameSize = static_cast<uint32_t>(jpeg.size());
        jpeg.insert(jpeg.begin(), 7, 0);
        jpeg.insert(jpeg.end(), 5, 0);
    }
    frame->setData(jpeg);
    return frame;
}
}  // namespace

TEST_CASE("VideoDecoder decodes MJPEG and converts output at runtime", "[video-decoder]") {
    const bool padded = GENERATE(false, true);
    dai::Pipeline pipeline(false);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    REQUIRE(decoder->getOutFrameType() == dai::ImgFrame::Type::BGR888i);
    REQUIRE_THROWS_AS(decoder->setOutFrameType(dai::ImgFrame::Type::BITSTREAM), std::invalid_argument);
    auto input = decoder->input.createInputQueue();
    auto output = decoder->out.createOutputQueue();
    pipeline.start();
    using Type = dai::ImgFrame::Type;
    for(auto type : {Type::BGR888i, Type::RGB888i, Type::BGR888p, Type::RGB888p, Type::GRAY8, Type::RAW8, Type::YUV420p, Type::NV12, Type::NV21}) {
        CAPTURE(type);
        decoder->setOutFrameType(type);
        auto encoded = makeFrame(32, 24, padded);
        input->send(encoded);
        bool timedOut = false;
        auto decoded = output->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(decoded);
        REQUIRE(decoded->getType() == type);
        REQUIRE(decoded->getWidth() == 32);
        REQUIRE(decoded->getHeight() == 24);
        REQUIRE(decoded->getSequenceNum() == encoded->getSequenceNum());
        REQUIRE(decoded->getTimestamp() == encoded->getTimestamp());
        REQUIRE(decoded->getTimestampSystem() == encoded->getTimestampSystem());
        REQUIRE(decoded->getTimestampDevice() == encoded->getTimestampDevice());
        REQUIRE(decoded->getInstanceNum() == 3);
        REQUIRE(decoded->cam.sensitivityIso == 200);
        REQUIRE(decoded->transformation.getSourceSize() == encoded->transformation.getSourceSize());
        REQUIRE(decoded->transformation.getTransformationMatrix() == encoded->transformation.getTransformationMatrix());
        auto image = decoded->getCvFrame();
        const auto mean = cv::mean(image);
        if(type == Type::GRAY8 || type == Type::RAW8) {
            REQUIRE(image.channels() == 1);
            REQUIRE(std::abs(mean[0] - 110) < 4);
        } else {
            REQUIRE(image.channels() == 3);
            REQUIRE(std::abs(mean[0] - 30) < 5);
            REQUIRE(std::abs(mean[1] - 90) < 5);
            REQUIRE(std::abs(mean[2] - 180) < 5);
        }
    }
    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("VideoDecoder rejects invalid frames without terminating the process", "[video-decoder]") {
    dai::Pipeline pipeline(false);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    auto input = decoder->input.createInputQueue();
    auto output = decoder->out.createOutputQueue();
    auto frame = makeFrame();
    SECTION("H264") {
        frame->setProfile(dai::EncodedFrame::Profile::AVC);
    }
    SECTION("H265") {
        frame->setProfile(dai::EncodedFrame::Profile::HEVC);
    }
    SECTION("empty") {
        frame->setData(std::vector<uint8_t>{});
    }
    SECTION("invalid offset") {
        frame->frameOffset = static_cast<uint32_t>(frame->getData().size() + 1);
    }
    SECTION("invalid size") {
        frame->frameSize = static_cast<uint32_t>(frame->getData().size() + 1);
    }
    SECTION("invalid JPEG") {
        frame->setData(std::vector<uint8_t>{0xff, 0xd8, 0, 0});
    }
    SECTION("wrong image format") {
        frame->setData(std::vector<uint8_t>{0x89, 0x50, 0x4e, 0x47});
    }
    SECTION("odd YUV dimensions") {
        frame = makeFrame(31, 23);
        decoder->setOutFrameType(dai::ImgFrame::Type::NV12);
    }
    pipeline.start();
    input->send(frame);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(decoder->isRunning() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    REQUIRE_FALSE(decoder->isRunning());
    pipeline.stop();
    pipeline.wait();
    REQUIRE_FALSE(pipeline.isRunning());
}

#ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
    #include <wels/codec_api.h>

namespace {
struct EncoderDeleter {
    void operator()(ISVCEncoder* encoder) const {
        encoder->Uninitialize();
        WelsDestroySVCEncoder(encoder);
    }
};

std::vector<std::shared_ptr<dai::EncodedFrame>> makeH264Frames() {
    ISVCEncoder* instance = nullptr;
    const auto result = WelsCreateSVCEncoder(&instance);
    std::unique_ptr<ISVCEncoder, EncoderDeleter> encoder(instance);
    REQUIRE(result == 0);
    REQUIRE(encoder);
    SEncParamBase params{};
    params.iUsageType = CAMERA_VIDEO_REAL_TIME;
    params.iPicWidth = 32;
    params.iPicHeight = 24;
    params.iTargetBitrate = 1000000;
    params.iRCMode = RC_OFF_MODE;
    params.fMaxFrameRate = 30;
    REQUIRE(encoder->Initialize(&params) == 0);
    std::vector<std::shared_ptr<dai::EncodedFrame>> frames;
    for(int i = 0; i < 20; ++i) {
        std::vector<uint8_t> pixels(32 * 24 * 3 / 2, 128);
        std::fill_n(pixels.begin(), 32 * 24, 40 + i * 5);
        SSourcePicture picture{};
        picture.iColorFormat = videoFormatI420;
        picture.iPicWidth = 32;
        picture.iPicHeight = 24;
        picture.iStride[0] = 32;
        picture.iStride[1] = picture.iStride[2] = 16;
        picture.pData[0] = pixels.data();
        picture.pData[1] = pixels.data() + 32 * 24;
        picture.pData[2] = pixels.data() + 32 * 24 * 5 / 4;
        picture.uiTimeStamp = i * 33;
        SFrameBSInfo info{};
        REQUIRE(encoder->EncodeFrame(&picture, &info) == 0);
        REQUIRE(info.eFrameType == (i == 0 ? videoFrameTypeIDR : videoFrameTypeP));
        std::vector<uint8_t> bytes(7, 0);
        for(int layer = 0; layer < info.iLayerNum; ++layer) {
            const auto& data = info.sLayerInfo[layer];
            int size = 0;
            for(int nal = 0; nal < data.iNalCount; ++nal) size += data.pNalLengthInByte[nal];
            bytes.insert(bytes.end(), data.pBsBuf, data.pBsBuf + size);
        }
        auto frame = makeFrame();
        frame->setProfile(dai::EncodedFrame::Profile::AVC);
        frame->setSequenceNum(i);
        frame->frameOffset = 7;
        frame->frameSize = static_cast<uint32_t>(bytes.size() - 7);
        bytes.insert(bytes.end(), 5, 0);
        frame->setData(bytes);
        frames.push_back(frame);
    }
    return frames;
}
}  // namespace

TEST_CASE("VideoDecoder decodes H264 reference frames and preserves metadata", "[video-decoder][h264]") {
    const auto frames = makeH264Frames();
    dai::Pipeline pipeline(false);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    auto input = decoder->input.createInputQueue();
    auto output = decoder->out.createOutputQueue();
    pipeline.start();
    using Type = dai::ImgFrame::Type;
    const std::array<Type, 9> types = {
        Type::BGR888i, Type::RGB888i, Type::BGR888p, Type::RGB888p, Type::GRAY8, Type::RAW8, Type::YUV420p, Type::NV12, Type::NV21};
    const auto firstData = frames.front()->getData();
    const std::array<uint8_t, 5> idrStart = {0, 0, 0, 1, 0x65};
    const auto idr = std::search(firstData.begin() + 7, firstData.end(), idrStart.begin(), idrStart.end());
    REQUIRE(idr != firstData.end());
    auto headers = std::make_shared<dai::EncodedFrame>(*frames.front());
    headers->setData(std::vector<uint8_t>(firstData.begin() + 7, idr));
    headers->frameOffset = 0;
    headers->frameSize = 0;
    headers->setSequenceNum(999);
    // Switching through JPEG resets decoder references; the next IDR starts a fresh stream.
    for(int stream = 0; stream < 2; ++stream) {
        input->send(headers);
        bool headersTimedOut = false;
        REQUIRE_FALSE(output->get<dai::ImgFrame>(std::chrono::milliseconds(100), headersTimedOut));
        REQUIRE(headersTimedOut);
        for(size_t i = 0; i < frames.size(); ++i) {
            const auto type = types[i % types.size()];
            decoder->setOutFrameType(type);
            input->send(frames[i]);
            bool timedOut = false;
            auto decoded = output->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(decoded);
            REQUIRE(decoded->getType() == type);
            REQUIRE(decoded->getWidth() == 32);
            REQUIRE(decoded->getHeight() == 24);
            REQUIRE(decoded->getSequenceNum() == frames[i]->getSequenceNum());
            REQUIRE(decoded->getTimestamp() == frames[i]->getTimestamp());
            REQUIRE(decoded->getTimestampDevice() == frames[i]->getTimestampDevice());
            REQUIRE(decoded->getTimestampSystem() == frames[i]->getTimestampSystem());
            REQUIRE(decoded->cam.sensitivityIso == 200);
            REQUIRE(decoded->transformation.getTransformationMatrix() == frames[i]->transformation.getTransformationMatrix());
            const double expected = (40 + i * 5 - 16) * 255.0 / 219.0;
            REQUIRE(std::abs(cv::mean(decoded->getCvFrame())[0] - expected) < 5);
        }
        input->send(makeFrame());
        bool timedOut = false;
        REQUIRE(output->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut));
        REQUIRE_FALSE(timedOut);
    }
    pipeline.stop();
    pipeline.wait();
}
#endif

#ifdef DEPTHAI_HAVE_OPENH264_SUPPORT
    #include "VideoDecoderH264Fixture.hpp"

TEST_CASE("VideoDecoder preserves H264 B-frame presentation order and metadata", "[video-decoder][h264]") {
    using namespace video_decoder_test;
    dai::Pipeline pipeline(false);
    auto decoder = pipeline.create<dai::node::VideoDecoder>();
    auto input = decoder->input.createInputQueue();
    auto output = decoder->out.createOutputQueue();
    const auto timestamp = std::chrono::steady_clock::now();
    pipeline.start();
    size_t offset = 0;
    for(size_t i = 0; i < PACKET_SIZES.size(); ++i) {
        auto frame = makeFrame(32, 32);
        frame->setProfile(dai::EncodedFrame::Profile::AVC);
        frame->setSequenceNum(DISPLAY_ORDER[i]);
        frame->setTimestamp(timestamp + std::chrono::milliseconds(DISPLAY_ORDER[i] * 33));
        frame->setData(std::vector<uint8_t>(B_FRAME_STREAM.begin() + offset, B_FRAME_STREAM.begin() + offset + PACKET_SIZES[i]));
        offset += PACKET_SIZES[i];
        input->send(frame);
    }
    // The final reordered pictures remain buffered until more input arrives or the stream is drained.
    for(int i = 0; i < 10; ++i) {
        bool timedOut = false;
        auto decoded = output->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(decoded);
        REQUIRE(decoded->getSequenceNum() == i);
        REQUIRE(decoded->getTimestamp() == timestamp + std::chrono::milliseconds(i * 33));
        REQUIRE(std::abs(cv::mean(decoded->getCvFrame())[0] - (40 + i * 5 - 16) * 255.0 / 219.0) < 5);
    }
    pipeline.stop();
    pipeline.wait();
}
#endif
