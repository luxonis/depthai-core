#include <catch2/catch_all.hpp>
#include <cstring>
#include <trompeloeil.hpp>

#include "../../../../../../src/pipeline/datatype/PacketizedData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/pipeline/node/internal/XLinkInHost.hpp"

namespace dai {
namespace node {
namespace internal {

class XLinkInHostTestable : public XLinkInHost {
   public:
    using dai::node::internal::XLinkInHost::parseMessageGroup;
    using dai::node::internal::XLinkInHost::parsePacketizedData;
    using dai::node::internal::XLinkInHost::readData;

    MAKE_CONST_MOCK0(readStreamMessage, dai::StreamPacketDesc(), override);
};
}  // namespace internal
}  // namespace node
}  // namespace dai

using trompeloeil::_;  // for wildcard matching if needed

dai::StreamPacketDesc getRawBuffer(std::shared_ptr<dai::ImgFrame> frame) {
    auto span = frame->getData();
    std::vector<std::uint8_t> pixelData(span.begin(), span.end());
    std::vector<std::uint8_t> metadata = dai::StreamMessageParser::serializeMetadata(frame);

    size_t totalSize = pixelData.size() + metadata.size();
    void* rawBuffer = std::malloc(totalSize);
    std::memcpy(rawBuffer, pixelData.data(), pixelData.size());
    std::memcpy(static_cast<uint8_t*>(rawBuffer) + pixelData.size(), metadata.data(), metadata.size());
    dai::StreamPacketDesc packetFrame;
    packetFrame.data = static_cast<uint8_t*>(rawBuffer);
    packetFrame.length = totalSize;
    packetFrame.fd = -1;
    return packetFrame;
}

dai::StreamPacketDesc getRawBuffer(std::shared_ptr<dai::MessageGroup> messageGroup) {
    std::vector<std::uint8_t> metadata = dai::StreamMessageParser::serializeMetadata(messageGroup);

    size_t totalSize = metadata.size();
    void* rawBuffer = std::malloc(totalSize);
    std::memcpy(static_cast<uint8_t*>(rawBuffer), metadata.data(), metadata.size());
    dai::StreamPacketDesc packetFrame;
    packetFrame.data = static_cast<uint8_t*>(rawBuffer);
    packetFrame.length = totalSize;
    packetFrame.fd = -1;
    return packetFrame;
}

dai::StreamPacketDesc getRawBuffer(std::shared_ptr<dai::PacketizedData> packetizedData) {
    std::vector<std::uint8_t> metadata = dai::StreamMessageParser::serializeMetadata(packetizedData);
    size_t totalSize = metadata.size();
    void* rawBuffer = std::malloc(totalSize);
    std::memcpy(static_cast<uint8_t*>(rawBuffer), metadata.data(), metadata.size());
    dai::StreamPacketDesc packetFrame;
    packetFrame.data = static_cast<uint8_t*>(rawBuffer);
    packetFrame.length = totalSize;
    packetFrame.fd = -1;
    return packetFrame;
}

std::vector<dai::StreamPacketDesc> getPackets(std::shared_ptr<dai::ImgFrame> frame, size_t maxPacketSize) {
    // 1. Prepare the full buffer
    auto span = frame->getData();
    std::vector<std::uint8_t> pixelData(span.begin(), span.end());
    std::vector<std::uint8_t> metadata = dai::StreamMessageParser::serializeMetadata(frame);

    std::vector<uint8_t> fullMessage;
    fullMessage.insert(fullMessage.end(), pixelData.begin(), pixelData.end());
    fullMessage.insert(fullMessage.end(), metadata.begin(), metadata.end());

    std::vector<dai::StreamPacketDesc> result;
    size_t totalSize = fullMessage.size();
    size_t offset = 0;

    while(offset < totalSize) {
        size_t currentChunkSize = std::min(maxPacketSize, totalSize - offset);

        void* chunkBuffer = std::malloc(currentChunkSize);
        std::memcpy(chunkBuffer, fullMessage.data() + offset, currentChunkSize);

        dai::StreamPacketDesc packet;
        packet.data = static_cast<uint8_t*>(chunkBuffer);
        packet.length = currentChunkSize;
        packet.fd = -1;

        result.push_back(std::move(packet));
        offset += currentChunkSize;
    }
    return result;
}

TEST_CASE("XLinkInHost - readData") {
    using namespace dai::node::internal;

    XLinkInHostTestable xlinkIn;
    REQUIRE(0 == 0);

    SECTION("ImgFrame") {
        auto frame = std::make_shared<dai::ImgFrame>();
        cv::Mat mat(2, 2, CV_8UC1, cv::Scalar(1));
        frame->setCvFrame(mat, dai::ImgFrame::Type::GRAY8);

        dai::StreamPacketDesc packet = getRawBuffer(frame);

        REQUIRE_CALL(xlinkIn, readStreamMessage()).LR_RETURN(std::move(packet));

        auto result = xlinkIn.readData();
        REQUIRE(result != nullptr);
        auto img = std::dynamic_pointer_cast<dai::ImgFrame>(result);
        REQUIRE(img != nullptr);
        REQUIRE(img->getType() == dai::ImgFrame::Type::GRAY8);
        auto cvFrame = img->getCvFrame();
        REQUIRE(cvFrame.rows == mat.rows);
        REQUIRE(cvFrame.cols == mat.cols);
        REQUIRE(cv::countNonZero(cvFrame != mat) == 0);
    }

    SECTION("MessageGroup") {
        auto frame1 = std::make_shared<dai::ImgFrame>();
        cv::Mat mat1(2, 2, CV_8UC1, cv::Scalar(1));
        frame1->setCvFrame(mat1, dai::ImgFrame::Type::GRAY8);

        auto frame2 = std::make_shared<dai::ImgFrame>();
        cv::Mat mat2(2, 2, CV_8UC1, cv::Scalar(2));
        frame2->setCvFrame(mat2, dai::ImgFrame::Type::GRAY8);

        auto messageGroup = std::make_shared<dai::MessageGroup>();
        messageGroup->add("frame1", frame1);
        messageGroup->add("frame2", frame2);

        std::vector<std::uint8_t> serialized = dai::StreamMessageParser::serializeMetadata(messageGroup);

        dai::StreamPacketDesc packetMessageGroup = getRawBuffer(messageGroup);
        dai::StreamPacketDesc packetFrame1 = getRawBuffer(frame1);
        dai::StreamPacketDesc packetFrame2 = getRawBuffer(frame2);

        trompeloeil::sequence seq;
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetMessageGroup));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetFrame1));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetFrame2));

        auto result = xlinkIn.readData();
        REQUIRE(result != nullptr);
        auto messasgeGroupGot = std::dynamic_pointer_cast<dai::MessageGroup>(result);
        REQUIRE(messasgeGroupGot != nullptr);
        auto imgGot1 = messasgeGroupGot->get<dai::ImgFrame>("frame1");
        auto imgGot2 = messasgeGroupGot->get<dai::ImgFrame>("frame2");
        REQUIRE(imgGot1 != nullptr);
        REQUIRE(imgGot2 != nullptr);
        auto cvFrame1 = imgGot1->getCvFrame();
        auto cvFrame2 = imgGot2->getCvFrame();

        REQUIRE(cvFrame1.rows == mat1.rows);
        REQUIRE(cvFrame1.cols == mat1.cols);

        REQUIRE(cvFrame2.rows == mat2.rows);
        REQUIRE(cvFrame2.cols == mat2.cols);

        REQUIRE(cv::countNonZero(cvFrame1 != mat1) == 0);
        REQUIRE(cv::countNonZero(cvFrame2 != mat2) == 0);
    }

    SECTION("PacketizedFrame") {
        auto frame = std::make_shared<dai::ImgFrame>();
        cv::Mat mat(2, 2, CV_8UC1, cv::Scalar(1));
        frame->setCvFrame(mat, dai::ImgFrame::Type::GRAY8);

        // 1. Prepare the chunks
        auto packetVec = getPackets(frame, 100);
        auto fullPacket = getRawBuffer(frame);

        // 2. Prepare the header that tells readData how many chunks to expect
        auto packetizedData = std::make_shared<dai::PacketizedData>(packetVec.size(), fullPacket.length);
        dai::StreamPacketDesc headerPacket = getRawBuffer(packetizedData);

        trompeloeil::sequence seq;
        std::vector<std::unique_ptr<trompeloeil::expectation>> expectations;
        REQUIRE(packetVec.size() == 4);

        // EXPECTATION 1: The Header
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(headerPacket));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[0]));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[1]));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[2]));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[3]));

        // 3. Act
        auto result = xlinkIn.readData();

        // 4. Assert
        REQUIRE(result != nullptr);
        // Note: readData should return the RECONSTRUCTED ImgFrame, not the PacketizedData descriptor
        auto imgGot = std::dynamic_pointer_cast<dai::ImgFrame>(result);
        REQUIRE(imgGot->getType() == dai::ImgFrame::Type::GRAY8);
        auto cvFrame = imgGot->getCvFrame();
        REQUIRE(cvFrame.rows == mat.rows);
        REQUIRE(cvFrame.cols == mat.cols);
        REQUIRE(cv::countNonZero(cvFrame != mat) == 0);
    }

    SECTION("PacketizedFrame Failed") {
        auto frame = std::make_shared<dai::ImgFrame>();
        cv::Mat mat(2, 2, CV_8UC1, cv::Scalar(1));
        frame->setCvFrame(mat, dai::ImgFrame::Type::GRAY8);

        // 1. Prepare the chunks
        auto packetVec = getPackets(frame, 100);
        auto fullPacket = getRawBuffer(frame);

        // 2. Prepare the header that tells readData how many chunks to expect
        auto packetizedData = std::make_shared<dai::PacketizedData>(packetVec.size(), fullPacket.length + 200);
        dai::StreamPacketDesc headerPacket = getRawBuffer(packetizedData);

        trompeloeil::sequence seq;
        std::vector<std::unique_ptr<trompeloeil::expectation>> expectations;
        REQUIRE(packetVec.size() == 4);

        // EXPECTATION 1: The Header
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(headerPacket));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[0]));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[1]));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[2]));
        REQUIRE_CALL(xlinkIn, readStreamMessage()).IN_SEQUENCE(seq).LR_RETURN(std::move(packetVec[3]));

        // 3. Act
        REQUIRE_THROWS_AS(xlinkIn.readData(), std::runtime_error);
    }
}
