#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

#include "depthai/pipeline/datatype/ADatatype.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

static bool operator==(const dai::span<uint8_t>& lhs, const std::vector<unsigned char>& rhs) {
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

auto TIMEOUT = 5s;

static void test_xlink_roundtrip(int w, int h) {
    std::vector<std::uint8_t> data(w * h * 3, 0xFF);

    dai::Pipeline p;
    auto x_in = p.create<dai::node::XLinkIn>();
    x_in->setStreamName("to_device");
    x_in->setMaxDataSize(data.size());
    auto x_out = p.create<dai::node::XLinkOut>();
    x_out->setStreamName("to_host");
    x_in->out.link(x_out->input);

    dai::Device device(p);

    auto outQ = device.getOutputQueue("to_host");
    auto inQ = device.getInputQueue("to_device");

    auto imgFrame = std::make_shared<dai::ImgFrame>();

    imgFrame->setSequenceNum(123);
    imgFrame->setWidth(w);
    imgFrame->setHeight(h);

    imgFrame->setData(data);
    imgFrame->setType(dai::ImgFrame::Type::BGR888p);
    // Send the frame
    inQ->send(imgFrame);

    auto t1 = steady_clock::now();
    bool success = false;
    do {
        auto retFrm = outQ->tryGet<dai::ImgFrame>();
        if(retFrm) {
            REQUIRE(imgFrame->getSequenceNum() == 123);
            REQUIRE(imgFrame->getWidth() == w);
            REQUIRE(imgFrame->getHeight() == h);
            REQUIRE((imgFrame->getData() == data));
            return;
        }
    } while(steady_clock::now() - t1 < TIMEOUT);
    // Timeout
    FAIL("Timeout receiving back the sent message");
}

template <class T, typename = std::enable_if_t<std::is_base_of<dai::Buffer, T>::value>>
static void test_xlink_message_type(T) {
    dai::Pipeline p;
    auto x_in = p.create<dai::node::XLinkIn>();
    x_in->setStreamName("to_device");
    auto x_out = p.create<dai::node::XLinkOut>();
    x_out->setStreamName("to_host");
    x_in->out.link(x_out->input);

    dai::Device device(p);

    auto outQ = device.getOutputQueue("to_host");
    auto inQ = device.getInputQueue("to_device");
    auto message = std::make_shared<T>();
    auto time = steady_clock::now();
    message->setSequenceNum(123);
    message->setTimestamp(time);
    message->setData({1, 2, 3, 4, 5});
    message->setTimestampDevice(time);
    // Send the frame
    inQ->send(message);

    auto t1 = steady_clock::now();
    bool success = false;
    do {
        auto retFrm = outQ->tryGet<dai::Buffer>();
        if(retFrm) {
            REQUIRE(retFrm->getSequenceNum() == 123);
            REQUIRE(retFrm->getTimestamp() == time);
            REQUIRE((retFrm->getData() == std::vector<unsigned char>{1, 2, 3, 4, 5}));
            REQUIRE(retFrm->getTimestampDevice() == time);

            // For a sanity check, check also that the original message returns the same values
            REQUIRE(message->getSequenceNum() == 123);
            REQUIRE(message->getTimestamp() == time);
            REQUIRE((message->getData() == std::vector<unsigned char>{1, 2, 3, 4, 5}));
            REQUIRE(message->getTimestampDevice() == time);
            return;
        }
    } while(steady_clock::now() - t1 < TIMEOUT);
    // Timeout
    FAIL("Timeout receiving back the sent message");
}

TEST_CASE("Test XLinkIn->XLinkOut passthrough with random 1000x1000 frame") {
    test_xlink_roundtrip(1000, 1000);
}

TEST_CASE("Test XLinkIn->XLinkOut passthrough with random 2000x1000 frame") {
    test_xlink_roundtrip(2000, 1000);
}

TEST_CASE("Test XLinkIn->XLinkOut passthrough with random 4000x3000 frame") {
    test_xlink_roundtrip(4000, 3000);
}

TEST_CASE("Buffer message type") {
    test_xlink_message_type(dai::Buffer());
}

TEST_CASE("ImgFrame message type") {
    test_xlink_message_type(dai::ImgFrame());
}

TEST_CASE("EncodedFrame message type") {
    test_xlink_message_type(dai::EncodedFrame());
}

TEST_CASE("NNData message type") {
    test_xlink_message_type(dai::NNData());
}
