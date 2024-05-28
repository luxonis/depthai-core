#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>

#include "depthai-shared/common/CameraBoardSocket.hpp"

TEST_CASE("Set and get messages") {
    auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    std::vector<unsigned char> buf1Data = {1, 2, 3};
    std::vector<unsigned char> buf2Data = {4, 5, 6};
    dai::MessageGroup msgGrp;
    dai::Buffer buf;
    buf.setTimestamp(buf1Ts);
    buf.setData(buf1Data);
    msgGrp.add("buf1", buf);

    dai::ImgFrame img;
    img.setTimestamp(buf2Ts);
    img.setData(buf2Data);
    img.setSize({5, 6});
    msgGrp.add("img1", img);

    REQUIRE(msgGrp.get<dai::Buffer>("buf1")->getTimestamp() == buf1Ts);
    REQUIRE(msgGrp.get<dai::ImgFrame>("img1")->getTimestamp() == buf2Ts);
    REQUIRE(msgGrp.get<dai::Buffer>("buf1")->getData() == buf1Data);
    REQUIRE(msgGrp.get<dai::ImgFrame>("img1")->getData() == buf2Data);
    REQUIRE(msgGrp.get<dai::ImgFrame>("img1")->getWidth() == 5);
    REQUIRE(msgGrp.get<dai::ImgFrame>("img1")->getHeight() == 6);
}

TEST_CASE("Send large messages") {
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();

    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_12_MP);

    camRgb->setFps(20);

    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    left->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    left->setFps(20);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    right->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    right->setFps(20);

    auto sync = pipeline.create<dai::node::Sync>();
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("out");

    sync->out.link(xout->input);
    camRgb->isp.link(sync->inputs["rgb"]);
    left->out.link(sync->inputs["left"]);
    right->out.link(sync->inputs["right"]);

    dai::Device device(pipeline);
    auto q = device.getOutputQueue("out", 8, true);

    bool hasTimedOut = false;
    auto msg = q->get(std::chrono::seconds(1), hasTimedOut);
    REQUIRE(!hasTimedOut);
}

// TODO(asahtik): Bring back when the [issue](https://github.com/luxonis/depthai-core/issues/929) is fixed
// TEST_CASE("Sync - demux") {
//     auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
//     auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);
//
//     dai::Pipeline pipeline;
//     auto xout1 = pipeline.create<dai::node::XLinkOut>();
//     xout1->setStreamName("out1");
//     auto xout2 = pipeline.create<dai::node::XLinkOut>();
//     xout2->setStreamName("out2");
//
//     auto demux = pipeline.create<dai::node::MessageDemux>();
//
//     auto sync = pipeline.create<dai::node::Sync>();
//     sync->setSyncThreshold(std::chrono::milliseconds(100));
//
//     auto xin1 = pipeline.create<dai::node::XLinkIn>();
//     xin1->setStreamName("in1");
//     auto xin2 = pipeline.create<dai::node::XLinkIn>();
//     xin2->setStreamName("in2");
//
//     xin1->out.link(sync->inputs["buf1"]);
//     xin2->out.link(sync->inputs["buf2"]);
//     sync->out.link(demux->input);
//     demux->outputs["buf1"].link(xout1->input);
//     demux->outputs["buf2"].link(xout2->input);
//
//     dai::Device device(pipeline);
//
//     auto inQ1 = device.getInputQueue("in1");
//     auto inQ2 = device.getInputQueue("in2");
//     auto outQ1 = device.getOutputQueue("out1");
//     auto outQ2 = device.getOutputQueue("out2");
//
//     dai::Buffer buf1;
//     buf1.setData({1, 2, 3, 4, 5});
//     buf1.setTimestamp(buf1Ts);
//
//     dai::ImgFrame img1;
//     img1.setData({6, 7, 8, 9, 10});
//     img1.setTimestamp(buf2Ts);
//     img1.setSize({5, 6});
//
//     inQ1->send(buf1);
//     inQ2->send(img1);
//
//     auto out1 = outQ1->get<dai::Buffer>();
//     auto out2 = outQ2->get<dai::ImgFrame>();
//
//     REQUIRE(out1->getTimestamp() == buf1Ts);
//     REQUIRE(out2->getTimestamp() == buf2Ts);
//     REQUIRE(out1->getData() == std::vector<unsigned char>{1, 2, 3, 4, 5});
//     REQUIRE(out2->getData() == std::vector<unsigned char>{6, 7, 8, 9, 10});
//     REQUIRE(out2->getWidth() == 5);
//     REQUIRE(out2->getHeight() == 6);
// }

TEST_CASE("MessageGroup ping-pong") {
    auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

    dai::Pipeline pipeline;
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("out");

    auto xin = pipeline.create<dai::node::XLinkIn>();
    xin->setStreamName("in");

    xin->out.link(xout->input);

    dai::Device device(pipeline);

    auto inQ = device.getInputQueue("in");
    auto outQ = device.getOutputQueue("out");

    dai::Buffer buf1;
    buf1.setData({1, 2, 3, 4, 5});
    buf1.setTimestamp(buf1Ts);

    dai::ImgFrame img1;
    img1.setData({6, 7, 8, 9, 10});
    img1.setTimestamp(buf2Ts);
    img1.setSize({5, 6});

    dai::MessageGroup msgGrp;
    msgGrp.add("buf1", buf1);
    msgGrp.add("img1", img1);

    inQ->send(msgGrp);

    auto out = outQ->get<dai::MessageGroup>();

    // REQUIRE(out->get<dai::Buffer>("buf1")->getTimestamp() == buf1Ts);
    // REQUIRE(out->get<dai::ImgFrame>("img1")->getTimestamp() == buf2Ts);
    REQUIRE(out->get<dai::Buffer>("buf1")->getData() == std::vector<unsigned char>{1, 2, 3, 4, 5});
    REQUIRE(out->get<dai::ImgFrame>("img1")->getData() == std::vector<unsigned char>{6, 7, 8, 9, 10});
    REQUIRE(out->get<dai::ImgFrame>("img1")->getWidth() == 5);
    REQUIRE(out->get<dai::ImgFrame>("img1")->getHeight() == 6);
}
