#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("Test XLinkIn->XLinkOut passthrough with large frames") {
    dai::Pipeline p;
    auto x_in = p.create<dai::node::XLinkIn>();
    x_in->setStreamName("to_device");
    auto x_out = p.create<dai::node::XLinkOut>();
    x_out->setStreamName("to_host");
    x_in->out.link(x_out->input);
    
    dai::Device device(p);

    auto outQ = d.getOutputQueue("to_host");
    auto inQ = d.getInputQueue("to_device");

    dai::ImgFrame imgFrame;
    
    imgFrame.setSequenceNum(123);
    imgFrame.setWidth(WIDTH);
    imgFrame.setHeight(HEIGHT);

    std::vector<std::uint8_t> data(WIDTH*HEIGHT*3, 0);
    imgFrame.setData(data);
    imgFrame.setType(dai::RawImgFrame::Type::BGR888p);
    imgFrame.setInstanceNum(dai::CameraBoardSocket::RGB);
    // Send the frame
    inQ->send(imgFrame);

    // TODO: add timeout
    auto imgFrame = outQ->get();
    
    REQUIRE(imgFrame->getSequenceNum() == 123)
}