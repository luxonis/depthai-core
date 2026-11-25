#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

// A control whose regions are mapped from a frame (setAutoFocusRegion / setAutoExposureRegion with a Rect and the frame's
// ImgTransformation) keeps its regions and commands through a device roundtrip.
TEST_CASE("CameraControl regions mapped from a frame survive a device roundtrip", "[CameraControl][Region]") {
    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();
    script->setScript("while True:\n    node.io['out'].send(node.io['in'].get())\n");
    auto input = script->inputs["in"].createInputQueue();
    auto output = script->outputs["out"].createOutputQueue();
    pipeline.start();

    // The IMX586's 1920x1080 mode on RVC4: 7680x4320 at (160, 840) of the 8000x6000 sensor, scaled by 1/4.
    dai::ImgTransformation transformation(8000, 6000);
    transformation.addCrop(160, 840, 7680, 4320);
    transformation.addScale(0.25f, 0.25f);
    auto control = std::make_shared<dai::CameraControl>();
    control->setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);
    control->setAutoFocusRegion(dai::Rect(100, 50, 40, 20, false), transformation);
    control->setAutoExposureRegion(dai::Rect(0.5f, 0.5f, 0.25f, 0.25f, true), transformation);
    control->setAutoFocusTrigger();
    input->send(control);

    bool timedOut = false;
    const auto received = output->get<dai::CameraControl>(std::chrono::seconds(5), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(received != nullptr);
    REQUIRE(received->getDatatype() == dai::DatatypeEnum::CameraControl);
    REQUIRE(received->afRegion.x == 560);
    REQUIRE(received->afRegion.y == 1040);
    REQUIRE(received->afRegion.width == 160);
    REQUIRE(received->afRegion.height == 80);
    REQUIRE(received->afRegion.priority == 1);
    REQUIRE(received->aeRegion.x == 4000);
    REQUIRE(received->aeRegion.y == 3000);
    REQUIRE(received->aeRegion.width == 1920);
    REQUIRE(received->aeRegion.height == 1080);
    REQUIRE(received->aeRegion.priority == 1);
    REQUIRE(received->getCommand(dai::CameraControl::Command::AF_MODE));
    REQUIRE(received->getCommand(dai::CameraControl::Command::AF_REGION));
    REQUIRE(received->getCommand(dai::CameraControl::Command::AE_REGION));
    REQUIRE(received->getCommand(dai::CameraControl::Command::AF_TRIGGER));
    REQUIRE(received->cmdMask == control->cmdMask);
    REQUIRE(received->getSequenceNum() == control->getSequenceNum());
    // RVC2 fills missing transport timestamps. Compare the rest of the serialized control.
    received->setTimestamp(control->getTimestamp());
    received->setTimestampDevice(control->getTimestampDevice());
    received->setTimestampSystem(control->getTimestampSystem());
    REQUIRE(dai::utility::serialize(*received) == dai::utility::serialize(*control));
}
