#include <catch2/catch_test_macros.hpp>
#include <cstdlib>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/Odometry.hpp"

TEST_CASE("Odometry frame metadata survives device Script transport", "[odometry][serialization][device]") {
    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();
    script->setScript("while True:\n    node.io['out'].send(node.io['in'].get())\n");
    auto input = script->inputs["in"].createInputQueue();
    auto output = script->outputs["out"].createOutputQueue();
    pipeline.start();
    for(bool withMetadata : {false, true}) {
        auto message = std::make_shared<dai::Odometry>(1, 2, 3, 0, 0, 0, 1, dai::Point3d{4, 5, 6});
        message->setSequenceNum(42);
        if(withMetadata) {
            message->trackedFrameExtrinsics =
                dai::Extrinsics({{0, -1, 0}, {0, 0, -1}, {1, 0, 0}}, {10, 20, 30}, dai::CameraBoardSocket::CAM_B, dai::LengthUnit::CENTIMETER);
        }
        input->send(message);
        bool timedOut = false;
        auto received = output->get<dai::Odometry>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(received);
        REQUIRE(received->getSequenceNum() == 42);
        REQUIRE(received->getTranslation().x == 1);
        REQUIRE(received->velocity.z == 6);
        // Explicit compatibility mode tests older firmware, which can parse the
        // legacy prefix but may drop the extension when it reserializes a message.
        const bool legacyFirmware = std::getenv("DEPTHAI_TEST_LEGACY_ODOMETRY_FIRMWARE") != nullptr;
        if(withMetadata && !legacyFirmware) {
            REQUIRE(received->trackedFrameExtrinsics);
            REQUIRE(received->trackedFrameExtrinsics->isEqualExtrinsics(*message->trackedFrameExtrinsics));
        } else if(!withMetadata) {
            REQUIRE_FALSE(received->trackedFrameExtrinsics);
        } else {
            INFO("Legacy firmware preserved the optional extension: " << bool(received->trackedFrameExtrinsics));
            if(received->trackedFrameExtrinsics) REQUIRE(received->trackedFrameExtrinsics->isEqualExtrinsics(*message->trackedFrameExtrinsics));
        }
    }
    pipeline.stop();
}
