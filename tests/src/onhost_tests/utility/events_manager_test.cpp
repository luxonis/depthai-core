#include <catch2/catch_all.hpp>
#include <memory>
#include <stdexcept>

#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/EventsManager.hpp"

using namespace dai;
using namespace dai::utility;

TEST_CASE("FileGroup throws on null pointer inputs", "[FileGroup][EventsManager]") {
    FileGroup fileGroup;

    SECTION("addFile with null ImgFrame throws") {
        std::shared_ptr<ImgFrame> nullFrame = nullptr;
        REQUIRE_THROWS_AS(fileGroup.addFile("test.jpg", nullFrame), std::invalid_argument);
    }

    SECTION("addFile with null EncodedFrame throws") {
        std::shared_ptr<EncodedFrame> nullFrame = nullptr;
        REQUIRE_THROWS_AS(fileGroup.addFile("test.jpg", nullFrame), std::invalid_argument);
    }

    SECTION("addFile with null ImgDetections throws") {
        std::shared_ptr<ImgDetections> nullDetections = nullptr;
        REQUIRE_THROWS_AS(fileGroup.addFile("test.json", nullDetections), std::invalid_argument);
    }

    SECTION("addImageDetectionsPair with null ImgFrame throws") {
        std::shared_ptr<ImgFrame> nullFrame = nullptr;
        auto detections = std::make_shared<ImgDetections>();
        REQUIRE_THROWS_AS(fileGroup.addImageDetectionsPair("test", nullFrame, detections), std::invalid_argument);
    }

    SECTION("addImageDetectionsPair with null EncodedFrame throws") {
        std::shared_ptr<EncodedFrame> nullFrame = nullptr;
        auto detections = std::make_shared<ImgDetections>();
        REQUIRE_THROWS_AS(fileGroup.addImageDetectionsPair("test", nullFrame, detections), std::invalid_argument);
    }

    SECTION("addImageDetectionsPair with null ImgDetections throws") {
        auto frame = std::make_shared<ImgFrame>();
        frame->setType(ImgFrame::Type::BGR888i).setSize(4, 4);
        std::vector<uint8_t> data(4 * 4 * 3, 128);
        frame->setData(data);
        std::shared_ptr<ImgDetections> nullDetections = nullptr;
        REQUIRE_THROWS_AS(fileGroup.addImageDetectionsPair("test", frame, nullDetections), std::invalid_argument);
    }
}

TEST_CASE("FileGroup accepts valid inputs", "[FileGroup][EventsManager]") {
    FileGroup fileGroup;

    SECTION("addFile with valid ImgFrame works") {
        auto frame = std::make_shared<ImgFrame>();
        frame->setType(ImgFrame::Type::BGR888i).setSize(4, 4);
        std::vector<uint8_t> data(4 * 4 * 3, 128);  // Gray image
        frame->setData(data);
        REQUIRE_NOTHROW(fileGroup.addFile("test.jpg", frame));
    }

    SECTION("addFile with valid ImgDetections works") {
        auto detections = std::make_shared<ImgDetections>();
        REQUIRE_NOTHROW(fileGroup.addFile("test.json", detections));
    }

    SECTION("addImageDetectionsPair with valid inputs works") {
        auto frame = std::make_shared<ImgFrame>();
        frame->setType(ImgFrame::Type::BGR888i).setSize(4, 4);
        std::vector<uint8_t> data(4 * 4 * 3, 128);
        frame->setData(data);
        auto detections = std::make_shared<ImgDetections>();
        REQUIRE_NOTHROW(fileGroup.addImageDetectionsPair("test", frame, detections));
    }

    SECTION("addFile with string data works") {
        REQUIRE_NOTHROW(fileGroup.addFile("test.txt", "hello world", "text/plain"));
    }
}
