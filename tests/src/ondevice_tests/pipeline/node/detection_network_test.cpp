#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("DetectionNetwork can load BLOB properly") {
    const std::filesystem::path archivePath = BLOB_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // No classes at the beginning
    REQUIRE(!nn->getClasses().has_value());

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    SECTION("setNNArchive") {
        REQUIRE_NOTHROW(nn->setNNArchive(nnArchive));
    }
    SECTION("setModelPath") {
        REQUIRE_NOTHROW(nn->setModelPath(archivePath));
    }

    // Network loads classes
    REQUIRE(nn->getClasses().has_value());

    // Classes match
    auto loadedClasses = nn->getClasses().value();
    auto archiveClasses = nnArchive.getConfig<dai::nn_archive::v1::Config>().model.heads->at(0).metadata.classes.value();
    REQUIRE(loadedClasses == archiveClasses);

    // Throws if number of shaves is specified
    REQUIRE_THROWS(nn->setNNArchive(nnArchive, 6));
}

TEST_CASE("DetectionNetwork can load SUPERBLOB properly") {
    const std::filesystem::path archivePath = SUPERBLOB_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    SECTION("setNNArchive") {
        REQUIRE_NOTHROW(nn->setNNArchive(nnArchive));
        // Does not throw if number of shaves is specified
        REQUIRE_NOTHROW(nn->setNNArchive(nnArchive, 6));
    }
    SECTION("setModelPath") {
        REQUIRE_NOTHROW(nn->setModelPath(archivePath));
    }
}

TEST_CASE("DetectionNetwork throws when passed the OTHER NNArchive type") {
    const std::filesystem::path archivePath = ONNX_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);

    SECTION("setNNArchive") {
        REQUIRE_NOTHROW(nn->setNNArchive(nnArchive));
        REQUIRE_THROWS(nn->setNNArchive(nnArchive, 6));
    }
    SECTION("setModelPath") {
        REQUIRE_NOTHROW(nn->setModelPath(archivePath));
    }
}

TEST_CASE("DetectionNetwork sets device for all subnodes") {
    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();
    auto pipelineDevice = p.getDefaultDevice();

    // Check that device is set for all subnodes
    REQUIRE(nn->neuralNetwork->getDevice() == pipelineDevice);
    REQUIRE(nn->detectionParser->getDevice() == pipelineDevice);
}
