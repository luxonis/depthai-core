#include <catch2/catch_all.hpp>
#include <depthai/openvino/OpenVINO.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("DetectionNetwork can load BLOB properly") {
    const std::string archivePath = BLOB_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    REQUIRE_NOTHROW(nn->setNNArchive(nnArchive));

    // Throws if number of shaves is specified
    REQUIRE_THROWS(nn->setNNArchive(nnArchive, 6));
}

TEST_CASE("DetectionNetwork can load SUPERBLOB properly") {
    const std::string archivePath = SUPERBLOB_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    REQUIRE_NOTHROW(nn->setNNArchive(nnArchive));

    // Does not throw if number of shaves is specified
    REQUIRE_NOTHROW(nn->setNNArchive(nnArchive, 6));
}

TEST_CASE("DetectionNetwork throws when passed the OTHER NNArchive type") {
    const std::string archivePath = ONNX_ARCHIVE_PATH;

    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::DetectionNetwork>();

    // Load NNArchive
    dai::NNArchive nnArchive(archivePath);
    REQUIRE_THROWS(nn->setNNArchive(nnArchive));
    REQUIRE_THROWS(nn->setNNArchive(nnArchive, 6));
}
