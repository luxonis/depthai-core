#include <catch2/catch_all.hpp>
#include <depthai/openvino/OpenVINO.hpp>

TEST_CASE("Superblob can generate blobs with different number of shaves") {
    dai::OpenVINO::SuperBlob superblob(SUPERBLOB_PATH);

    // Generate blobs for all possible number of shaves
    for(size_t numShaves = 1; numShaves <= dai::OpenVINO::SuperBlob::NUMBER_OF_PATCHES; ++numShaves) {
        dai::OpenVINO::Blob blob = superblob.getBlobWithNumShaves(static_cast<int>(numShaves));
        REQUIRE(blob.numShaves == numShaves);
    }
}

TEST_CASE("Superblob throws when invalid number of shaves are requested") {
    dai::OpenVINO::SuperBlob superblob(SUPERBLOB_PATH);
    REQUIRE_THROWS(superblob.getBlobWithNumShaves(-1));
    REQUIRE_THROWS(superblob.getBlobWithNumShaves(dai::OpenVINO::SuperBlob::NUMBER_OF_PATCHES + 1));
}

TEST_CASE("Superblob throws when file does not exist") {
    REQUIRE_THROWS(dai::OpenVINO::SuperBlob("totallyRandomSuperblobName.superblob"));
}