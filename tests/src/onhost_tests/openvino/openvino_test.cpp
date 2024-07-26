#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <depthai/openvino/OpenVINO.hpp>
#include <fstream>
#include <stdexcept>

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

TEST_CASE("Superblob throws if invalid data is passed in") {
    // Constructor does not throw if valid data is passed in
    std::ifstream file(SUPERBLOB_PATH, std::ios::binary);
    std::vector<uint8_t> myRandomData = std::vector<uint8_t>(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    REQUIRE_NOTHROW(dai::OpenVINO::SuperBlob(myRandomData));

    // Constructor throws if header it too small
    std::vector<uint8_t> myRandomData2 = {0, 1, 2, 3, 4, 5, 6, 7};
    REQUIRE_THROWS_AS(dai::OpenVINO::SuperBlob(myRandomData2), std::invalid_argument);

    // Constructor throws if header is of correct size, but the header data is invalid
    std::vector<uint8_t> myRandomData3 = myRandomData;
    myRandomData3[0] = 1;
    REQUIRE_THROWS_AS(dai::OpenVINO::SuperBlob(myRandomData3), std::invalid_argument);
}