#include <catch2/catch_all.hpp>

// std
#include <atomic>
#include <fstream>
#include <iostream>

// Include depthai library
#include <depthai/depthai.hpp>

// setBlob

void checkBlob(dai::OpenVINO::Blob& blob) {
    printf("Blob: ");
    for(const auto& in : blob.networkInputs) {
        std::string name = in.first;
        auto tensor = in.second;
    }
    for(const auto& out : blob.networkOutputs) {
        std::string name = out.first;
        auto tensor = out.second;
    }

    REQUIRE(blob.networkInputs.size() == 1);
    REQUIRE(blob.networkInputs.at("0").numDimensions == 4);
    REQUIRE(blob.networkInputs.at("0").order == dai::TensorInfo::StorageOrder::NCHW);

    REQUIRE(blob.networkOutputs.size() == 1);
    REQUIRE(blob.networkOutputs.at("14").numDimensions == 4);
    REQUIRE(blob.networkOutputs.at("14").order == dai::TensorInfo::StorageOrder::NCHW);
    REQUIRE(blob.networkOutputs.at("14").dataType == dai::TensorInfo::DataType::FP16);
}

// Check if an exception is thrown if blob is corrupted
TEST_CASE("OpenVINO corrupted blob") {
    std::ifstream stream(OPENVINO_2021_4_BLOB_PATH, std::ios::in | std::ios::binary);
    std::vector<std::uint8_t> blobData(std::istreambuf_iterator<char>(stream), {});

    // Corrupt blob by removing half the file size
    blobData.resize(blobData.size() / 2);

    REQUIRE_THROWS(dai::OpenVINO::Blob(blobData));
}
