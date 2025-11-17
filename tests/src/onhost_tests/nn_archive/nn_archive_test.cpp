#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/nn_archive/NNArchive.hpp>
#include <filesystem>
#include <memory>
#include <string>

TEST_CASE("NNArchive loads a BLOB properly") {
    dai::NNArchive nnArchive(BLOB_ARCHIVE_PATH);

    // Loaded archive is BLOB
    REQUIRE(nnArchive.getModelType() == dai::model::ModelType::BLOB);

    // Returns blob
    REQUIRE(nnArchive.getBlob().has_value());
    REQUIRE(!nnArchive.getSuperBlob().has_value());
    REQUIRE(!nnArchive.getOtherModelFormat().has_value());
}

TEST_CASE("NNArchive loads a SUPERBLOB properly") {
    dai::NNArchive nnArchive(SUPERBLOB_ARCHIVE_PATH);

    // Loaded archive is SUPERBLOB
    REQUIRE(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB);

    // Returns superblob
    REQUIRE(nnArchive.getSuperBlob().has_value());
    REQUIRE(!nnArchive.getBlob().has_value());
    REQUIRE(!nnArchive.getOtherModelFormat().has_value());

    REQUIRE(nnArchive.getSupportedPlatforms().size() == 1);
    REQUIRE(nnArchive.getSupportedPlatforms()[0] == dai::Platform::RVC2);
}

TEST_CASE("NNArchive loads other formats properly") {
    dai::NNArchive nnArchive(ONNX_ARCHIVE_PATH);

    // Loaded archive is ONNX
    REQUIRE(nnArchive.getModelType() == dai::model::ModelType::OTHER);

    // Returns model
    REQUIRE(nnArchive.getOtherModelFormat().has_value());
    REQUIRE(!nnArchive.getBlob().has_value());
    REQUIRE(!nnArchive.getSuperBlob().has_value());

    REQUIRE(nnArchive.getInputSize().has_value());
    REQUIRE(nnArchive.getInputHeight().has_value());
    REQUIRE(nnArchive.getInputWidth().has_value());
    REQUIRE(nnArchive.getInputSize().value().first == nnArchive.getInputWidth().value());
    REQUIRE(nnArchive.getInputSize().value().second == nnArchive.getInputHeight().value());
    REQUIRE(nnArchive.getInputWidth().value() > 0);
    REQUIRE(nnArchive.getInputHeight().value() > 0);

    REQUIRE_THROWS(nnArchive.getInputHeight(1));
    REQUIRE(nnArchive.getSupportedPlatforms().empty());
}
