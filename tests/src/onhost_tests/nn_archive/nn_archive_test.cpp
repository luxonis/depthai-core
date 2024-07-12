#include <catch2/catch_all.hpp>
#include <depthai/nn_archive/NNArchive.hpp>

TEST_CASE("NNArchive loads a BLOB properly") {
    dai::NNArchive nnArchive(BLOB_PATH);

    // Loaded archive is BLOB
    REQUIRE(nnArchive.getArchiveType() == dai::NNArchiveType::BLOB);

    // Returns blob
    REQUIRE(nnArchive.getBlob().has_value());

    // Returns nothing for other types
    REQUIRE(!nnArchive.getSuperBlob().has_value());
    REQUIRE(!nnArchive.getModelPath().has_value());
}

TEST_CASE("NNArchive loads a SUPERBLOB properly") {
    dai::NNArchive nnArchive(SUPERBLOB_PATH);

    // Loaded archive is SUPERBLOB
    REQUIRE(nnArchive.getArchiveType() == dai::NNArchiveType::SUPERBLOB);

    // Returns superblob
    REQUIRE(nnArchive.getSuperBlob().has_value());

    // Returns nothing for other types
    REQUIRE(!nnArchive.getBlob().has_value());
    REQUIRE(!nnArchive.getModelPath().has_value());
}

TEST_CASE("NNArchive loads other formats properly") {
    // TODO: (lnotspot) Implement this once there's a model in artifactory
}
