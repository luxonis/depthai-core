#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/nn_archive/NNArchive.hpp>
#include <filesystem>
#include <memory>
#include <string>

namespace {
class TestHelper {
   public:
    TestHelper() {
        srand(time(nullptr));
        extractFolder = std::filesystem::path("/tmp/depthai_test_" + std::to_string(rand())).string();
    }

    ~TestHelper() {
        std::filesystem::remove_all(extractFolder);
    }

    std::string extractFolder;
};
}  // namespace

TEST_CASE("NNArchive loads a BLOB properly") {
    dai::NNArchive nnArchive(BLOB_ARCHIVE_PATH);

    auto helper = std::make_unique<TestHelper>();
    std::string extractFolder = helper->extractFolder;

    // Loaded archive is BLOB
    REQUIRE(nnArchive.getModelType() == dai::model::ModelType::BLOB);

    // Returns blob
    REQUIRE(nnArchive.getBlob().has_value());

    // Returns path to blob
    REQUIRE(nnArchive.getModelPath().has_value());
    std::filesystem::path path = nnArchive.getModelPath().value();
    REQUIRE(std::filesystem::exists(path));
    REQUIRE(path.extension() == ".blob");

    // Returns nothing for other types
    REQUIRE(!nnArchive.getSuperBlob().has_value());
}

TEST_CASE("NNArchive loads a SUPERBLOB properly") {
    dai::NNArchive nnArchive(SUPERBLOB_ARCHIVE_PATH);

    auto helper = std::make_unique<TestHelper>();
    std::string extractFolder = helper->extractFolder;

    // Loaded archive is SUPERBLOB
    REQUIRE(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB);

    // Returns superblob
    REQUIRE(nnArchive.getSuperBlob().has_value());

    // Returns path to superblob
    REQUIRE(nnArchive.getModelPath().has_value());
    std::filesystem::path path = nnArchive.getModelPath().value();
    REQUIRE(std::filesystem::exists(path));
    REQUIRE(path.extension() == ".superblob");

    // Returns nothing for other types
    REQUIRE(!nnArchive.getBlob().has_value());
    REQUIRE(nnArchive.getSupportedPlatforms().size() == 1);
    REQUIRE(nnArchive.getSupportedPlatforms()[0] == dai::Platform::RVC2);
}

TEST_CASE("NNArchive loads other formats properly") {
    // Used for initialization and cleanup (even if the test fails)
    auto helper = std::make_unique<TestHelper>();
    std::string extractFolder = helper->extractFolder;

    dai::NNArchive nnArchive(ONNX_ARCHIVE_PATH, dai::NNArchiveOptions().extractFolder(extractFolder));

    // Loaded archive is ONNX
    REQUIRE(nnArchive.getModelType() == dai::model::ModelType::OTHER);

    // Returns model path
    REQUIRE(nnArchive.getModelPath().has_value());

    // Check that the returned path exists
    std::filesystem::path modelPath = nnArchive.getModelPath().value();
    REQUIRE(std::filesystem::exists(modelPath));

    // Check that it has the .onnx extension
    REQUIRE(modelPath.extension() == ".onnx");

    // Returns nothing for other types
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
