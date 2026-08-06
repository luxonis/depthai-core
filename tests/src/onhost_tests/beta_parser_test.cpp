#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "../../src/utility/Platform.hpp"
#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/utility/Compression.hpp"

namespace {

using Json = nlohmann::json;
namespace fs = std::filesystem;

constexpr std::chrono::seconds OUTPUT_TIMEOUT{6};

struct Fixture {
    fs::path directory;
    Json json;
    std::shared_ptr<dai::NNData> input;
};

class TestDataHelper {
   public:
    explicit TestDataHelper(const fs::path& archivePath) : extractionDirectory(dai::platform::getTempPath()) {
        try {
            const auto archiveEntries = dai::utility::filenamesInArchive(archivePath);
            std::vector<std::string> filesInArchive;
            std::vector<fs::path> filesOnDisk;
            std::optional<fs::path> manifestPath;

            for(const auto& entry : archiveEntries) {
                const fs::path relativePath = fs::path(entry).lexically_normal();
                if(relativePath.empty() || relativePath.is_absolute() || relativePath.has_root_name()) {
                    throw std::runtime_error("Beta parser fixture archive contains an invalid path: " + entry);
                }
                for(const auto& component : relativePath) {
                    if(component == "..") {
                        throw std::runtime_error("Beta parser fixture archive path escapes its extraction directory: " + entry);
                    }
                }

                if(entry.back() == '/' || entry.back() == '\\') continue;

                const fs::path destination = extractionDirectory / relativePath;
                std::error_code error;
                fs::create_directories(destination.parent_path(), error);
                if(error) {
                    throw std::runtime_error("Could not create beta parser fixture directory: " + destination.parent_path().string());
                }

                filesInArchive.push_back(entry);
                filesOnDisk.push_back(destination);
                if(relativePath.filename() == "manifest.json") {
                    if(manifestPath) {
                        throw std::runtime_error("Beta parser fixture archive contains multiple manifests");
                    }
                    manifestPath = destination;
                }
            }

            if(!manifestPath) {
                throw std::runtime_error("Beta parser fixture archive does not contain manifest.json");
            }

            dai::utility::extractFiles(archivePath, filesInArchive, filesOnDisk);
            if(!fs::is_regular_file(*manifestPath)) {
                throw std::runtime_error("Could not extract the beta parser fixture manifest");
            }
            root = manifestPath->parent_path();
        } catch(...) {
            cleanup();
            throw;
        }
    }

    ~TestDataHelper() {
        cleanup();
    }

    const fs::path& getRoot() const {
        return root;
    }

   private:
    void cleanup() noexcept {
        std::error_code error;
        fs::remove_all(extractionDirectory, error);
    }

    fs::path extractionDirectory;
    fs::path root;
};

const fs::path& fixtureRoot() {
    static const TestDataHelper testData(BETA_PARSER_TEST_DATA_PATH);
    return testData.getRoot();
}

Json readJson(const fs::path& path) {
    std::ifstream stream(path);
    if(!stream) {
        throw std::runtime_error("Could not open JSON fixture: " + path.string());
    }
    auto value = Json::parse(stream, nullptr, false);
    if(value.is_discarded()) {
        throw std::runtime_error("Could not parse JSON fixture: " + path.string());
    }
    return value;
}

std::vector<std::uint8_t> readBytes(const fs::path& path) {
    std::ifstream stream(path, std::ios::binary | std::ios::ate);
    if(!stream) {
        throw std::runtime_error("Could not open fixture payload: " + path.string());
    }
    const auto end = stream.tellg();
    if(end < 0) {
        throw std::runtime_error("Could not determine fixture payload size: " + path.string());
    }
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(end));
    stream.seekg(0);
    if(!bytes.empty() && !stream.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()))) {
        throw std::runtime_error("Could not read fixture payload: " + path.string());
    }
    return bytes;
}

std::vector<float> readFloat32(const fs::path& path) {
    const auto bytes = readBytes(path);
    if(bytes.size() % sizeof(float) != 0) {
        throw std::runtime_error("FP32 fixture payload has an invalid size: " + path.string());
    }

    std::vector<float> values(bytes.size() / sizeof(float));
    for(std::size_t index = 0; index < values.size(); ++index) {
        const std::size_t offset = index * sizeof(float);
        const std::uint32_t bits = static_cast<std::uint32_t>(bytes[offset]) | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U)
                                   | (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U) | (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
        std::memcpy(&values[index], &bits, sizeof(float));
    }
    return values;
}

std::size_t elementCount(const Json& dimensions) {
    std::size_t count = 1;
    for(const auto& dimension : dimensions) {
        const auto value = dimension.get<std::size_t>();
        if(value != 0 && count > std::numeric_limits<std::size_t>::max() / value) {
            throw std::overflow_error("Fixture tensor dimensions overflow size_t");
        }
        count *= value;
    }
    return count;
}

dai::TensorInfo::StorageOrder parseStorageOrder(const std::string& order) {
    if(order == "C") return dai::TensorInfo::StorageOrder::C;
    if(order == "NC") return dai::TensorInfo::StorageOrder::NC;
    if(order == "CHW") return dai::TensorInfo::StorageOrder::CHW;
    if(order == "NCHW") return dai::TensorInfo::StorageOrder::NCHW;
    throw std::runtime_error("Unsupported fixture storage order: " + order);
}

std::shared_ptr<dai::NNData> loadNNData(const fs::path& directory, const Json& tensors) {
    auto nnData = std::make_shared<dai::NNData>();
    nnData->batchSize = 1;

    for(const auto& descriptor : tensors) {
        if(descriptor.at("dtype") != "float32" || descriptor.at("encoding") != "raw-little-endian") {
            throw std::runtime_error("Beta parser input fixture must be raw little-endian FP32");
        }

        auto values = readFloat32(directory / descriptor.at("file").get<std::string>());
        if(values.size() != elementCount(descriptor.at("dimensions"))) {
            throw std::runtime_error("Fixture payload element count does not match its dimensions");
        }

        dai::TensorInfo info;
        info.name = descriptor.at("name").get<std::string>();
        info.dataType = dai::TensorInfo::DataType::FP32;
        info.order = parseStorageOrder(descriptor.at("storage_order").get<std::string>());
        info.numDimensions = descriptor.at("dimensions").size();
        for(const auto& dimension : descriptor.at("dimensions")) {
            info.dims.push_back(dimension.get<unsigned>());
        }
        info.strides.resize(info.dims.size());
        unsigned stride = sizeof(float);
        for(std::size_t index = info.dims.size(); index-- > 0;) {
            info.strides[index] = stride;
            stride *= info.dims[index];
        }
        info.validateStorageOrder();

        auto destination = nnData->emplaceTensor(info);
        if(destination.size() != values.size() * sizeof(float)) {
            throw std::runtime_error("NNData tensor allocation does not match fixture payload");
        }
        std::memcpy(destination.data(), values.data(), destination.size());
    }

    nnData->setSequenceNum(0);
    nnData->setTimestamp(std::chrono::steady_clock::now());
    return nnData;
}

Fixture loadFixture(const std::string& parser, const std::string& caseName) {
    const fs::path directory = fixtureRoot() / parser / caseName;
    auto fixture = readJson(directory / "fixture.json");
    if(fixture.at("format_version") != 1 || fixture.at("parser") != parser || fixture.at("case") != caseName) {
        throw std::runtime_error("Fixture identity or format mismatch: " + (directory / "fixture.json").string());
    }
    return {directory, fixture, loadNNData(directory, fixture.at("tensors"))};
}

fs::path resolveArchive(const Fixture& fixture, const std::string& platform) {
    dai::NNModelDescription description;
    description.model = fixture.json.at("archive").at("model").get<std::string>();
    description.platform = platform;
    return dai::getModelFromZoo(description);
}

template <typename Parser, typename Output, typename Configure>
std::shared_ptr<Output> runParser(const Fixture& fixture, const std::string& platform, Configure&& configure) {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<Parser>();
    const auto archivePath = resolveArchive(fixture, platform);
    INFO("Model platform: " << platform);
    INFO("NNArchive: " << archivePath);
    const dai::NNArchive archive(archivePath);
    parser->setNNArchive(archive);
    std::invoke(std::forward<Configure>(configure), *parser);

    auto inputQueue = parser->input.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();

    bool started = false;
    try {
        pipeline.start();
        started = true;
        inputQueue->send(fixture.input);
        bool timedOut = false;
        auto output = outputQueue->template get<Output>(OUTPUT_TIMEOUT, timedOut);
        pipeline.stop();
        pipeline.wait();
        started = false;
        if(timedOut || output == nullptr) {
            throw std::runtime_error("Timed out waiting for parser output");
        }
        return output;
    } catch(...) {
        if(started) {
            pipeline.stop();
            pipeline.wait();
        }
        throw;
    }
}

template <typename Parser, typename Output>
std::shared_ptr<Output> runParser(const Fixture& fixture, const std::string& platform) {
    return runParser<Parser, Output>(fixture, platform, [](Parser&) {});
}

void checkRelative(float actual, double expected, const std::string& field) {
    INFO(field << ": expected " << expected << ", actual " << actual);
    CHECK(actual == Catch::Approx(expected).epsilon(1e-2));
}

void checkAbsolute(float actual, double expected, double tolerance, const std::string& field) {
    INFO(field << ": expected " << expected << ", actual " << actual);
    CHECK(actual == Catch::Approx(expected).margin(tolerance));
}

std::vector<float> readExpectedFloatArray(const Fixture& fixture, const Json& descriptor) {
    if(descriptor.at("dtype") != "float32" || descriptor.at("encoding") != "raw-little-endian") {
        throw std::runtime_error("Expected array fixture must be raw little-endian FP32");
    }
    auto values = readFloat32(fixture.directory / descriptor.at("file").get<std::string>());
    if(values.size() != elementCount(descriptor.at("shape"))) {
        throw std::runtime_error("Expected array payload element count does not match its shape");
    }
    return values;
}

std::vector<float> readTensorFloat32(const dai::NNData& data, const std::string& name) {
    const auto info = data.getTensorInfo(name);
    if(!info || info->dataType != dai::TensorInfo::DataType::FP32) {
        throw std::runtime_error("Expected an FP32 tensor named " + name);
    }
    std::size_t count = 1;
    for(const auto dimension : info->dims) count *= dimension;
    const auto bytes = data.getData();
    if(info->offset + count * sizeof(float) > bytes.size()) {
        throw std::runtime_error("NNData tensor exceeds its backing buffer");
    }
    std::vector<float> values(count);
    std::memcpy(values.data(), bytes.data() + info->offset, count * sizeof(float));
    return values;
}

void checkKeypoints(const dai::beta::Keypoints& output, const Json& expected) {
    const auto actual = output.getKeypoints();
    const auto& expectedPoints = expected.at("keypoints");
    REQUIRE(actual.size() == expectedPoints.size());
    for(std::size_t index = 0; index < actual.size(); ++index) {
        CAPTURE(index);
        checkRelative(actual[index].imageCoordinates.x, expectedPoints[index][0].get<double>(), "keypoint x");
        checkRelative(actual[index].imageCoordinates.y, expectedPoints[index][1].get<double>(), "keypoint y");
    }
}

void checkDetections(const dai::ImgDetections& output, const Json& expected) {
    const auto& expectedDetections = expected.at("detections");
    REQUIRE(output.detections.size() == expectedDetections.size());
    for(std::size_t index = 0; index < output.detections.size(); ++index) {
        CAPTURE(index);
        const auto& actual = output.detections[index];
        const auto& expectedDetection = expectedDetections[index];
        const auto box = actual.getBoundingBox();
        checkAbsolute(actual.confidence, expectedDetection.at("confidence").get<double>(), 0.01, "confidence");

        const int expectedLabel = expectedDetection.at("label").get<int>();
        if(!(expectedLabel == -1 && actual.label == 0)) CHECK(static_cast<int>(actual.label) == expectedLabel);
        checkAbsolute(box.center.x, expectedDetection.at("x_center").get<double>(), 0.01, "box center x");
        checkAbsolute(box.center.y, expectedDetection.at("y_center").get<double>(), 0.01, "box center y");
        checkAbsolute(box.size.width, expectedDetection.at("width").get<double>(), 0.01, "box width");
        checkAbsolute(box.size.height, expectedDetection.at("height").get<double>(), 0.01, "box height");
        checkAbsolute(box.angle, expectedDetection.at("angle").get<double>(), 0.01, "box angle");

        if(expectedDetection.contains("keypoints")) {
            const auto keypoints = actual.getKeypoints();
            const auto& expectedKeypoints = expectedDetection.at("keypoints");
            REQUIRE(keypoints.size() == expectedKeypoints.size());
            for(std::size_t keypointIndex = 0; keypointIndex < keypoints.size(); ++keypointIndex) {
                CAPTURE(keypointIndex);
                checkAbsolute(keypoints[keypointIndex].imageCoordinates.x, expectedKeypoints[keypointIndex][0].get<double>(), 0.01, "detection keypoint x");
                checkAbsolute(keypoints[keypointIndex].imageCoordinates.y, expectedKeypoints[keypointIndex][1].get<double>(), 0.01, "detection keypoint y");
                checkAbsolute(keypoints[keypointIndex].confidence, expectedKeypoints[keypointIndex][2].get<double>(), 0.01, "detection keypoint confidence");
            }
        }
    }
}

void checkClassificationCase(const std::string& caseName, const std::string& platform, std::optional<bool> softmaxOverride = std::nullopt) {
    const auto fixture = loadFixture("ClassificationParser", caseName);
    const auto output = runParser<dai::beta::node::ClassificationParser, dai::beta::Classifications>(fixture, platform, [&](auto& parser) {
        if(softmaxOverride) parser.setSoftmax(*softmaxOverride);
    });
    const auto& expected = fixture.json.at("expected");
    REQUIRE_FALSE(output->classes.empty());
    REQUIRE_FALSE(output->scores.empty());
    CHECK(output->getTopClass() == expected.at("class").get<std::string>());
    checkRelative(output->getTopScore(), expected.at("score").get<double>(), "top classification score");
}

}  // namespace

TEST_CASE("ClassificationParser matches efficientnet-lite golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    // This recorded tensor is post-softmax; the RVC4 archive describes the logits emitted by its live DLC.
    checkClassificationCase("efficientnet-lite", platform, true);
}

TEST_CASE("ClassificationParser matches emotion-recognition golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    checkClassificationCase("emotion-recognition", platform);
}

TEST_CASE("ClassificationSequenceParser matches paddle-text-recognition golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("ClassificationSequenceParser", "paddle-text-recognition");
    const auto output = runParser<dai::beta::node::ClassificationSequenceParser, dai::beta::Classifications>(fixture, platform);
    CHECK(output->classes == fixture.json.at("expected").at("class").get<std::vector<std::string>>());
}

TEST_CASE("EmbeddingsParser matches arcface golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("EmbeddingsParser", "arcface");
    const auto output = runParser<dai::beta::node::EmbeddingsParser, dai::NNData>(fixture, platform);
    const auto names = output->getAllLayerNames();
    REQUIRE(names.size() == 1);

    const auto& descriptor = fixture.json.at("expected").at("embeddings");
    const auto expected = readExpectedFloatArray(fixture, descriptor);
    const auto actual = readTensorFloat32(*output, names.front());
    REQUIRE(actual.size() == expected.size());
    const auto info = output->getTensorInfo(names.front());
    REQUIRE(info.has_value());
    CHECK(info->dims == descriptor.at("shape").get<std::vector<unsigned>>());
    for(std::size_t index = 0; index < actual.size(); ++index) {
        CAPTURE(index);
        checkRelative(actual[index], expected[index], "embedding value");
    }
}

TEST_CASE("FastSAMParser matches fastsam-s golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("FastSAMParser", "fastsam-s");
    const auto output = runParser<dai::beta::node::FastSAMParser, dai::SegmentationMask>(fixture, platform);
    const auto& descriptor = fixture.json.at("expected").at("mask");
    REQUIRE(descriptor.at("dtype") == "uint8");
    const auto expected = readBytes(fixture.directory / descriptor.at("file").get<std::string>());
    const auto shape = descriptor.at("shape").get<std::vector<std::size_t>>();
    REQUIRE(shape.size() == 2);
    REQUIRE(output->getHeight() == shape[0]);
    REQUIRE(output->getWidth() == shape[1]);
    const auto actual = output->getMaskData();
    REQUIRE(actual.size() == expected.size());
    const auto equal = std::inner_product(actual.begin(), actual.end(), expected.begin(), std::size_t{0}, std::plus<>(), std::equal_to<>());
    const double accuracy = static_cast<double>(equal) / static_cast<double>(expected.size());
    INFO("equal mask pixels: " << equal << '/' << expected.size() << ", accuracy: " << accuracy);
    CHECK(accuracy > 0.9);
}

TEST_CASE("HRNetParser matches lite-hrnet golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("HRNetParser", "lite-hrnet");
    const auto output = runParser<dai::beta::node::HRNetParser, dai::beta::Keypoints>(fixture, platform);
    checkKeypoints(*output, fixture.json.at("expected"));
}

TEST_CASE("ImageOutputParser matches dncnn3 golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("ImageOutputParser", "dncnn3");
    const auto output = runParser<dai::beta::node::ImageOutputParser, dai::ImgFrame>(fixture, platform);
    const auto& descriptor = fixture.json.at("expected").at("output");
    const auto expected = readExpectedFloatArray(fixture, descriptor);
    const auto shape = descriptor.at("shape").get<std::vector<std::size_t>>();
    REQUIRE(shape.size() == 2);
    REQUIRE(output->getHeight() == shape[0]);
    REQUIRE(output->getWidth() == shape[1]);
    const auto actual = output->getData();
    REQUIRE(actual.size() == expected.size());
    for(std::size_t index = 0; index < actual.size(); ++index) {
        CAPTURE(index);
        CHECK(static_cast<double>(actual[index]) == Catch::Approx(expected[index]).margin(1.0));
    }
}

TEST_CASE("KeypointParser matches mediapipe-face-landmarker golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("KeypointParser", "mediapipe-face-landmarker");
    const auto output = runParser<dai::beta::node::KeypointParser, dai::beta::Keypoints>(fixture, platform);
    checkKeypoints(*output, fixture.json.at("expected"));
}

TEST_CASE("LaneDetectionParser matches ultra-fast-lane-detection golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("LaneDetectionParser", "ultra-fast-lane-detection");
    const auto output = runParser<dai::beta::node::LaneDetectionParser, dai::beta::Clusters>(fixture, platform);
    const auto& expectedClusters = fixture.json.at("expected").at("clusters");
    REQUIRE(output->clusters.size() == expectedClusters.size());
    for(std::size_t clusterIndex = 0; clusterIndex < output->clusters.size(); ++clusterIndex) {
        CAPTURE(clusterIndex);
        const auto& actualPoints = output->clusters[clusterIndex].points;
        const auto& expectedPoints = expectedClusters[clusterIndex];
        REQUIRE(actualPoints.size() == expectedPoints.size());
        for(std::size_t expectedIndex = 0; expectedIndex < expectedPoints.size(); ++expectedIndex) {
            bool found = false;
            for(const auto& actualPoint : actualPoints) {
                if(std::abs(actualPoint.x - expectedPoints[expectedIndex][0].get<float>()) <= 0.001f
                   && std::abs(actualPoint.y - expectedPoints[expectedIndex][1].get<float>()) <= 0.001f) {
                    found = true;
                    break;
                }
            }
            CAPTURE(expectedIndex);
            CHECK(found);
        }
    }
}

TEST_CASE("MapOutputParser matches dm-count golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("MapOutputParser", "dm-count");
    const auto output = runParser<dai::beta::node::MapOutputParser, dai::beta::Map2D>(fixture, platform);
    const auto& descriptor = fixture.json.at("expected").at("map");
    const auto expected = readExpectedFloatArray(fixture, descriptor);
    const auto shape = descriptor.at("shape").get<std::vector<std::size_t>>();
    REQUIRE(shape.size() == 2);
    REQUIRE(output->getHeight() == shape[0]);
    REQUIRE(output->getWidth() == shape[1]);
    const auto actual = output->getMap();
    REQUIRE(actual.size() == expected.size());
    for(std::size_t index = 0; index < actual.size(); ++index) {
        CAPTURE(index);
        checkAbsolute(actual[index], expected[index], 0.001, "map value");
    }
}

TEST_CASE("MLSDParser matches m-lsd-tiny golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("MLSDParser", "m-lsd-tiny");
    const auto output = runParser<dai::beta::node::MLSDParser, dai::beta::Lines>(fixture, platform);
    const auto& expected = fixture.json.at("expected").at("lines");
    REQUIRE(output->lines.size() == expected.size());
    for(std::size_t index = 0; index < output->lines.size(); ++index) {
        CAPTURE(index);
        const auto& actual = output->lines[index];
        checkAbsolute(actual.confidence, expected[index].at("confidence").get<double>(), 0.01, "line confidence");
        checkAbsolute(actual.startPoint.x, expected[index].at("start_point")[0].get<double>(), 0.01, "line start x");
        checkAbsolute(actual.startPoint.y, expected[index].at("start_point")[1].get<double>(), 0.01, "line start y");
        checkAbsolute(actual.endPoint.x, expected[index].at("end_point")[0].get<double>(), 0.01, "line end x");
        checkAbsolute(actual.endPoint.y, expected[index].at("end_point")[1].get<double>(), 0.01, "line end y");
    }
}

TEST_CASE("MPPalmDetectionParser matches mediapipe-palm-detection golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("MPPalmDetectionParser", "mediapipe-palm-detection");
    const auto output = runParser<dai::beta::node::MPPalmDetectionParser, dai::ImgDetections>(fixture, platform);
    checkDetections(*output, fixture.json.at("expected"));
}

TEST_CASE("PPTextDetectionParser matches paddle-text-detection golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("PPTextDetectionParser", "paddle-text-detection");
    const auto output = runParser<dai::beta::node::PPTextDetectionParser, dai::ImgDetections>(fixture, platform, [](auto& parser) {
        // The recorded fixture predates an archive whose output was named softmax_2.tmp_0.
        parser.setOutputLayerName("output");
    });
    checkDetections(*output, fixture.json.at("expected"));
}

TEST_CASE("RegressionParser matches gaze-estimation-adas golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("RegressionParser", "gaze-estimation-adas");
    const auto output = runParser<dai::beta::node::RegressionParser, dai::beta::Predictions>(fixture, platform, [](auto& parser) {
        // The current RVC4 archive calls this output Identity; the recorded NNData uses Identity0.
        parser.setOutputLayerName("Identity0");
    });
    const auto& expected = fixture.json.at("expected").at("value");
    REQUIRE(output->predictions.size() == expected.size());
    for(std::size_t index = 0; index < output->predictions.size(); ++index) {
        CAPTURE(index);
        checkRelative(output->predictions[index].prediction, expected[index].get<double>(), "regression value");
    }
}

TEST_CASE("SCRFDParser matches scrfd-face-detection golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("SCRFDParser", "scrfd-face-detection");
    const auto output = runParser<dai::beta::node::SCRFDParser, dai::ImgDetections>(fixture, platform);
    checkDetections(*output, fixture.json.at("expected"));
}

TEST_CASE("SuperAnimalParser matches superanimal-landmarker golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("SuperAnimalParser", "superanimal-landmarker");
    const auto output = runParser<dai::beta::node::SuperAnimalParser, dai::beta::Keypoints>(fixture, platform);
    checkKeypoints(*output, fixture.json.at("expected"));
}

TEST_CASE("YuNetParser matches yunet golden output", "[beta][parser][onhost]") {
    const std::string platform = GENERATE("RVC2", "RVC4");
    CAPTURE(platform);
    const auto fixture = loadFixture("YuNetParser", "yunet");
    const auto output = runParser<dai::beta::node::YuNetParser, dai::ImgDetections>(fixture, platform);
    checkDetections(*output, fixture.json.at("expected"));
}
