#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstring>
#include <memory>
#include <type_traits>

#include "depthai/beta/datatypes.hpp"
#include "depthai/beta/nodes.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"
#include "depthai/utility/Serialization.hpp"

namespace {

template <typename Config, typename Configure>
void checkConfigRoundTrip(Configure&& configure) {
    static_assert(std::is_base_of<dai::Buffer, Config>::value, "Parser configs must be Buffer messages");

    Config original;
    configure(original);
    REQUIRE(original.validate());

    const auto metadata = dai::utility::serialize(original);
    Config metadataCopy;
    dai::utility::deserialize(metadata, metadataCopy);
    REQUIRE(metadata == dai::utility::serialize(metadataCopy));
    REQUIRE(metadataCopy.getDatatype() == original.getDatatype());

    auto packetBytes = dai::StreamMessageParser::serializeMetadata(original);
    streamPacketDesc_t packet{};
    packet.data = packetBytes.data();
    packet.length = packetBytes.size();
    packet.fd = -1;

    const auto parsed = std::dynamic_pointer_cast<Config>(dai::StreamMessageParser::parseMessage(&packet));
    REQUIRE(parsed);
    REQUIRE(parsed->getDatatype() == original.getDatatype());
    REQUIRE(dai::utility::serialize(*parsed) == metadata);
}

void addTensor(const std::shared_ptr<dai::NNData>& data, const std::string& name, const std::vector<unsigned>& dimensions, const std::vector<float>& values) {
    std::size_t count = 1;
    for(const auto dimension : dimensions) count *= dimension;
    REQUIRE(count == values.size());

    dai::TensorInfo tensor;
    tensor.name = name;
    tensor.dataType = dai::TensorInfo::DataType::FP32;
    tensor.numDimensions = dimensions.size();
    tensor.dims = dimensions;
    switch(dimensions.size()) {
        case 1:
            tensor.order = dai::TensorInfo::StorageOrder::C;
            break;
        case 2:
            tensor.order = dai::TensorInfo::StorageOrder::NC;
            break;
        case 3:
            tensor.order = dai::TensorInfo::StorageOrder::CHW;
            break;
        case 4:
            tensor.order = dai::TensorInfo::StorageOrder::NCHW;
            break;
        default:
            FAIL("Unsupported test tensor rank");
    }

    tensor.strides.resize(dimensions.size());
    unsigned stride = sizeof(float);
    for(std::size_t index = dimensions.size(); index-- > 0;) {
        tensor.strides[index] = stride;
        stride *= dimensions[index];
    }
    tensor.validateStorageOrder();

    auto destination = data->emplaceTensor(tensor);
    REQUIRE(destination.size() == values.size() * sizeof(float));
    std::memcpy(destination.data(), values.data(), destination.size());
}

std::shared_ptr<dai::NNData> makeXFeatData() {
    auto data = std::make_shared<dai::NNData>();
    addTensor(data, "feats", {1, 2, 1, 2}, {1.0f, 0.0f, 0.0f, 1.0f});

    std::vector<float> keypoints(1 * 64 * 1 * 2, 0.0f);
    keypoints[9 * 2] = 10.0f;
    keypoints[18 * 2 + 1] = 10.0f;
    addTensor(data, "keypoints", {1, 64, 1, 2}, keypoints);
    addTensor(data, "heatmaps", {1, 1, 1, 2}, {1.0f, 1.0f});
    return data;
}

}  // namespace

TEST_CASE("Beta parser Config defaults are valid", "[beta][config]") {
    CHECK(dai::beta::FastSAMParserConfig{}.validate());
    CHECK(dai::beta::HRNetParserConfig{}.validate());
    CHECK(dai::beta::MLSDParserConfig{}.validate());
    CHECK(dai::beta::MPPalmDetectionParserConfig{}.validate());
    CHECK(dai::beta::PPTextDetectionParserConfig{}.validate());
    CHECK(dai::beta::RFDETRParserConfig{}.validate());
    CHECK(dai::beta::SCRFDParserConfig{}.validate());
    CHECK(dai::beta::SuperAnimalParserConfig{}.validate());
    CHECK(dai::beta::YuNetParserConfig{}.validate());
    CHECK(dai::beta::ClassificationSequenceParserConfig{}.validate());
    CHECK(dai::beta::MapOutputParserConfig{}.validate());
    CHECK(dai::beta::XFeatMonoParserConfig{}.validate());
    CHECK(dai::beta::XFeatStereoParserConfig{}.validate());
}

TEST_CASE("FastSAM prompt Config is validated atomically", "[beta][config][FastSAM]") {
    dai::beta::FastSAMParserConfig config;
    config.prompt = dai::beta::FastSAMParserConfig::Prompt::POINT;
    CHECK_FALSE(config.validate());
    config.points = std::make_pair<std::int32_t, std::int32_t>(17, 23);
    CHECK_FALSE(config.validate());
    config.pointLabel = 1;
    CHECK(config.validate());

    config.prompt = dai::beta::FastSAMParserConfig::Prompt::BOUNDING_BOX;

    config.pointLabel = 1;
    config.boundingBox = std::array<std::int32_t, 4>{10, 10, 0, 20};
    CHECK_FALSE(config.validate());
    config.boundingBox = std::array<std::int32_t, 4>{20, 10, 10, 20};
    CHECK_FALSE(config.validate());
    CHECK_THROWS(config.setBoundingBox({20, 10, 10, 20}));
    CHECK_FALSE(config.validate());
    config.boundingBox = std::array<std::int32_t, 4>{1, 2, 30, 40};
    CHECK(config.validate());

    config.pointLabel = 2;
    CHECK_FALSE(config.validate());
    CHECK_THROWS(config.setPointLabel(2));
}

TEST_CASE("Invalid public Config fields are rejected", "[beta][config][validation]") {
    dai::beta::HRNetParserConfig threshold;
    threshold.scoreThreshold = 1.1f;
    CHECK_FALSE(threshold.validate());

    dai::beta::MLSDParserConfig count;
    count.topK = 0;
    CHECK_FALSE(count.validate());

    dai::beta::ClassificationSequenceParserConfig indexes;
    indexes.ignoredIndexes = {-1};
    CHECK_FALSE(indexes.validate());

    dai::beta::XFeatStereoParserConfig features;
    features.maxKeypoints = 0;
    CHECK_FALSE(features.validate());

    dai::beta::YuNetParserConfig unlimitedDetections;
    unlimitedDetections.setMaxDetections(0);
    CHECK(unlimitedDetections.validate());
}

TEST_CASE("Every beta parser Config round trips through metadata and stream transport", "[beta][config][serialization]") {
    SECTION("FastSAM") {
        checkConfigRoundTrip<dai::beta::FastSAMParserConfig>([](auto& config) {
            config.confidenceThreshold = 0.25f;
            config.iouThreshold = 0.4f;
            config.maskConfidence = 0.75f;
        });
    }
    SECTION("HRNet") {
        checkConfigRoundTrip<dai::beta::HRNetParserConfig>([](auto& config) { config.scoreThreshold = 0.25f; });
    }
    SECTION("MLSD") {
        checkConfigRoundTrip<dai::beta::MLSDParserConfig>([](auto& config) {
            config.topK = 25;
            config.scoreThreshold = 0.2f;
            config.distanceThreshold = 8.0f;
        });
    }
    SECTION("MPPalmDetection") {
        checkConfigRoundTrip<dai::beta::MPPalmDetectionParserConfig>([](auto& config) {
            config.confidenceThreshold = 0.25f;
            config.iouThreshold = 0.4f;
            config.maxDetections = 5;
        });
    }
    SECTION("PPTextDetection") {
        checkConfigRoundTrip<dai::beta::PPTextDetectionParserConfig>([](auto& config) {
            config.confidenceThreshold = 0.25f;
            config.maskThreshold = 0.4f;
            config.maxDetections = 5;
        });
    }
    SECTION("RFDETR") {
        checkConfigRoundTrip<dai::beta::RFDETRParserConfig>([](auto& config) {
            config.confidenceThreshold = 0.25f;
            config.maxDetections = 5;
            config.maskConfidence = 0.4f;
        });
    }
    SECTION("SCRFD") {
        checkConfigRoundTrip<dai::beta::SCRFDParserConfig>([](auto& config) {
            config.confidenceThreshold = 0.25f;
            config.iouThreshold = 0.4f;
            config.maxDetections = 5;
        });
    }
    SECTION("SuperAnimal") {
        checkConfigRoundTrip<dai::beta::SuperAnimalParserConfig>([](auto& config) { config.scoreThreshold = 0.25f; });
    }
    SECTION("YuNet") {
        checkConfigRoundTrip<dai::beta::YuNetParserConfig>([](auto& config) {
            config.confidenceThreshold = 0.25f;
            config.iouThreshold = 0.4f;
            config.maxDetections = 5;
        });
    }
    SECTION("ClassificationSequence") {
        checkConfigRoundTrip<dai::beta::ClassificationSequenceParserConfig>([](auto& config) {
            config.ignoredIndexes = {0, 2};
            config.removeDuplicates = true;
            config.concatenateClasses = true;
        });
    }
    SECTION("MapOutput") {
        checkConfigRoundTrip<dai::beta::MapOutputParserConfig>([](auto& config) { config.minMaxScaling = true; });
    }
    SECTION("XFeatMono") {
        checkConfigRoundTrip<dai::beta::XFeatMonoParserConfig>([](auto& config) { config.maxKeypoints = 128; });
    }
    SECTION("XFeatStereo") {
        checkConfigRoundTrip<dai::beta::XFeatStereoParserConfig>([](auto& config) { config.maxKeypoints = 128; });
    }
}

TEST_CASE("MapOutput runtime Config drains to the newest value and persists", "[beta][config][runtime]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::beta::node::MapOutputParser>();
    parser->setOutputLayerName("map");

    auto inputQueue = parser->input.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();

    auto nnData = std::make_shared<dai::NNData>();
    nnData->addTensor("map", std::vector<float>{2.0f, 4.0f});

    auto getOutput = [&]() {
        bool timedOut = false;
        auto output = outputQueue->get<dai::beta::Map2D>(std::chrono::seconds(1), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output);
        return output;
    };

    auto staleConfig = std::make_shared<dai::beta::MapOutputParserConfig>();
    staleConfig->minMaxScaling = true;
    auto newestConfig = std::make_shared<dai::beta::MapOutputParserConfig>();
    newestConfig->minMaxScaling = false;
    configQueue->send(staleConfig);
    configQueue->send(newestConfig);

    pipeline.start();
    inputQueue->send(nnData);
    CHECK(getOutput()->getMap() == std::vector<float>{2.0f, 4.0f});

    auto scalingConfig = std::make_shared<dai::beta::MapOutputParserConfig>();
    scalingConfig->minMaxScaling = true;
    configQueue->send(scalingConfig);
    inputQueue->send(nnData);
    getOutput();  // Wake a node that may already be blocked on its primary input.

    inputQueue->send(nnData);
    CHECK(getOutput()->getMap() == std::vector<float>{0.0f, 1.0f});

    inputQueue->send(nnData);
    CHECK(getOutput()->getMap() == std::vector<float>{0.0f, 1.0f});
}

TEST_CASE("Synchronized MapOutput runtime Config is consumed before its frame", "[beta][config][runtime][sync]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::beta::node::MapOutputParser>();
    parser->setOutputLayerName("map");
    parser->inputConfig.setWaitForMessage(true);

    auto inputQueue = parser->input.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();

    auto nnData = std::make_shared<dai::NNData>();
    nnData->addTensor("map", std::vector<float>{2.0f, 4.0f});
    auto config = std::make_shared<dai::beta::MapOutputParserConfig>();
    config->minMaxScaling = true;

    pipeline.start();
    inputQueue->send(nnData);
    configQueue->send(config);

    bool timedOut = false;
    auto output = outputQueue->get<dai::beta::Map2D>(std::chrono::seconds(1), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(output);
    CHECK(output->getMap() == std::vector<float>{0.0f, 1.0f});
}

TEST_CASE("Invalid ClassificationSequence Config does not partially replace active state", "[beta][config][runtime][validation]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::beta::node::ClassificationSequenceParser>();
    parser->setOutputLayerName("sequence");
    parser->setClasses({"a", "b", "c"});
    parser->setSoftmax(true);

    auto inputQueue = parser->input.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();

    auto nnData = std::make_shared<dai::NNData>();
    dai::TensorInfo tensor;
    tensor.name = "sequence";
    tensor.dataType = dai::TensorInfo::DataType::FP32;
    tensor.order = dai::TensorInfo::StorageOrder::NC;
    tensor.numDimensions = 2;
    tensor.dims = {2, 3};
    tensor.strides = {3 * sizeof(float), sizeof(float)};
    tensor.validateStorageOrder();
    auto tensorBytes = nnData->emplaceTensor(tensor);
    const std::vector<float> values{0.1f, 0.8f, 0.1f, 0.1f, 0.8f, 0.1f};
    std::memcpy(tensorBytes.data(), values.data(), tensorBytes.size());

    auto getClasses = [&]() {
        bool timedOut = false;
        auto output = outputQueue->get<dai::beta::Classifications>(std::chrono::seconds(1), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output);
        return output->classes;
    };

    pipeline.start();

    auto validConfig = std::make_shared<dai::beta::ClassificationSequenceParserConfig>();
    validConfig->removeDuplicates = true;
    configQueue->send(validConfig);
    inputQueue->send(nnData);
    getClasses();  // Wake a node that may already be blocked on its primary input.

    inputQueue->send(nnData);
    CHECK(getClasses() == std::vector<std::string>{"b"});

    auto invalidConfig = std::make_shared<dai::beta::ClassificationSequenceParserConfig>();
    invalidConfig->ignoredIndexes = {-1};
    invalidConfig->removeDuplicates = false;
    configQueue->send(invalidConfig);
    inputQueue->send(nnData);
    CHECK(getClasses() == std::vector<std::string>{"b"});

    inputQueue->send(nnData);
    CHECK(getClasses() == std::vector<std::string>{"b"});
}
TEST_CASE("RFDETR runtime Config changes the maximum detection count", "[beta][config][runtime][RFDETR]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::beta::node::RFDETRParser>();
    parser->setOutputLayerNames({"boxes", "logits"});
    parser->inputConfig.setWaitForMessage(true);

    auto inputQueue = parser->input.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();

    auto nnData = std::make_shared<dai::NNData>();
    addTensor(nnData, "boxes", {1, 2, 4}, {0.25f, 0.25f, 0.2f, 0.2f, 0.75f, 0.75f, 0.2f, 0.2f});
    addTensor(nnData, "logits", {1, 2, 1}, {5.0f, 4.0f});

    const auto getOutput = [&]() {
        bool timedOut = false;
        auto output = outputQueue->get<dai::ImgDetections>(std::chrono::seconds(1), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output);
        return output;
    };

    pipeline.start();

    auto initial = std::make_shared<dai::beta::RFDETRParserConfig>(*parser->initialConfig);
    configQueue->send(initial);
    inputQueue->send(nnData);
    CHECK(getOutput()->detections.size() == 2);

    auto limited = std::make_shared<dai::beta::RFDETRParserConfig>(*initial);
    limited->maxDetections = 1;
    configQueue->send(limited);
    inputQueue->send(nnData);
    CHECK(getOutput()->detections.size() == 1);
}

TEST_CASE("SCRFD runtime Config changes the maximum detection count", "[beta][config][runtime][SCRFD]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::beta::node::SCRFDParser>();
    parser->setOutputLayerNames({"score_8", "bbox_8", "kps_8"});
    parser->setInputSize(16, 8);
    parser->setFeatStrideFPN({8});
    parser->setNumAnchors(1);
    parser->inputConfig.setWaitForMessage(true);

    auto inputQueue = parser->input.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();

    auto nnData = std::make_shared<dai::NNData>();
    addTensor(nnData, "score_8", {1, 2}, {0.9f, 0.8f});
    addTensor(nnData, "bbox_8", {2, 4}, {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f});
    addTensor(nnData, "kps_8", {2, 10}, std::vector<float>(20, 0.0f));

    const auto getOutput = [&]() {
        bool timedOut = false;
        auto output = outputQueue->get<dai::ImgDetections>(std::chrono::seconds(1), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output);
        return output;
    };

    pipeline.start();

    auto initial = std::make_shared<dai::beta::SCRFDParserConfig>(*parser->initialConfig);
    configQueue->send(initial);
    inputQueue->send(nnData);
    CHECK(getOutput()->detections.size() == 2);

    auto limited = std::make_shared<dai::beta::SCRFDParserConfig>(*initial);
    limited->maxDetections = 1;
    configQueue->send(limited);
    inputQueue->send(nnData);
    CHECK(getOutput()->detections.size() == 1);
}

TEST_CASE("XFeatMono runtime Config changes the feature count", "[beta][config][runtime][XFeat]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::beta::node::XFeatMonoParser>();
    parser->setOutputLayerFeats("feats");
    parser->setOutputLayerKeypoints("keypoints");
    parser->setOutputLayerHeatmaps("heatmaps");
    parser->setOriginalSize(16, 8);
    parser->setInputSize(16, 8);
    parser->inputConfig.setWaitForMessage(true);

    auto inputQueue = parser->input.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();
    auto nnData = makeXFeatData();

    const auto getOutput = [&]() {
        bool timedOut = false;
        auto output = outputQueue->get<dai::TrackedFeatures>(std::chrono::seconds(1), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output);
        return output;
    };

    auto initial = std::make_shared<dai::beta::XFeatMonoParserConfig>(*parser->initialConfig);
    pipeline.start();

    parser->setTrigger();
    configQueue->send(initial);
    inputQueue->send(nnData);
    CHECK(getOutput()->trackedFeatures.empty());

    configQueue->send(initial);
    inputQueue->send(nnData);
    CHECK(getOutput()->trackedFeatures.size() == 4);

    auto limited = std::make_shared<dai::beta::XFeatMonoParserConfig>(*initial);
    limited->maxKeypoints = 1;
    configQueue->send(limited);
    inputQueue->send(nnData);
    CHECK(getOutput()->trackedFeatures.size() == 2);
}

TEST_CASE("XFeatStereo applies one runtime Config snapshot to a frame pair", "[beta][config][runtime][XFeat]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::beta::node::XFeatStereoParser>();
    parser->setOutputLayerFeats("feats");
    parser->setOutputLayerKeypoints("keypoints");
    parser->setOutputLayerHeatmaps("heatmaps");
    parser->setOriginalSize(16, 8);
    parser->setInputSize(16, 8);
    parser->inputConfig.setWaitForMessage(true);

    auto referenceQueue = parser->referenceInput.createInputQueue();
    auto targetQueue = parser->targetInput.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();
    auto nnData = makeXFeatData();

    const auto getOutput = [&]() {
        bool timedOut = false;
        auto output = outputQueue->get<dai::TrackedFeatures>(std::chrono::seconds(1), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output);
        return output;
    };

    pipeline.start();

    auto initial = std::make_shared<dai::beta::XFeatStereoParserConfig>(*parser->initialConfig);
    configQueue->send(initial);
    referenceQueue->send(nnData);
    targetQueue->send(nnData);
    CHECK(getOutput()->trackedFeatures.size() == 4);

    auto limited = std::make_shared<dai::beta::XFeatStereoParserConfig>(*initial);
    limited->maxKeypoints = 1;
    configQueue->send(limited);
    referenceQueue->send(nnData);
    targetQueue->send(nnData);
    CHECK(getOutput()->trackedFeatures.size() == 2);
}
