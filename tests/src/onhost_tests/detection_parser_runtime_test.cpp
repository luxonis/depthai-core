#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstring>
#include <fp16/fp16.h>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"

namespace {
#ifdef DETECTION_PARSER_TEST_DEVICE
constexpr bool TEST_ON_DEVICE = true;
#else
constexpr bool TEST_ON_DEVICE = false;
#endif

std::shared_ptr<dai::NNData> makeDetections(bool yolo) {
    auto data = std::make_shared<dai::NNData>();
    dai::TensorInfo info;
    info.name = yolo ? "output_yolo" : "detections";
    info.order = dai::TensorInfo::StorageOrder::NCHW;
    info.dataType = dai::TensorInfo::DataType::FP16;
    info.numDimensions = 4;
    info.dims = yolo ? std::vector<unsigned>{1, 6, 1, 2} : std::vector<unsigned>{1, 1, 2, 7};
    info.strides = yolo ? std::vector<unsigned>{24, 4, 4, 2} : std::vector<unsigned>{28, 28, 14, 2};
    const std::vector<float> values = yolo ? std::vector<float>{0.5f, 1, 0.5f, 0.5f, 1, 0.5f, 0.5f, 0.5f, 1, 1, 0.75f, 0.5f}
                                         : std::vector<float>{0, 0, 0.75f, 0, 0, 0.75f, 1, 0, 0, 0.5f, 0.25f, 0, 1, 1};
    auto bytes = data->emplaceTensor(info);
    for(size_t i = 0; i < values.size(); ++i) {
        const auto value = fp16_ieee_from_fp32_value(values[i]);
        std::memcpy(bytes.data() + i * sizeof(value), &value, sizeof(value));
    }
    data->transformation = dai::ImgTransformation(16, 8);
    return data;
}
}  // namespace

TEST_CASE("DetectionParser updates runtime thresholds", "[detection-parser-runtime]") {
    const bool yolo = GENERATE(false, true);
    const bool syncConfig = GENERATE(false, true);
    dai::Pipeline pipeline(TEST_ON_DEVICE);
    auto parser = pipeline.create<dai::node::DetectionParser>();
    parser->setRunOnHost(!TEST_ON_DEVICE);
    parser->setNNFamily(yolo ? DetectionNetworkType::YOLO : DetectionNetworkType::MOBILENET);
    parser->setInputImageSize(16, 8);
    parser->setNumClasses(1);
    parser->setCoordinateSize(4);
    parser->setSubtype("yolov6r2");
    parser->setStrides({8});
    parser->setConfidenceThreshold(0.25f);
    parser->setIouThreshold(0.3f);
    parser->inputConfig.setWaitForMessage(syncConfig);
    auto input = parser->input.createInputQueue();
    auto configInput = parser->inputConfig.createInputQueue();
    auto output = parser->out.createOutputQueue();
    auto config = std::make_shared<dai::DetectionParserConfig>(*parser->initialConfig);
    auto data = makeDetections(yolo);
    pipeline.start();

    // In synchronous mode, a tensor alone must not produce a result.
    if(syncConfig) {
        input->send(data);
        bool timedOut = false;
        REQUIRE(output->get<dai::ImgDetections>(std::chrono::milliseconds(100), timedOut) == nullptr);
        REQUIRE(timedOut);
        configInput->send(config);
        auto first = output->get<dai::ImgDetections>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(first != nullptr);
        REQUIRE(first->detections.size() == (yolo ? 1 : 2));
    }

    auto checkCount = [&](size_t expected, bool sendConfig) {
        if(sendConfig && !syncConfig) configInput->send(config);
        // An asynchronous update may arrive while the parser is already waiting for a tensor.
        for(int attempt = 0; attempt < 3; ++attempt) {
            if(syncConfig) configInput->send(config);
            input->send(data);
            bool timedOut = false;
            auto result = output->get<dai::ImgDetections>(std::chrono::seconds(5), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(result != nullptr);
            if(syncConfig || result->detections.size() == expected || attempt == 2) {
                REQUIRE(result->detections.size() == expected);
                break;
            }
        }
    };
    checkCount(yolo ? 1 : 2, false);
    config->setConfidenceThreshold(0.9f);
    checkCount(0, true);
    checkCount(0, false);  // Updated thresholds persist without another config.
    config->setConfidenceThreshold(0.25f);
    config->setIouThreshold(0.8f);
    checkCount(2, true);
    // Runtime messages do not overwrite startup configuration.
    REQUIRE(parser->getConfidenceThreshold() == Catch::Approx(0.25f));
    REQUIRE(parser->getIouThreshold() == Catch::Approx(0.3f));
}

TEST_CASE("DetectionParserConfig serialization roundtrip", "[detection-parser-runtime]") {
    auto config = std::make_shared<dai::DetectionParserConfig>();
    config->setConfidenceThreshold(0.65f);
    config->setIouThreshold(0.4f);
    config->setSequenceNum(42);
    config->setTimestamp(std::chrono::steady_clock::now());
    std::shared_ptr<dai::DetectionParserConfig> result;
    if(TEST_ON_DEVICE) {
        dai::Pipeline pipeline;
        auto gate = pipeline.create<dai::node::Gate>();
        auto input = gate->input.createInputQueue();
        auto output = gate->output.createOutputQueue();
        pipeline.start();
        input->send(config);
        bool timedOut = false;
        result = output->get<dai::DetectionParserConfig>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
    } else {
        auto serialized = dai::StreamMessageParser::serializeMetadata(config);
        streamPacketDesc_t packet{};
        packet.data = serialized.data();
        packet.length = serialized.size();
        result = std::dynamic_pointer_cast<dai::DetectionParserConfig>(dai::StreamMessageParser::parseMessage(&packet));
    }
    REQUIRE(result != nullptr);
    REQUIRE(result->getConfidenceThreshold() == Catch::Approx(0.65f));
    REQUIRE(result->getIouThreshold() == Catch::Approx(0.4f));
    REQUIRE(result->getSequenceNum() == 42);
    REQUIRE(result->getTimestamp() == config->getTimestamp());
}

TEST_CASE("DetectionParser initialConfig is serialized in startup properties", "[detection-parser-runtime]") {
    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<dai::node::DetectionParser>();
    parser->setNNFamily(DetectionNetworkType::YOLO);
    parser->setNumClasses(5);
    parser->setConfidenceThreshold(0.25f);
    parser->setIouThreshold(0.3f);
    REQUIRE(parser->initialConfig->getConfidenceThreshold() == Catch::Approx(0.25f));
    REQUIRE(parser->initialConfig->getIouThreshold() == Catch::Approx(0.3f));
    parser->initialConfig->setConfidenceThreshold(0.65f);
    parser->initialConfig->setIouThreshold(0.8f);
    dai::DetectionParserProperties restored;
    dai::utility::deserialize(dai::utility::serialize(parser->getProperties()), restored);
    REQUIRE(restored.parser.confidenceThreshold == Catch::Approx(0.65f));
    REQUIRE(restored.parser.iouThreshold == Catch::Approx(0.8f));
    REQUIRE(restored.parser.nnFamily == DetectionNetworkType::YOLO);
    REQUIRE(restored.parser.classes == 5);
}

TEST_CASE("Detection networks preserve their default parser confidence", "[detection-parser-runtime]") {
    auto network = dai::node::DetectionNetwork::create(nullptr);
    auto spatialNetwork = dai::node::SpatialDetectionNetwork::create(nullptr);
    REQUIRE(network->getConfidenceThreshold() == Catch::Approx(0.5f));
    REQUIRE(spatialNetwork->getConfidenceThreshold() == Catch::Approx(0.5f));
}
