#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace {

template <typename Parser>
void checkDeviceSerialization() {
    static_assert(std::is_base_of<dai::DeviceNode, Parser>::value, "Beta parsers must be device nodes");
    static_assert(std::is_base_of<dai::beta::BetaNode, Parser>::value, "Beta parsers must inherit the shared beta-node behavior");

    dai::Pipeline pipeline(false);
    auto parser = pipeline.create<Parser>();
    REQUIRE(parser->runOnHost());

    const auto serialized = dai::utility::serialize(parser->properties);
    REQUIRE_FALSE(serialized.empty());

    typename Parser::Properties properties;
    dai::utility::deserialize(serialized, properties);

    parser->setRunOnHost(true);
    REQUIRE(parser->runOnHost());
}

template <typename Parser>
typename Parser::Properties getSerializedProperties(const dai::PipelineSchema& schema) {
    for(const auto& entry : schema.nodes) {
        const auto& node = entry.second;
        if(node.name == Parser::NAME) {
            typename Parser::Properties properties;
            dai::utility::deserialize(node.properties, properties);
            return properties;
        }
    }
    throw std::runtime_error(std::string("Parser is missing from pipeline schema: ") + Parser::NAME);
}

}  // namespace

TEST_CASE("Beta parsers serialize as device nodes") {
    checkDeviceSerialization<dai::beta::node::ClassificationParser>();
    checkDeviceSerialization<dai::beta::node::ClassificationSequenceParser>();
    checkDeviceSerialization<dai::beta::node::EmbeddingsParser>();
    checkDeviceSerialization<dai::beta::node::FastSAMParser>();
    checkDeviceSerialization<dai::beta::node::HRNetParser>();
    checkDeviceSerialization<dai::beta::node::ImageOutputParser>();
    checkDeviceSerialization<dai::beta::node::KeypointParser>();
    checkDeviceSerialization<dai::beta::node::LaneDetectionParser>();
    checkDeviceSerialization<dai::beta::node::MapOutputParser>();
    checkDeviceSerialization<dai::beta::node::MLSDParser>();
    checkDeviceSerialization<dai::beta::node::MPPalmDetectionParser>();
    checkDeviceSerialization<dai::beta::node::PPTextDetectionParser>();
    checkDeviceSerialization<dai::beta::node::RFDETRParser>();
    checkDeviceSerialization<dai::beta::node::RegressionParser>();
    checkDeviceSerialization<dai::beta::node::SCRFDParser>();
    checkDeviceSerialization<dai::beta::node::SuperAnimalParser>();
    checkDeviceSerialization<dai::beta::node::XFeatMonoParser>();
    checkDeviceSerialization<dai::beta::node::XFeatStereoParser>();
    checkDeviceSerialization<dai::beta::node::YuNetParser>();
}

TEST_CASE("Beta parser configuration round trips through properties") {
    dai::Pipeline pipeline(false);

    auto classification = pipeline.create<dai::beta::node::ClassificationParser>();
    classification->setOutputLayerName("scores");
    classification->setClasses({"first", "second"});
    classification->setSoftmax(false);

    auto fastSam = pipeline.create<dai::beta::node::FastSAMParser>();
    fastSam->setConfidenceThreshold(0.25f);
    fastSam->setNumClasses(3);
    fastSam->setPoints(17, 23);
    fastSam->setPointLabel(1);
    fastSam->setBoundingBox({1, 2, 30, 40});

    auto keypoint = pipeline.create<dai::beta::node::KeypointParser>();
    keypoint->setNumKeypoints(4);
    keypoint->setEdges({{0, 1}, {2, 3}});

    auto xfeat = pipeline.create<dai::beta::node::XFeatMonoParser>();
    xfeat->setOriginalSize(1280, 720);
    xfeat->setInputSize(640, 360);

    const auto schema = pipeline.getPipelineSchema();

    for(const auto& entry : schema.nodes) {
        const auto& node = entry.second;
        if(node.name == dai::beta::node::ClassificationParser::NAME) {
            dai::beta::ClassificationParserProperties properties;
            dai::utility::deserialize(node.properties, properties);
            REQUIRE(properties.outputLayerName == "scores");
            REQUIRE(properties.classes == std::vector<std::string>{"first", "second"});
            REQUIRE(properties.nClasses == 2);
            REQUIRE_FALSE(properties.isSoftmax);
        } else if(node.name == dai::beta::node::FastSAMParser::NAME) {
            dai::beta::FastSAMParserProperties properties;
            dai::utility::deserialize(node.properties, properties);
            REQUIRE(properties.initialConfig.confidenceThreshold == 0.25f);
            REQUIRE(properties.numClasses == 3);
            REQUIRE(properties.initialConfig.points == std::make_pair<std::int32_t, std::int32_t>(17, 23));
            REQUIRE(properties.initialConfig.pointLabel == 1);
            REQUIRE(properties.initialConfig.boundingBox == std::array<std::int32_t, 4>{1, 2, 30, 40});
        } else if(node.name == dai::beta::node::KeypointParser::NAME) {
            dai::beta::KeypointParserProperties properties;
            dai::utility::deserialize(node.properties, properties);
            REQUIRE(properties.nKeypoints == 4);
            REQUIRE(properties.edges == std::vector<dai::Edge>{{0, 1}, {2, 3}});
        } else if(node.name == dai::beta::node::XFeatMonoParser::NAME) {
            dai::beta::XFeatMonoParserProperties properties;
            dai::utility::deserialize(node.properties, properties);
            REQUIRE(properties.originalSize == std::make_pair<std::uint32_t, std::uint32_t>(1280, 720));
            REQUIRE(properties.inputSize == std::make_pair<std::uint32_t, std::uint32_t>(640, 360));
        }
    }
}

TEST_CASE("Runtime parser setters populate initialConfig while model-contract fields stay static") {
    dai::Pipeline pipeline(false);

    auto fastSam = pipeline.create<dai::beta::node::FastSAMParser>();
    fastSam->setConfidenceThreshold(0.2f);
    fastSam->setIouThreshold(0.3f);
    fastSam->setMaskConfidence(0.4f);
    fastSam->setNumClasses(7);

    auto hrnet = pipeline.create<dai::beta::node::HRNetParser>();
    hrnet->setScoreThreshold(0.2f);
    hrnet->setOutputLayerName("hrnet-output");

    auto mlsd = pipeline.create<dai::beta::node::MLSDParser>();
    mlsd->setTopK(20);
    mlsd->setScoreThreshold(0.3f);
    mlsd->setDistanceThreshold(4.0f);
    mlsd->setInputSize(320, 240);

    auto palm = pipeline.create<dai::beta::node::MPPalmDetectionParser>();
    palm->setConfidenceThreshold(0.2f);
    palm->setIouThreshold(0.3f);
    palm->setMaxDetections(4);
    palm->setScale(256);

    auto ppText = pipeline.create<dai::beta::node::PPTextDetectionParser>();
    ppText->setConfidenceThreshold(0.2f);
    ppText->setMaskThreshold(0.3f);
    ppText->setMaxDetections(4);
    ppText->setOutputLayerName("text-output");

    auto rfdetr = pipeline.create<dai::beta::node::RFDETRParser>();
    rfdetr->setConfidenceThreshold(0.2f);
    rfdetr->setMaxDetections(4);
    rfdetr->setMaskConfidence(0.3f);
    rfdetr->setLabelNames({"object"});

    auto scrfd = pipeline.create<dai::beta::node::SCRFDParser>();
    scrfd->setConfidenceThreshold(0.2f);
    scrfd->setIouThreshold(0.3f);
    scrfd->setMaxDetections(4);
    scrfd->setInputSize(320, 240);

    auto superAnimal = pipeline.create<dai::beta::node::SuperAnimalParser>();
    superAnimal->setScoreThreshold(0.2f);
    superAnimal->setNumKeypoints(12);

    auto yunet = pipeline.create<dai::beta::node::YuNetParser>();
    yunet->setConfidenceThreshold(0.2f);
    yunet->setIouThreshold(0.3f);
    yunet->setMaxDetections(4);
    yunet->setInputSize(320, 240);

    auto sequence = pipeline.create<dai::beta::node::ClassificationSequenceParser>();
    sequence->setIgnoredIndexes({1, 3});
    sequence->setRemoveDuplicates(true);
    sequence->setConcatenateClasses(true);
    sequence->setClasses({"a", "b", "c", "d"});

    auto map = pipeline.create<dai::beta::node::MapOutputParser>();
    map->setMinMaxScaling(true);
    map->setOutputLayerName("map-output");

    auto mono = pipeline.create<dai::beta::node::XFeatMonoParser>();
    mono->setMaxKeypoints(128);
    mono->setOriginalSize(1280, 720);

    auto stereo = pipeline.create<dai::beta::node::XFeatStereoParser>();
    stereo->setMaxKeypoints(64);
    stereo->setInputSize(320, 192);

    const auto schema = pipeline.getPipelineSchema();

    const auto fastSamProperties = getSerializedProperties<dai::beta::node::FastSAMParser>(schema);
    CHECK(fastSamProperties.initialConfig.confidenceThreshold == 0.2f);
    CHECK(fastSamProperties.initialConfig.iouThreshold == 0.3f);
    CHECK(fastSamProperties.initialConfig.maskConfidence == 0.4f);
    CHECK(fastSamProperties.numClasses == 7);

    const auto hrnetProperties = getSerializedProperties<dai::beta::node::HRNetParser>(schema);
    CHECK(hrnetProperties.initialConfig.scoreThreshold == 0.2f);
    CHECK(hrnetProperties.outputLayerName == "hrnet-output");

    const auto mlsdProperties = getSerializedProperties<dai::beta::node::MLSDParser>(schema);
    CHECK(mlsdProperties.initialConfig.topK == 20);
    CHECK(mlsdProperties.initialConfig.scoreThreshold == 0.3f);
    CHECK(mlsdProperties.initialConfig.distanceThreshold == 4.0f);
    CHECK(mlsdProperties.inputSize == std::make_pair<std::uint32_t, std::uint32_t>(320, 240));

    const auto palmProperties = getSerializedProperties<dai::beta::node::MPPalmDetectionParser>(schema);
    CHECK(palmProperties.initialConfig.confidenceThreshold == 0.2f);
    CHECK(palmProperties.initialConfig.iouThreshold == 0.3f);
    CHECK(palmProperties.initialConfig.maxDetections == 4);
    CHECK(palmProperties.scale == 256);

    const auto ppTextProperties = getSerializedProperties<dai::beta::node::PPTextDetectionParser>(schema);
    CHECK(ppTextProperties.initialConfig.confidenceThreshold == 0.2f);
    CHECK(ppTextProperties.initialConfig.maskThreshold == 0.3f);
    CHECK(ppTextProperties.initialConfig.maxDetections == 4);
    CHECK(ppTextProperties.outputLayerName == "text-output");

    const auto rfdetrProperties = getSerializedProperties<dai::beta::node::RFDETRParser>(schema);
    CHECK(rfdetrProperties.initialConfig.confidenceThreshold == 0.2f);
    CHECK(rfdetrProperties.initialConfig.maxDetections == 4);
    CHECK(rfdetrProperties.initialConfig.maskConfidence == 0.3f);
    CHECK(rfdetrProperties.labelNames == std::vector<std::string>{"object"});

    const auto scrfdProperties = getSerializedProperties<dai::beta::node::SCRFDParser>(schema);
    CHECK(scrfdProperties.initialConfig.confidenceThreshold == 0.2f);
    CHECK(scrfdProperties.initialConfig.iouThreshold == 0.3f);
    CHECK(scrfdProperties.initialConfig.maxDetections == 4);
    CHECK(scrfdProperties.inputSize == std::make_pair<std::uint32_t, std::uint32_t>(320, 240));

    const auto superAnimalProperties = getSerializedProperties<dai::beta::node::SuperAnimalParser>(schema);
    CHECK(superAnimalProperties.initialConfig.scoreThreshold == 0.2f);
    CHECK(superAnimalProperties.nKeypoints == 12);

    const auto yunetProperties = getSerializedProperties<dai::beta::node::YuNetParser>(schema);
    CHECK(yunetProperties.initialConfig.confidenceThreshold == 0.2f);
    CHECK(yunetProperties.initialConfig.iouThreshold == 0.3f);
    CHECK(yunetProperties.initialConfig.maxDetections == 4);
    CHECK(yunetProperties.inputSize == std::make_pair<std::uint32_t, std::uint32_t>(320, 240));

    const auto sequenceProperties = getSerializedProperties<dai::beta::node::ClassificationSequenceParser>(schema);
    CHECK(sequenceProperties.initialConfig.ignoredIndexes == std::vector<std::int32_t>{1, 3});
    CHECK(sequenceProperties.initialConfig.removeDuplicates);
    CHECK(sequenceProperties.initialConfig.concatenateClasses);
    CHECK(sequenceProperties.classes == std::vector<std::string>{"a", "b", "c", "d"});

    const auto mapProperties = getSerializedProperties<dai::beta::node::MapOutputParser>(schema);
    CHECK(mapProperties.initialConfig.minMaxScaling);
    CHECK(mapProperties.outputLayerName == "map-output");

    const auto monoProperties = getSerializedProperties<dai::beta::node::XFeatMonoParser>(schema);
    CHECK(monoProperties.initialConfig.maxKeypoints == 128);
    CHECK(monoProperties.originalSize == std::make_pair<std::uint32_t, std::uint32_t>(1280, 720));

    const auto stereoProperties = getSerializedProperties<dai::beta::node::XFeatStereoParser>(schema);
    CHECK(stereoProperties.initialConfig.maxKeypoints == 64);
    CHECK(stereoProperties.inputSize == std::make_pair<std::uint32_t, std::uint32_t>(320, 192));
}
