#include <catch2/catch_all.hpp>
#include <depthai/depthai.hpp>
#include <type_traits>

namespace {

template <typename Parser>
void checkDeviceSerialization() {
    static_assert(std::is_base_of<dai::DeviceNode, Parser>::value, "Beta parsers must be device nodes");

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
            REQUIRE(properties.confidenceThreshold == 0.25f);
            REQUIRE(properties.numClasses == 3);
            REQUIRE(properties.points == std::make_pair<std::int32_t, std::int32_t>(17, 23));
            REQUIRE(properties.pointLabel == 1);
            REQUIRE(properties.boundingBox == std::array<std::int32_t, 4>{1, 2, 30, 40});
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
