//
// Created by thwdpc on 7/24/25.
//

#include "../ParserGenerator.hpp"

#include <iosfwd>
#include <variant>
#include <vector>

#include "utility/ErrorMacros.hpp"
#include "parsers/SimCCKeypointParser.hpp"

namespace dai::node {

// Utility for device parser names
const std::vector<std::string> DEVICE_PARSERS = {"YOLO", "SSD"};

static const std::unordered_map<std::string, std::function<std::shared_ptr<BaseParser>(Pipeline&)>> parserMap = {
    // { "YOLOExtendedParser", [](){ return std::make_shared<YOLOExtendedParser>(); } },
    { std::string(KeypointParser::NAME), [](Pipeline& p) { return std::static_pointer_cast<BaseParser>(p.create<KeypointParser>()); }},
    { std::string(SimCCKeypointParser::NAME), [](Pipeline& p) { return std::static_pointer_cast<BaseParser>(p.create<SimCCKeypointParser>()); }}
};

std::shared_ptr<BaseParser> getHostParserByName(const std::string& parserName, Pipeline& pipeline) {
    std::string parserNameExtended;
    if(parserName == "YOLO") {
        parserNameExtended = "YOLOExtendedParser";
    } else if(parserName == "SSD") {
        parserNameExtended = "SSDExtendedParser";
    } else {
        parserNameExtended = parserName;
    }
    DAI_CHECK(parserMap.find(parserNameExtended) != parserMap.end(), "Parser " + parserNameExtended + " not found");
    return parserMap.find(parserNameExtended)->second(pipeline);
}

std::vector<HostOrDeviceParser> ParserGenerator::generateAllParsers(Pipeline pipeline, const NNArchive& nnArchive, const bool hostOnly) {
    auto [model, heads] = archiveGetModelEnsureOneHeadV1(nnArchive, pipeline.getDefaultDevice()->getPlatform());

    std::vector<HostOrDeviceParser> parsers;

    for(int i = 0; i < heads.size(); i++) {
        HostOrDeviceParser parser = generateOneV1Parser(pipeline, nnArchive, heads[i], model, hostOnly);
        parsers.push_back(std::move(parser));
    }
    return parsers;
}


ConfigModelWithHeads ParserGenerator::archiveGetModelEnsureOneHeadV1(const NNArchive& nnArchive, const Platform targetPlatform) {
    const auto& nnArchiveCfg = nnArchive.getVersionedConfig();

    DAI_CHECK_V(nnArchiveCfg.getVersion() == NNArchiveConfigVersion::V1, "Only V1 configs are supported for NeuralNetwork.build method");
    auto supportedPlatforms = nnArchive.getSupportedPlatforms();
    bool platformSupported = std::find(supportedPlatforms.begin(), supportedPlatforms.end(), targetPlatform) != supportedPlatforms.end();
    DAI_CHECK_V(platformSupported, "Platform not supported by the neural network model");

    // Get model heads
    auto [_, model] = nnArchive.getConfig<nn_archive::v1::Config>();

    if(const auto headsOpt = model.heads) {
        if(const auto headsV1 = *headsOpt; !headsV1.empty()) {
            return ConfigModelWithHeads{.model = model, .heads = headsV1};
        }
    }
    throw std::runtime_error(fmt::format("No heads defined in the NN Archive."));
}

HostOrDeviceParser ParserGenerator::generateOneV1Parser(
    Pipeline& pipeline, const NNArchive& owningArchive, const nn_archive::v1::Head& head, const nn_archive::v1::Model& model, const bool hostOnly) {
    std::string parser_name = head.parser;

    // If this *could* be an on-device parser(currently just DetectionParser) then check whether that's allowed by !hostOnly
    if(std::find(DEVICE_PARSERS.begin(), DEVICE_PARSERS.end(), parser_name) != DEVICE_PARSERS.end() && !hostOnly) {
        // Device parser handling
        auto device_parser = pipeline.create<DetectionParser>();
        device_parser->setNNArchive(owningArchive);
        return device_parser;
    }
    return getHostParserByName(parser_name, pipeline)->build(head, model);
}

}  // namespace dai::node
