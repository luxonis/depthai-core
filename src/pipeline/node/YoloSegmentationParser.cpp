#include "depthai/pipeline/node/YoloSegmentationParser.hpp"

#include <memory>
#include <string>

#include "common/ModelType.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "nn_archive/NNArchive.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "spdlog/fmt/fmt.h"

// internal headers
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

void YoloSegmentationParser::setNNArchive(const NNArchive& nnArchive) {
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::BLOB:
        case dai::model::ModelType::SUPERBLOB:
        case dai::model::ModelType::DLC:
        case dai::model::ModelType::NNARCHIVE:
            break;
        case dai::model::ModelType::OTHER:
            setNNArchiveOther(nnArchive);
            break;
    }
}

void YoloSegmentationParser::setNNArchiveOther(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

std::shared_ptr<YoloSegmentationParser> YoloSegmentationParser::build(Node::Output& nnInput, const NNArchive& nnArchive) {
    setNNArchive(nnArchive);
    nnInput.link(input);
    return std::static_pointer_cast<YoloSegmentationParser>(shared_from_this());
}

void YoloSegmentationParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    archiveConfig = config;

    auto configV1 = config.getConfig<nn_archive::v1::Config>();

    const auto model = configV1.model;
    DAI_CHECK(model.heads, "Heads array is not defined in the NN Archive config file.");
    const auto head = (*model.heads)[0];
}

}  // namespace node
}  // namespace dai
