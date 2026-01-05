//
// Created by thwdpc on 7/24/25.
//

#include "ParsingNeuralNetwork.hpp"

#include <iosfwd>
#include <utility/ErrorMacros.hpp>
#include <utility>
#include <variant>
#include <vector>
namespace dai::node {

std::shared_ptr<ParsingNeuralNetwork> ParsingNeuralNetwork::build(Output& input, const NNArchive& nnArchive) {
    nn = getParentPipeline().create<NeuralNetwork>();
    nn->build(input, nnArchive);
    updateParsers(nnArchive);
    return std::static_pointer_cast<ParsingNeuralNetwork>(shared_from_this());
}

std::shared_ptr<ParsingNeuralNetwork> ParsingNeuralNetwork::build(const std::shared_ptr<Camera>& input,
                                                                  NNModelDescription modelDesc,
                                                                  const std::optional<float> fps) {
    nn = getParentPipeline().create<NeuralNetwork>();
    nn->build(input, std::move(modelDesc), fps);
    try {
        const NNArchive& archive = nn->getNNArchive().value();
        updateParsers(archive);
    } catch(std::bad_optional_access& e) {
        std::cout << "NeuralNetwork node failed to create an archive and failed silently, getNNArchive returned std::nullopt: " << e.what() << std::endl;
    }
    return std::static_pointer_cast<ParsingNeuralNetwork>(shared_from_this());
}

std::shared_ptr<ParsingNeuralNetwork> ParsingNeuralNetwork::build(const std::shared_ptr<Camera>& input,
                                                                  const NNArchive& nnArchive,
                                                                  const std::optional<float> fps) {
    nn = getParentPipeline().create<NeuralNetwork>();
    nn->build(input, nnArchive, fps);
    updateParsers(nnArchive);
    return std::static_pointer_cast<ParsingNeuralNetwork>(shared_from_this());
}

// Updates parsers based on the provided NNArchive
void ParsingNeuralNetwork::updateParsers(const NNArchive& nnArchive) {
    removeOldParserNodes();
    parsers = getParserNodes(nnArchive);
}

// Removes previously created parser nodes and internal sync node from the pipeline
void ParsingNeuralNetwork::removeOldParserNodes() {
    for(const auto& entry : parsers) {
        std::visit([this](auto& p) { getParentPipeline().remove(p); }, entry);
    }
    if(parsers.size() > 1) {
        parserSync = std::nullopt;
    }
    parsers.clear();
}

void ParsingNeuralNetwork::run() {
    DAI_CHECK(nn != nullptr, "ParsingNeuralNetwork run before NN was initialized(was this node built via `build()`?)");
    DAI_CHECK(nn->getNNArchive() != std::nullopt, "ParsingNeuralNetwork run before fried NN was given archive(was this node built via `build()`?)");
}

// Creates new parser nodes from NNArchive, links their input/output, and returns them
std::vector<HostOrDeviceParser> ParsingNeuralNetwork::getParserNodes(const NNArchive& nnArchive) {
    std::vector<HostOrDeviceParser> newParsers = ParserGenerator::generateAllParsers(getParentPipeline(), nnArchive);

    if(auto& newParser = newParsers[0]; newParsers.size() == 1) {
        std::visit(
            [this](auto& p) {
                nn->out.link(p->input);
                out = p->out;
            },
            newParser);
    } else {
        auto sync = parserSync.value_or(Subnode<Sync>(*this, "sync"));
        for(std::size_t idx = 0; idx < newParsers.size(); ++idx) {
            std::visit(
                [this, idx, sync](auto& p) {
                    nn->out.link(p->input);
                    p->out.link(sync->inputs[std::to_string(idx)]);
                },
                parsers[idx]);
        }
        parserSync = sync;
        out = sync->out;
    }

    return newParsers;
}

}  // namespace dai::node
