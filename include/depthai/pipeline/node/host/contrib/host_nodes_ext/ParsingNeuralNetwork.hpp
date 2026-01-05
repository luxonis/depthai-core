//
// Created by thwdpc on 7/24/25.
//
#pragma once
#include <variant>
#include <vector>

#include "../ParserGenerator.hpp"
#include "depthai/depthai.hpp"
#include "parsers/BaseParser.hpp"
#include "parsers/KeypointParser.hpp"

namespace dai::node {

class ParsingNeuralNetwork : public CustomThreadedNode<ParsingNeuralNetwork> {
   public:
    std::shared_ptr<ParsingNeuralNetwork> build(Output& input, const NNArchive& nnArchive);
    std::shared_ptr<ParsingNeuralNetwork> build(const std::shared_ptr<Camera>& input, NNModelDescription modelDesc, std::optional<float> fps = std::nullopt);
    std::shared_ptr<ParsingNeuralNetwork> build(const std::shared_ptr<Camera>& input, const NNArchive& nnArchive, std::optional<float> fps = std::nullopt);

    InputMap& inputs = nn->inputs;
    Input& input = nn->input;
    std::optional<std::reference_wrapper<Output>> out = std::nullopt;
    Output& passthrough = nn->passthrough;
    OutputMap& passthroughs = nn->passthroughs;

    template <typename T>
    std::optional<size_t> getIndexOfFirstParserOfType() const {
        const auto which = std::find_if(parsers.begin(), parsers.end(), [](const auto& p) {
            return std::visit([](auto& anyP) { return std::dynamic_pointer_cast<T>(anyP) != nullptr; }, p);;
        });
        return which == parsers.end() ? std::nullopt : static_cast<std::optional<size_t>>(std::distance(parsers.begin(), which));
    }

    void run() override;

   private:
    std::vector<HostOrDeviceParser> getParserNodes(const NNArchive& nnArchive);

    void updateParsers(const NNArchive& nnArchive);

    void removeOldParserNodes();
    std::shared_ptr<NeuralNetwork> nn;
    std::optional<Subnode<Sync>> parserSync = std::nullopt;

   protected:
    std::vector<HostOrDeviceParser> parsers;
};

}  // namespace dai::node
