//
// Created by thwdpc on 7/24/25.
//

#pragma once
#include <variant>

#include "depthai/depthai.hpp"
#include "host_nodes_ext/parsers/BaseParser.hpp"

namespace dai::node {

typedef std::variant<std::shared_ptr<BaseParser>, std::shared_ptr<DetectionParser>> HostOrDeviceParser;
template<typename V>
constexpr bool all_alternatives_shared_ptr = false;
template<typename... Ts>
constexpr bool all_alternatives_shared_ptr<std::variant<Ts...>> =
    (std::conjunction_v<std::is_same<std::shared_ptr<typename Ts::element_type>, Ts>...>);
static_assert(all_alternatives_shared_ptr<HostOrDeviceParser>, "All alternatives must be std::shared_ptr<T>");

struct ConfigModelWithHeads {
    nn_archive::v1::Model model;
    std::vector<nn_archive::v1::Head> heads;
};

class ParserGenerator {
   public:
    static std::vector<HostOrDeviceParser> generateAllParsers(Pipeline pipeline, const NNArchive& nnArchive, bool hostOnly = false);

   private:
    static ConfigModelWithHeads archiveGetModelEnsureOneHeadV1(const NNArchive& nnArchive, Platform targetPlatform);
    static HostOrDeviceParser generateOneV1Parser(
        Pipeline& pipeline, const NNArchive& owningArchive, const nn_archive::v1::Head& head, const nn_archive::v1::Model& model, bool hostOnly = false);
};
}  // namespace dai::node
