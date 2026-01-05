//
// Created by thwdpc on 7/24/25.
//

#include "parsers/BaseParser.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai::node {

std::shared_ptr<BaseParser> BaseParser::build(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model) {
    DAI_CHECK_IN(head.parser == getName());
    buildImpl(head, model);
    return std::static_pointer_cast<BaseParser>(shared_from_this());
}
}  // namespace dai::node