//
// Created by thwdpc on 7/24/25.
//

#pragma once
#include "depthai/depthai.hpp"

namespace dai::node {
class BaseParser : public CustomThreadedNode<BaseParser> {
    const char* getName() const override = 0;

   public:
    Input input{*this, {"in", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NNData, true}}}, true}};
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    virtual std::shared_ptr<BaseParser> build(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model);

   protected:
    virtual void buildImpl(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model) = 0;
};
}  // namespace dai::node
