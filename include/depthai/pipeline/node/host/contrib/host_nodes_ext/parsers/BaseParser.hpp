//
// Created by thwdpc on 7/24/25.
//

#pragma once
#include "depthai/depthai.hpp"

namespace dai::node {
class BaseParser : public ThreadedHostNode {
   public:
    Input input{*this, {"in", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NNData, true}}}, true}};
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};
    virtual std::shared_ptr<BaseParser> build(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model);

   protected:
    virtual void buildImpl(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model) = 0;
};

/**
 * @brief Custom node for parser. When creating a custom parser, inherit from this class!
 * @tparam T Node type (same as the class you are creating)
 *
 * Example:
 * @code{.cpp}
 * class MyParser : public CustomParser<MyNode> {
 *     std::shared_ptr<Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
 *         auto frame = in->get<dai::ImgFrame>("data");
 *         // process frame
 *         // ...
 *         return nullptr; // Don't return anything, just process
 *     }
 * };
 * @endcode
 */
template <typename T>
using CustomParser = NodeCRTP<BaseParser, T>;
}  // namespace dai::node
