#pragma once

#include "depthai/pipeline/Node.hpp"

namespace dai {
namespace node {

/**
 * @brief ToF node
 */
class ToF : public Node {
   public:

   private:

   public:
  std::string getName() const override { return "ToF"; }
  std::shared_ptr<Node> clone() override;
  nlohmann::json getProperties() override;
    /**
     * Constructs ToF node.
     */
    ToF(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

      Input inputImage{*this, "inputImage", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Outputs ImgFrame message that carries modified image.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
};

}  // namespace node
}  // namespace dai
