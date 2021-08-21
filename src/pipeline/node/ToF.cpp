#include "depthai/pipeline/node/ToF.hpp"

namespace dai {
namespace node {

ToF::ToF(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : Node(par, nodeId) {
    inputs = { &inputImage};
    outputs = {&out, &amp_out, &err_out};
}

  std::shared_ptr<Node> ToF::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
  }
  nlohmann::json ToF::getProperties() {
    nlohmann::json j;
    return j;   
  }
  
  
}
}
