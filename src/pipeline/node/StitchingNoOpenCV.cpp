#include <stdexcept>

#include "depthai/pipeline/node/host/Stitching.hpp"

namespace dai {
namespace node {

void Stitching::invalidateHostState() {}

void Stitching::run() {
    throw std::runtime_error("Stitching requires OpenCV support to run on the host. Enable OpenCV support or use setRunOnHost(false) with an RVC4 device.");
}

}  // namespace node
}  // namespace dai
