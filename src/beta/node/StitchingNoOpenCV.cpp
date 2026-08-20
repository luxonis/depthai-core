#include <stdexcept>

#include "depthai/beta/node/Stitching.hpp"

namespace dai {
namespace beta {
namespace node {

void Stitching::initializeHostState() {}

void Stitching::run() {
    throw std::runtime_error("Stitching requires OpenCV support to run on the host. Enable OpenCV support or use setRunOnHost(false) with an RVC4 device.");
}

}  // namespace node
}  // namespace beta
}  // namespace dai
