#include "depthai/pipeline/node/TestIn.hpp"

#include "depthai/pipeline/Pipeline.hpp"

namespace dai {
namespace node {

void TestIn::setNumMessagesToSend(int num) {
    properties.numMessagesToSend = num;
}

}  // namespace node
}  // namespace dai
