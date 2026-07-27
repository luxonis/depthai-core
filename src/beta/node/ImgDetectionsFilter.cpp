#include "depthai/beta/node/ImgDetectionsFilter.hpp"

namespace dai {
namespace beta {
namespace node {

void ImgDetectionsFilter::run() {
    while(mainLoop()) {
        auto detections = input.get<ImgDetections>();
        output.send(detections);
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
