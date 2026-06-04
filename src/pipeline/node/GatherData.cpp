#include "depthai/pipeline/node/GatherData.hpp"

namespace dai {
namespace node {

GatherData::GatherData(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, GatherData, GatherDataProperties>(std::move(props)) {}

}  // namespace node
}  // namespace dai
