#include "depthai/pipeline/Assets.hpp"
#include "depthai/pipeline/FilterParams.hpp"
#include "depthai/pipeline/NodeConnectionSchema.hpp"
#include "depthai/pipeline/NodeIoInfo.hpp"
#include "depthai/pipeline/NodeObjInfo.hpp"
#include "depthai/pipeline/PipelineSchema.hpp"
#include "utility/serialization/SerializationDefine.hpp"

namespace dai {

DEPTHAI_SERIALIZE_DEFINE(Assets)
DEPTHAI_SERIALIZE_EXT_DEFINE(NodeConnectionSchema)
DEPTHAI_SERIALIZE_EXT_DEFINE(NodeIoInfo)
DEPTHAI_SERIALIZE_EXT_DEFINE(NodeObjInfo)
DEPTHAI_SERIALIZE_EXT_DEFINE(PipelineSchema)

}  // namespace dai

namespace dai {
namespace filters {
namespace params {

DEPTHAI_SERIALIZE_DEFINE(SpatialFilter)
DEPTHAI_SERIALIZE_DEFINE(TemporalFilter)
DEPTHAI_SERIALIZE_DEFINE(ThresholdFilter)
DEPTHAI_SERIALIZE_DEFINE(SpeckleFilter)

}  // namespace params
}  // namespace filters
}  // namespace dai
