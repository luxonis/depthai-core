#include "depthai/beta/datatype/ClassificationSequenceParserConfig.hpp"
#include "depthai/beta/datatype/Classifications.hpp"
#include "depthai/beta/datatype/Clusters.hpp"
#include "depthai/beta/datatype/FastSAMParserConfig.hpp"
#include "depthai/beta/datatype/HRNetParserConfig.hpp"
#include "depthai/beta/datatype/Keypoints.hpp"
#include "depthai/beta/datatype/Lines.hpp"
#include "depthai/beta/datatype/MLSDParserConfig.hpp"
#include "depthai/beta/datatype/MPPalmDetectionParserConfig.hpp"
#include "depthai/beta/datatype/Map2D.hpp"
#include "depthai/beta/datatype/MapOutputParserConfig.hpp"
#include "depthai/beta/datatype/MultiDeviceCalibrationResult.hpp"
#include "depthai/beta/datatype/PPTextDetectionParserConfig.hpp"
#include "depthai/beta/datatype/Predictions.hpp"
#include "depthai/beta/datatype/RFDETRParserConfig.hpp"
#include "depthai/beta/datatype/SCRFDParserConfig.hpp"
#include "depthai/beta/datatype/SuperAnimalParserConfig.hpp"
#include "depthai/beta/datatype/XFeatMonoParserConfig.hpp"
#include "depthai/beta/datatype/XFeatStereoParserConfig.hpp"
#include "depthai/beta/datatype/YuNetParserConfig.hpp"
#include "utility/serialization/SerializationDefine.hpp"

namespace dai {
namespace beta {

DEPTHAI_SERIALIZE_DEFINE(ClassificationSequenceParserConfig)
DEPTHAI_SERIALIZE_DEFINE(Classifications)
DEPTHAI_SERIALIZE_DEFINE(Cluster)
DEPTHAI_SERIALIZE_DEFINE(Clusters)
DEPTHAI_SERIALIZE_DEFINE(FastSAMParserConfig)
DEPTHAI_SERIALIZE_DEFINE(HRNetParserConfig)
DEPTHAI_SERIALIZE_DEFINE(Keypoints)
DEPTHAI_SERIALIZE_DEFINE(Line)
DEPTHAI_SERIALIZE_DEFINE(Lines)
DEPTHAI_SERIALIZE_DEFINE(MLSDParserConfig)
DEPTHAI_SERIALIZE_DEFINE(MPPalmDetectionParserConfig)
DEPTHAI_SERIALIZE_DEFINE(Map2D)
DEPTHAI_SERIALIZE_DEFINE(MapOutputParserConfig)
DEPTHAI_SERIALIZE_DEFINE(MultiDeviceCalibrationResult)
DEPTHAI_SERIALIZE_DEFINE(PPTextDetectionParserConfig)
DEPTHAI_SERIALIZE_DEFINE(Prediction)
DEPTHAI_SERIALIZE_DEFINE(Predictions)
DEPTHAI_SERIALIZE_DEFINE(RFDETRParserConfig)
DEPTHAI_SERIALIZE_DEFINE(SCRFDParserConfig)
DEPTHAI_SERIALIZE_DEFINE(SuperAnimalParserConfig)
DEPTHAI_SERIALIZE_DEFINE(XFeatMonoParserConfig)
DEPTHAI_SERIALIZE_DEFINE(XFeatStereoParserConfig)
DEPTHAI_SERIALIZE_DEFINE(YuNetParserConfig)

}  // namespace beta
}  // namespace dai
