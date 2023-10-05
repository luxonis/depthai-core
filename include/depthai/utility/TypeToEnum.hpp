#pragma once

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/AprilTagConfig.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"
#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"

namespace dai {

template <typename T>
DatatypeEnum rawToType() {
    return DatatypeEnum::MessageGroup;  // The only non implemented type
}
template <>
DatatypeEnum rawToType<Buffer>();
template <>
DatatypeEnum rawToType<ImgFrame>();
template <>
DatatypeEnum rawToType<NNData>();
template <>
DatatypeEnum rawToType<ImageManipConfig>();
template <>
DatatypeEnum rawToType<CameraControl>();
template <>
DatatypeEnum rawToType<ImgDetections>();
template <>
DatatypeEnum rawToType<SpatialImgDetections>();
template <>
DatatypeEnum rawToType<SystemInformation>();
template <>
DatatypeEnum rawToType<SpatialLocationCalculatorConfig>();
template <>
DatatypeEnum rawToType<SpatialLocationCalculatorData>();
template <>
DatatypeEnum rawToType<EdgeDetectorConfig>();
template <>
DatatypeEnum rawToType<AprilTagConfig>();
template <>
DatatypeEnum rawToType<AprilTags>();
template <>
DatatypeEnum rawToType<Tracklets>();
template <>
DatatypeEnum rawToType<IMUData>();
template <>
DatatypeEnum rawToType<StereoDepthConfig>();
template <>
DatatypeEnum rawToType<FeatureTrackerConfig>();
template <>
DatatypeEnum rawToType<ToFConfig>();
template <>
DatatypeEnum rawToType<TrackedFeatures>();

}  // namespace dai
