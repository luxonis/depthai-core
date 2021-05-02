#pragma once

// Includes common classes needed to start the development

// Include initialization utility
#include "utility/Initialization.hpp"

// Include some common device classes
#include "device/Device.hpp"
#include "device/DeviceBootloader.hpp"

// Include Pipeline
#include "pipeline/Pipeline.hpp"

// Include common nodes
#include "pipeline/node/ColorCamera.hpp"
#include "pipeline/node/DetectionNetwork.hpp"
#include "pipeline/node/ImageManip.hpp"
#include "pipeline/node/MonoCamera.hpp"
#include "pipeline/node/NeuralNetwork.hpp"
#include "pipeline/node/ObjectTracker.hpp"
#include "pipeline/node/SPIOut.hpp"
#include "pipeline/node/SpatialDetectionNetwork.hpp"
#include "pipeline/node/SpatialLocationCalculator.hpp"
#include "pipeline/node/StereoDepth.hpp"
#include "pipeline/node/SystemLogger.hpp"
#include "pipeline/node/VideoEncoder.hpp"
#include "pipeline/node/XLinkIn.hpp"
#include "pipeline/node/XLinkOut.hpp"

// Include common datatypes
#include "pipeline/datatype/Buffer.hpp"
#include "pipeline/datatype/CameraControl.hpp"
#include "pipeline/datatype/ImageManipConfig.hpp"
#include "pipeline/datatype/ImgDetections.hpp"
#include "pipeline/datatype/ImgFrame.hpp"
#include "pipeline/datatype/NNData.hpp"
#include "pipeline/datatype/SpatialImgDetections.hpp"
#include "pipeline/datatype/SpatialLocationCalculatorData.hpp"
#include "pipeline/datatype/SystemInformation.hpp"
#include "pipeline/datatype/Tracklets.hpp"

// namespace dai {
// namespace{
// bool initializeForce = [](){initialize();};
// } // namespace
// }
