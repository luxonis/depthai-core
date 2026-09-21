#pragma once

// Includes common classes needed to start the development

// IWYU pragma: begin_exports

// Include initialization utility
#include "utility/Initialization.hpp"

// Include experimental APIs
#ifdef DEPTHAI_HAVE_BETA
    #include "beta/datatypes.hpp"
    #include "beta/nodes.hpp"
#endif

// Include some common device classes
#include "device/CalibrationHandler.hpp"
#include "device/Device.hpp"
#include "device/DeviceBootloader.hpp"
#include "pipeline/InputQueue.hpp"
// Include Pipeline
#include "pipeline/Pipeline.hpp"

// Include common nodes
#include "pipeline/nodes.hpp"

// Include common datatypes
#include "pipeline/datatypes.hpp"

// OpenCV-specific utilities
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "utility/ColorizeDepthFrame.hpp"
#endif

// Model zoo
#include "modelzoo/Zoo.hpp"

// namespace dai {
// namespace{
// bool initializeForce = [](){initialize();};
// } // namespace
// }

// IWYU pragma: end_exports
