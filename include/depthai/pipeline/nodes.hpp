#pragma once

// all the nodes
#include "node/AprilTag.hpp"
#include "node/BenchmarkIn.hpp"
#include "node/BenchmarkOut.hpp"
#include "node/Camera.hpp"
#include "node/ColorCamera.hpp"
#include "node/DetectionNetwork.hpp"
#include "node/DetectionParser.hpp"
#include "node/EdgeDetector.hpp"
#include "node/FeatureTracker.hpp"
#include "node/IMU.hpp"
#include "node/ImageManip.hpp"
#include "node/MessageDemux.hpp"
#include "node/MonoCamera.hpp"
#include "node/NeuralNetwork.hpp"
#include "node/ObjectTracker.hpp"
#include "node/PointCloud.hpp"
#include "node/Pool.hpp"
#include "node/SPIIn.hpp"
#include "node/SPIOut.hpp"
#include "node/Script.hpp"
#include "node/SpatialDetectionNetwork.hpp"
#include "node/SpatialLocationCalculator.hpp"
#include "node/StereoDepth.hpp"
#include "node/Sync.hpp"
#include "node/SystemLogger.hpp"
#include "node/ToF.hpp"
#include "node/UVC.hpp"
#include "node/VideoEncoder.hpp"
#include "node/Warp.hpp"
#include "node/XLinkIn.hpp"
#include "node/XLinkOut.hpp"
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "node/host/Display.hpp"
    #include "node/host/HostCamera.hpp"
    #include "node/host/Record.hpp"
    #include "node/host/Replay.hpp"
#endif
#include "ThreadedHostNode.hpp"
#include "node/host/HostNode.hpp"
#ifdef DEPTHAI_HAVE_RTABMAP_SUPPORT
#include "depthai/rtabmap/RTABMapSLAM.hpp"
#include "depthai/rtabmap/RTABMapVIO.hpp"
#endif
#ifdef DEPTHAI_HAVE_BASALT_SUPPORT
#include "depthai/basalt/BasaltVIO.hpp"
#endif