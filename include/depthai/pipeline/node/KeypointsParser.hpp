#pragma once

// project
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/properties/KeypointsParserProperties.hpp"
#include "depthai/pipeline/datatype/Keypoints.hpp"
#include "depthai/pipeline/datatype/creators/KeypointsCreator.hpp"

// shared
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"

#if defined(__clang__)
    #if __has_warning("-Wswitch-enum")
        #pragma clang diagnostic push
        #pragma clang diagnostic ignored "-Wswitch-enum"
    #endif
#elif defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wswitch-enum"
#elif defined(_MSC_VER)
    #pragma warning(push)
    #pragma warning(disable : 4061)
#endif
#ifdef DEPTHAI_XTENSOR_SUPPORT
    #include "xtensor/xarray.hpp"
    #include "xtensor/xmath.hpp"
#endif
#if defined(__clang__)
    #if __has_warning("-Wswitch-enum")
        #pragma clang diagnostic pop
    #endif
#elif defined(__GNUC__)
    #pragma GCC diagnostic pop
#elif defined(_MSC_VER)
    #pragma warning(pop)
#endif

namespace dai {
namespace node {

/**
 * @brief Keypoint detections parser node.
 */
class KeypointsParser : public DeviceNodeCRTP<DeviceNode, KeypointsParser, KeypointsParserProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   public:
    constexpr static const char* NAME = "KeypointsParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Input message with keypoint data from the neural network.
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::NNData, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs Keypoints message that carries the detected keypoints.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Keypoints, false}}}}};

    /**
     * Set the scale factor to divide the keypoints by.
     */
    void setScaleFactor(float scaleFactor);
    /**
     * Get the scale factor to divide the keypoints by.
     */
    float getScaleFactor() const;

    /**
     * Set the number of keypoints.
     */
    void setNumKeypoints(int numKeypoints);
    /**
     * Get the number of keypoints.
     */
    int getNumKeypoints() const;

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);
    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

};

}  // namespace node
}  // namespace dai
