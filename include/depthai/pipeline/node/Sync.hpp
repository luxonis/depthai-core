#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/SyncProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Sync node. Performs syncing between image frames
 */
class Sync : public NodeCRTP<DeviceNode, Sync, SyncProperties> {
   public:
    constexpr static const char* NAME = "Sync";
    using NodeCRTP::NodeCRTP;

   public:
    /**
     *  Inputs to Sync node. Can be accessed using subscript operator (Eg: inputs['in1'])
     *  By default inputs are set to blocking with queue size 8
     */
    InputMap inputs{true, *this, "inputs", Input(*this, "", Input::Type::SReceiver, {{DatatypeEnum::ImgFrame, false}})};

    /**
     * Outputs from Sync node. Can be accessed subscript operator (Eg: outputs['out1'])
     */
    OutputMap outputs{true, *this, "outputs", Output(*this, "", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}})};

    /**
     * Optional manual sync threshold.
     * If not specified default threshold is obtained as:
     * thresholdMS = 1000.f / (minimum FPS of input frames) / 2
     * Frame timestamp difference below this threshold are considered synced.
     * 0 is not recommended in real time system, as frame interrupts are received
     * at slightly different time, even with perfect hardware sync.
     * 0 can be used when replaying frames.
     */
    void setSyncThresholdMs(float thresholdMs);
};

}  // namespace node
}  // namespace dai
