#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/SyncProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Sync node. Performs syncing between images
 */
class Sync : public NodeCRTP<DeviceNode, Sync, SyncProperties> {
   public:
    constexpr static const char* NAME = "Sync";

   public:
    /**
     * Input image
     */
    Input input1{true, *this, "input1", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Input image
     */
    Input input2{true, *this, "input2", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Input image
     */
    Input input3{true, *this, "input3", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Output image
     */
    Output output1{true, *this, "output1", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Output image
     */
    Output output2{true, *this, "output2", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Output image
     */
    Output output3{true, *this, "output3", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
};

}  // namespace node
}  // namespace dai
