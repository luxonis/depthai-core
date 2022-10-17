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

   protected:
    Properties& getProperties();

   private:
   public:
    Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    Sync(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Input image
     */
    Input input1{*this, "input1", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Input image
     */
    Input input2{*this, "input2", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Input image
     */
    Input input3{*this, "input3", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Output image
     */
    Output output1{*this, "output1", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Output image
     */
    Output output2{*this, "output2", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    /**
     * Output image
     */
    Output output3{*this, "output3", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
};

}  // namespace node
}  // namespace dai
