#pragma once

#include <depthai/common/optional.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/ImageManip.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/host/FocusController.hpp>

#include <vector>

namespace dai {
namespace node {

class FocusedDepth : public DeviceNodeGroup {
   public:
    FocusedDepth();
    ~FocusedDepth() override = default;

    std::shared_ptr<FocusedDepth> build(Node::Output& left,
                                        Node::Output& right,
                                        std::optional<float> fps = std::nullopt,
                                        std::optional<std::pair<uint32_t, uint32_t>> resolution = std::nullopt);

    std::shared_ptr<FocusedDepth> setFocusModels(const std::vector<DeviceModelZoo>& models);
    std::shared_ptr<FocusedDepth> setFocusSelectionMode(FocusController::SelectionMode mode);
    std::shared_ptr<FocusedDepth> setFocusDispatchMode(FocusController::DispatchMode mode);

    constexpr static const char* NAME = "FocusedDepth";

   private:
    bool built_ = false;

    // NOTE: the Subnodes must be declared before the Input&/Output& reference
    // members below, because the constructor initializes those references from
    // focusController. Members are initialized in declaration order, so
    // focusController has to exist first.
    Subnode<FocusController> focusController{*this, "focusController"};
    Subnode<Rectification> rectification{*this, "rectification"};

    // One left/right crop ImageManip pair and one NeuralDepth backend per size tier
    // (FocusController::kNumTiers). Each backend has a fixed input size, so the left/right crops
    // reaching any one backend always match in size, and small crops use the fast small model
    // while large crops use a larger one. Crops are routed to a tier by FocusController::selectTier.
    Subnode<ImageManip> leftManip0{*this, "leftManip0"};
    Subnode<ImageManip> rightManip0{*this, "rightManip0"};
    Subnode<ImageManip> leftManip1{*this, "leftManip1"};
    Subnode<ImageManip> rightManip1{*this, "rightManip1"};
    Subnode<ImageManip> leftManip2{*this, "leftManip2"};
    Subnode<ImageManip> rightManip2{*this, "rightManip2"};
    Subnode<NeuralDepth> neuralDepth0{*this, "neuralDepth0"};
    Subnode<NeuralDepth> neuralDepth1{*this, "neuralDepth1"};
    Subnode<NeuralDepth> neuralDepth2{*this, "neuralDepth2"};

   public:
    Input& inputDetections;
    Output& depth;
    Output& confidence;
};

}  // namespace node
}  // namespace dai
