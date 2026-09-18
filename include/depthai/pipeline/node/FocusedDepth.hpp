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

    // Allocate only the configured tiers; a single-model pipeline needs one backend.
    std::vector<std::unique_ptr<Subnode<ImageManip>>> leftManips;
    std::vector<std::unique_ptr<Subnode<ImageManip>>> rightManips;
    std::vector<std::unique_ptr<Subnode<NeuralDepth>>> neuralDepths;

   public:
    Input& inputDetections;
    Output& depth;
    Output& confidence;
    Output& focusDebug;
};

}  // namespace node
}  // namespace dai
