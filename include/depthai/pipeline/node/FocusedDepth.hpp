#pragma once

#include <depthai/common/optional.hpp>
#include <depthai/pipeline/DeviceNodeGroup.hpp>
#include <depthai/pipeline/Subnode.hpp>
#include <depthai/pipeline/node/Gate.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>
#include <depthai/pipeline/node/ImageManip.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/pipeline/node/host/FocusController.hpp>

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

    constexpr static const char* NAME = "FocusedDepth";

   private:
    bool built_ = false;

    // NOTE: the Subnodes must be declared before the Input&/Output& reference
    // members below, because the constructor initializes those references from
    // focusController. Members are initialized in declaration order, so
    // focusController has to exist first.
    Subnode<FocusController> focusController{*this, "focusController"};
    Subnode<Rectification> rectification{*this, "rectification"};
    Subnode<ImageManip> leftImageManip{*this, "leftImageManip"};
    Subnode<ImageManip> rightImageManip{*this, "rightImageManip"};
    Subnode<Gate> gateNeuralLeft{*this, "gateNeuralLeft"};
    Subnode<Gate> gateNeuralRight{*this, "gateNeuralRight"};
    Subnode<Gate> gateStereoLeft{*this, "gateStereoLeft"};
    Subnode<Gate> gateStereoRight{*this, "gateStereoRight"};
    Subnode<Gate> gateGpuLeft{*this, "gateGpuLeft"};
    Subnode<Gate> gateGpuRight{*this, "gateGpuRight"};
    Subnode<NeuralDepth> neuralDepth{*this, "neuralDepth"};
    Subnode<StereoDepth> stereoDepth{*this, "stereoDepth"};
    Subnode<GPUStereo> gpuStereo{*this, "gpuStereo"};

   public:
    Input& inputDetections;
    Output& depth;
    Output& confidence;
};

}  // namespace node
}  // namespace dai
