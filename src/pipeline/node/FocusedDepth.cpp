#include "depthai/pipeline/node/FocusedDepth.hpp"

#include "depthai/pipeline/datatype/GateControl.hpp"

namespace dai {
namespace node {

FocusedDepth::FocusedDepth()
    : DeviceNodeGroup(nullptr),
      inputDetections(focusController->inputs["inputDetections"]),
      depth(focusController->out),
      confidence(focusController->confidenceOut) {}

std::shared_ptr<FocusedDepth> FocusedDepth::build(Node::Output& left,
                                                  Node::Output& right,
                                                  std::optional<float> fps,
                                                  std::optional<std::pair<uint32_t, uint32_t>> resolution) {
    if(built_) {
        return std::static_pointer_cast<FocusedDepth>(shared_from_this());
    }
    built_ = true;

    // Synchronize detections with the left/right rectified streams.
    focusController->inputs["left"].setWaitForMessage(true);
    focusController->inputs["right"].setWaitForMessage(true);
    focusController->inputs["inputDetections"].setWaitForMessage(inputDetections.isConnected());

    // Rectification step for the focused sub-flow.
    left.link(rectification->input1);
    right.link(rectification->input2);
    if(resolution) {
        rectification->setOutputSize(resolution->first, resolution->second);
    }

    rectification->output1.link(focusController->inputs["left"]);
    rectification->output2.link(focusController->inputs["right"]);
    rectification->output1.link(leftImageManip->inputImage);
    rectification->output2.link(rightImageManip->inputImage);

    // Crop generation is driven by runtime config; the same image must be reused for sequential crops.
    leftImageManip->inputConfig.setWaitForMessage(true);
    rightImageManip->inputConfig.setWaitForMessage(true);
    leftImageManip->setMaxOutputFrameSize(8 * 1024 * 1024);
    rightImageManip->setMaxOutputFrameSize(8 * 1024 * 1024);
    leftImageManip->setNumFramesPool(4);
    rightImageManip->setNumFramesPool(4);

    focusController->leftConfig.link(leftImageManip->inputConfig);
    focusController->rightConfig.link(rightImageManip->inputConfig);

    // One ImageManip output feeds the inputs of all candidate gates; only one gate is opened per crop.
    leftImageManip->out.link(gateNeuralLeft->input);
    leftImageManip->out.link(gateStereoLeft->input);
    leftImageManip->out.link(gateGpuLeft->input);
    rightImageManip->out.link(gateNeuralRight->input);
    rightImageManip->out.link(gateStereoRight->input);
    rightImageManip->out.link(gateGpuRight->input);

    // Gate controls open both left and right gates for the selected backend.
    focusController->gateControlNeural.link(gateNeuralLeft->inputControl);
    focusController->gateControlNeural.link(gateNeuralRight->inputControl);
    focusController->gateControlStereo.link(gateStereoLeft->inputControl);
    focusController->gateControlStereo.link(gateStereoRight->inputControl);
    focusController->gateControlGpu.link(gateGpuLeft->inputControl);
    focusController->gateControlGpu.link(gateGpuRight->inputControl);

    // Gates start closed.
    gateNeuralLeft->initialConfig = GateControl::closeGate();
    gateNeuralRight->initialConfig = GateControl::closeGate();
    gateStereoLeft->initialConfig = GateControl::closeGate();
    gateStereoRight->initialConfig = GateControl::closeGate();
    gateGpuLeft->initialConfig = GateControl::closeGate();
    gateGpuRight->initialConfig = GateControl::closeGate();

    // Backend build() links the gate outputs to the backend inputs.

    neuralDepth->depth.link(focusController->depthCrop);
    neuralDepth->confidence.link(focusController->confidenceCrop);
    stereoDepth->depth.link(focusController->depthCrop);
    stereoDepth->confidenceMap.link(focusController->confidenceCrop);
    gpuStereo->depth.link(focusController->depthCrop);
    gpuStereo->confidenceMap.link(focusController->confidenceCrop);

    // Default focused backend on RVC4 is the small NeuralDepth model.
    neuralDepth->setRectification(false).build(gateNeuralLeft->output, gateNeuralRight->output, DeviceModelZoo::NEURAL_DEPTH_480X300);
    stereoDepth->setRectification(false);
    stereoDepth->setInputResolution(640, 400);
    stereoDepth->build(gateStereoLeft->output, gateStereoRight->output, StereoDepth::PresetMode::FAST_ACCURACY);
    gpuStereo->setRectification(false);
    gpuStereo->build(gateGpuLeft->output, gateGpuRight->output);

    focusController->setTargetFps(fps.value_or(30.0f));
    focusController->setNeuralModel(DeviceModelZoo::NEURAL_DEPTH_480X300);

    return std::static_pointer_cast<FocusedDepth>(shared_from_this());
}

}  // namespace node
}  // namespace dai
