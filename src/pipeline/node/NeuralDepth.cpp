#include "depthai/pipeline/node/NeuralDepth.hpp"

#include <mutex>

#include "pipeline/node/NeuralNetwork.hpp"
#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

NeuralDepth::Properties& NeuralDepth::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

NeuralDepth::NeuralDepth(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, NeuralDepth, NeuralDepthProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

std::shared_ptr<NeuralDepth> NeuralDepth::build(Output& leftInput, Output& rightInput, DeviceModelZoo model) {
    leftInput.link(left);
    rightInput.link(right);
    // Set model
    neuralNetwork->setModelFromDeviceZoo(model);
    // Set rectification output size based on model
    rectification->setOutputSize(modelToInputSize[model].first, modelToInputSize[model].second);
    return std::static_pointer_cast<NeuralDepth>(shared_from_this());
}

void NeuralDepth::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("NeuralDepth node is not supported on RVC4 devices.");
        }

        if(device->isNeuralDepthSupported() == false) {
            throw std::runtime_error("NeuralDepth node is not supported on the connected device - please update LuxonisOS to 1.20.4 or higher.");
        }
    }
    auto defaultModel = DeviceModelZoo::NEURAL_DEPTH_SMALL;
    neuralNetwork->setModelFromDeviceZoo(defaultModel);
    rectification->setOutputSize(modelToInputSize[defaultModel].first, modelToInputSize[defaultModel].second);
    // Link sync outputs to message demux inputs
    sync->out.link(messageDemux->input);

    // Link message demux outputs to rectification inputs
    messageDemux->outputs["left"].link(rectification->input1);
    messageDemux->outputs["right"].link(rectification->input2);

    // Link left and right inputs to internal inputs
    rectification->output1.link(leftInternal);
    rectification->output2.link(rightInternal);

    // Link rectification outputs to neural network
    rectification->output1.link(neuralNetwork->inputs["left"]);
    rectification->output2.link(neuralNetwork->inputs["right"]);

    // Link neural network outputs to nnDataInput
    neuralNetwork->out.link(nnDataInput);
}

}  // namespace node
}  // namespace dai
