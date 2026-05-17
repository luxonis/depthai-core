#include "depthai/pipeline/node/GPUStereo.hpp"

#include <algorithm>
#include <stdexcept>

namespace dai {
namespace node {

namespace {

bool isLinked(dai::Node::Output& out, dai::Node::Input& in) {
    for(const auto& c : out.getConnections()) {
        if(c.in == &in) return true;
    }
    return false;
}

void ensureLinked(dai::Node::Output& out, dai::Node::Input& in) {
    if(!isLinked(out, in)) out.link(in);
}

void ensureUnlinked(dai::Node::Output& out, dai::Node::Input& in) {
    if(isLinked(out, in)) out.unlink(in);
}

}  // namespace

GPUStereo::Properties& GPUStereo::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

GPUStereo::GPUStereo(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, GPUStereo, GPUStereoProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

std::shared_ptr<GPUStereo> GPUStereo::build(Output& leftInput, Output& rightInput) {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftInput.link(left);
    rightInput.link(right);
#endif
    return std::static_pointer_cast<GPUStereo>(shared_from_this());
}

GPUStereo& GPUStereo::setRectification(bool enable) {
    rectification->enableRectification(enable);

    if(enable) {
        ensureUnlinked(messageDemux->outputs["left"], leftInternal);
        ensureUnlinked(messageDemux->outputs["right"], rightInternal);

        ensureLinked(messageDemux->outputs["left"], rectification->input1);
        ensureLinked(messageDemux->outputs["right"], rectification->input2);
        ensureLinked(rectification->output1, leftInternal);
        ensureLinked(rectification->output2, rightInternal);
    } else {
        ensureUnlinked(messageDemux->outputs["left"], rectification->input1);
        ensureUnlinked(messageDemux->outputs["right"], rectification->input2);
        ensureUnlinked(rectification->output1, leftInternal);
        ensureUnlinked(rectification->output2, rightInternal);

        ensureLinked(messageDemux->outputs["left"], leftInternal);
        ensureLinked(messageDemux->outputs["right"], rightInternal);
    }

    return *this;
}

GPUStereo& GPUStereo::setConfidenceThreshold(int threshold) {
    initialConfig->confidenceThreshold = std::clamp(threshold, 0, 255);
    return *this;
}

void GPUStereo::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("GPUStereo node is not supported on RVC2 devices.");
        }
    }

    sync->out.link(messageDemux->input);
    // Default to rectification enabled; setRectification(false) can rewire to bypass rectification.
    messageDemux->outputs["left"].link(rectification->input1);
    messageDemux->outputs["right"].link(rectification->input2);
    rectification->output1.link(leftInternal);
    rectification->output2.link(rightInternal);
}

}  // namespace node
}  // namespace dai
