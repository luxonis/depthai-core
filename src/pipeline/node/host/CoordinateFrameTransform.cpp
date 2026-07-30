#include "depthai/pipeline/node/host/CoordinateFrameTransform.hpp"

#include <fmt/format.h>

#include <chrono>
#include <optional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

namespace {

/// Complete an under-specified reference frame with the frame declared for the input.
CoordinateFrame completeFrame(const CoordinateFrame& current, const CoordinateFrame& declared) {
    CoordinateFrame completed = current;
    if(completed.deviceId.empty()) {
        completed.deviceId = declared.deviceId;
    }
    if(completed.socket == CameraBoardSocket::AUTO) {
        completed.socket = declared.socket;
    }
    return completed;
}

/// Frame of a device the rig already knows, i.e. the one the other cameras of that device are attached to.
std::optional<CoordinateFrame> rigFrameOf(const MultiDeviceCalibrationHandler& rig, const std::string& deviceId, const CoordinateFrame& target) {
    if(target.deviceId == deviceId) return target;
    for(const auto& frame : rig.getFrames()) {
        if(frame.deviceId == deviceId) return frame;
    }
    return std::nullopt;
}

}  // namespace

std::string CoordinateFrameTransform::inputName(size_t index) {
    return fmt::format("input{}", index);
}

std::string CoordinateFrameTransform::outputName(size_t index) {
    return fmt::format("output{}", index);
}

void CoordinateFrameTransform::buildInternal() {}

std::shared_ptr<CoordinateFrameTransform> CoordinateFrameTransform::build(size_t numInputs, const CoordinateFrame& target) {
    DAI_CHECK_V(this->numInputs == 0, "CoordinateFrameTransform node was already built");
    DAI_CHECK_V(numInputs >= 1, "CoordinateFrameTransform node needs at least one input, got {}", numInputs);

    for(size_t i = 0; i < numInputs; ++i) {
        auto& input = inputs[inputName(i)];
        input.setBlocking(false);
        input.setMaxSize(4);
        // Instantiate the matching output
        (void)outputs[outputName(i)];
    }
    this->numInputs = numInputs;
    setTarget(target);

    return std::static_pointer_cast<CoordinateFrameTransform>(shared_from_this());
}

std::shared_ptr<CoordinateFrameTransform> CoordinateFrameTransform::build(const std::vector<Node::Output*>& sources, const CoordinateFrame& target) {
    build(sources.size(), target);

    for(size_t i = 0; i < sources.size(); ++i) {
        DAI_CHECK_V(sources[i] != nullptr, "CoordinateFrameTransform source {} is null", i);
        sources[i]->link(inputs[inputName(i)]);
    }

    return std::static_pointer_cast<CoordinateFrameTransform>(shared_from_this());
}

size_t CoordinateFrameTransform::getNumInputs() const {
    return numInputs;
}

void CoordinateFrameTransform::setTarget(const CoordinateFrame& target) {
    DAI_CHECK_V(target.isQualified(), "CoordinateFrameTransform target frame must specify both a device id and a camera socket, got {}", toString(target));
    this->target = target;
}

CoordinateFrame CoordinateFrameTransform::getTarget() const {
    return target;
}

void CoordinateFrameTransform::setSourceFrame(size_t inputIndex, const CoordinateFrame& frame) {
    sourceFrames[inputIndex] = frame;
}

void CoordinateFrameTransform::setCalibration(const MultiDeviceCalibrationHandler& calibration) {
    this->calibration = std::make_shared<MultiDeviceCalibrationHandler>(calibration);
}

std::shared_ptr<const MultiDeviceCalibrationHandler> CoordinateFrameTransform::resolveCalibration() const {
    if(calibration) return calibration;
    return getParentPipeline().getMultiDeviceCalibration();
}

std::shared_ptr<const MultiDeviceCalibrationHandler> CoordinateFrameTransform::withDeviceCalibrations(const MultiDeviceCalibrationHandler& rig) const {
    // A rig only holds the transformations *between* devices; how the cameras of one device see each other is factory
    // calibrated and read from the device itself, so that a rig file stays valid whatever the devices are recalibrated to.
    auto augmented = std::make_shared<MultiDeviceCalibrationHandler>(rig);
    for(const auto& device : getParentPipeline().getAllAssignedDevices()) {
        const auto deviceId = device->getDeviceId();
        const auto anchor = rigFrameOf(*augmented, deviceId, target);
        if(!anchor.has_value()) continue;

        const auto deviceCalibration = device->getCalibration();
        for(const auto& camera : deviceCalibration.getEepromData().cameraData) {
            const CoordinateFrame frame{deviceId, camera.first};
            if(frame == *anchor || augmented->canTransform(frame, *anchor)) continue;

            RigEdge edge;
            edge.from = frame;
            edge.to = *anchor;
            edge.source = "device-calibration";
            try {
                edge.transform.setTransformationMatrix(deviceCalibration.getCameraExtrinsics(frame.socket, anchor->socket));
            } catch(const std::exception& e) {
                // Cameras the device is not calibrated against simply stay out of the rig
                pimpl->logger->debug("No calibration between {} and {}: {}", toString(frame), toString(*anchor), e.what());
                continue;
            }
            edge.transform.setReferenceFrame(*anchor);
            augmented->setEdge(edge);
            pimpl->logger->trace("Added {} -> {} from the calibration of the device", toString(frame), toString(*anchor));
        }
    }
    return augmented;
}

void CoordinateFrameTransform::run() {
    DAI_CHECK_V(numInputs > 0, "CoordinateFrameTransform node was not built, call build() with the sources to re-express");
    auto& logger = pimpl->logger;

    const auto configured = resolveCalibration();
    DAI_CHECK_V(configured != nullptr,
                "CoordinateFrameTransform requires a multi-device calibration. Set it with Pipeline::setMultiDeviceCalibration() or "
                "CoordinateFrameTransform::setCalibration().");
    const auto rig = withDeviceCalibrations(*configured);

    const auto reexpress = [this, &rig, &logger](size_t index, const std::shared_ptr<ADatatype>& message) {
        const auto declared = sourceFrames.find(index);

        if(auto frame = std::dynamic_pointer_cast<ImgFrame>(message)) {
            auto& transformation = frame->getTransformation();
            if(declared != sourceFrames.end()) {
                transformation.setReferenceFrame(completeFrame(transformation.getReferenceFrame(), declared->second));
            }
            rig->reexpress(transformation, target);
            return;
        }

        if(auto transformable = std::dynamic_pointer_cast<Transformable>(message)) {
            auto transformation = transformable->getTransformation();
            if(!transformation.has_value()) {
                logger->warn("Message on input{} carries no transformation, passing it through unchanged", index);
                return;
            }
            if(declared != sourceFrames.end()) {
                transformation->setReferenceFrame(completeFrame(transformation->getReferenceFrame(), declared->second));
            }
            rig->reexpress(*transformation, target);
            transformable->setTransformation(*transformation);
            return;
        }

        logger->warn("Message on input{} carries no transformation metadata, passing it through unchanged", index);
    };

    while(mainLoop()) {
        bool received = false;
        for(size_t i = 0; i < numInputs; ++i) {
            std::shared_ptr<ADatatype> message;
            {
                auto blockEvent = this->inputBlockEvent();
                message = inputs[inputName(i)].tryGet<ADatatype>();
            }
            if(message == nullptr) continue;
            received = true;

            reexpress(i, message);
            outputs[outputName(i)].send(message);
        }
        if(!received) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

}  // namespace node
}  // namespace dai
