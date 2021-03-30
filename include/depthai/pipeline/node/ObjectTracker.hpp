#pragma once

#include <depthai/pipeline/Node.hpp>

#include "depthai/pipeline/datatype/Tracklets.hpp"

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/ObjectTrackerProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief ObjectTracker node. Perform object tracking.
 */
class ObjectTracker : public Node {
    using Properties = dai::ObjectTrackerProperties;

    std::string getName() const override;
    std::vector<Input> getInputs() override;
    std::vector<Output> getOutputs() override;
    nlohmann::json getProperties() override;
    std::shared_ptr<Node> clone() override;

    Properties properties;

   public:
    ObjectTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);

    /**
     * Input ImgFrame message on which object detection was performed.
     * Default queue is non-blocking with size 4.
     */
    Input inputFrame{*this, "inputFrame", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Input message with image detection from neural network.
     * Default queue is non-blocking with size 4.
     */
    Input inputDetections{*this, "inputDetections", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImgDetections, true}}};

    /**
     * Outputs Tracklets message that carries object tracking results.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::Tracklets, false}}};

    /**
     * Passthrough ImgFrame message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughFrame{*this, "passthroughFrame", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough image detections message from neural nework output.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDetections{*this, "passthroughDetections", Output::Type::MSender, {{DatatypeEnum::ImgDetections, true}}};

    /**
     * Specify tracker threshold.
     * @param threshold Above this threshold the detected objects will be tracked. Default 0, all image detections are tracked.
     */
    void setTrackerThreshold(float threshold);

    /**
     * Specify maximum number of object to track.
     * @param maxObjectsToTrack Maximum number of object to track. Maximum 60.
     */
    void setMaxObjectsToTrack(std::int32_t maxObjectsToTrack);

    /**
     * Specify detection labels to track.
     * @param labels Detection labels to track. Default every label is tracked from image detection network output.
     */
    void setDetectionLabelsToTrack(std::vector<std::uint32_t> labels);

    /**
     * Specify tracker type algorithm.
     * @param type Tracker type.
     */
    void setTrackerType(TrackType type);
};

}  // namespace node
}  // namespace dai
