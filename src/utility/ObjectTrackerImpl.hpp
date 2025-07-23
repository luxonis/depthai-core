#pragma once

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/properties/ObjectTrackerProperties.hpp"

namespace dai {
namespace impl {

class Tracker {
   public:
    virtual ~Tracker() = default;

    /**
     * Initialize the tracker with the first frame and detections.
     * @param frame The first frame to initialize the tracker.
     * @param detections The initial detections to track.
     */
    virtual void init(const ImgFrame& frame, const std::vector<ImgDetection>& detections) = 0;

    /**
     * Update the tracker with a new frame and detections. In one loop should only call either update or track, not both.
     * @param frame The new frame to update the tracker.
     * @param detections The new detections to track.
     */
    virtual void update(const ImgFrame& frame, const std::vector<ImgDetection>& detections) = 0;

    /**
     * Track objects in the current frame. In one loop should only call either update or track, not both.
     * @param frame The current frame to track objects in.
     */
    virtual void track(const ImgFrame& frame) = 0;

    /**
     * Get the current tracklets.
     * @return A vector of current tracklets.
     */
    virtual std::vector<Tracklet> getTracklets() const = 0;
};

class OCSTracker : public Tracker {
    class State;

    std::unique_ptr<State> state;
    int32_t maxObjectsToTrack;
    TrackerIdAssignmentPolicy trackerIdAssignmentPolicy;
    bool trackingPerClass;
    float occlusionRatioThreshold;
    uint32_t trackletMaxLifespan;
    uint32_t trackletBirthThreshold;

   public:
    OCSTracker(const ObjectTrackerProperties& properties);
    ~OCSTracker() override;
    void init(const ImgFrame& frame, const std::vector<ImgDetection>& detections) override;
    void update(const ImgFrame& frame, const std::vector<ImgDetection>& detections) override;
    void track(const ImgFrame& frame) override;
    std::vector<Tracklet> getTracklets() const override;
};

}  // namespace impl
}  // namespace dai
