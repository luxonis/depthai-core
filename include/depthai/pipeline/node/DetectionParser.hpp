#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai-shared/properties/DetectionParserProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief DetectionParser node. Performs edge detection using 3x3 Sobel filter
 */
class DetectionParser : public NodeCRTP<DeviceNode, DetectionParser, DetectionParserProperties> {
   public:
    constexpr static const char* NAME = "DetectionParser";
    using NodeCRTP::NodeCRTP;

    /**
     * Input NN results with detection data to parse
     * Default queue is blocking with size 5
     */
    Input input{true, *this, "in", Input::Type::SReceiver, true, 5, true, {{DatatypeEnum::NNData, true}}};

    /**
     * Outputs image frame with detected edges
     */
    Output out{true, *this, "out", Output::Type::MSender, {{DatatypeEnum::ImgDetections, false}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Returns number of frames in pool
     *
     */
    int getNumFramesPool();

    /**
     * Retrieves some input tensor information from the blob
     *
     * @param blob OpenVINO blob to retrieve the information from
     */
    void setBlob(const OpenVINO::Blob& blob);

    /**
     * Set input image size
     *
     * This should only be used instead of setBlob, not besides it
     * @param size Size of the input image (needed for yolo)
    */
   void setInputImgSize(std::vector<unsigned> size);

    /**
     * Sets NN Family to parse
     */
    void setNNFamily(DetectionNetworkType type);

    /**
     * Gets NN Family to parse
     */
    DetectionNetworkType getNNFamily();

    /**
     * Specifies confidence threshold at which to filter the rest of the detections.
     * @param thresh Detection confidence must be greater than specified threshold to be added to the list
     */
    void setConfidenceThreshold(float thresh);

    /**
     * Retrieves threshold at which to filter the rest of the detections.
     * @returns Detection confidence
     */
    float getConfidenceThreshold() const;

    /// Set num classes
    void setNumClasses(int numClasses);
    /// Set coordianate size
    void setCoordinateSize(int coordinates);
    /// Set anchors
    void setAnchors(std::vector<float> anchors);
    /// Set anchor masks
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);
    /// Set Iou threshold
    void setIouThreshold(float thresh);

    /// Get num classes
    int getNumClasses() const;
    /// Get coordianate size
    int getCoordinateSize() const;
    /// Get anchors
    std::vector<float> getAnchors() const;
    /// Get anchor masks
    std::map<std::string, std::vector<int>> getAnchorMasks() const;
    /// Get Iou threshold
    float getIouThreshold() const;
};

}  // namespace node
}  // namespace dai
