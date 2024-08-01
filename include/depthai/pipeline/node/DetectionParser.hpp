#pragma once

#include <depthai/modelzoo/NNModelDescription.hpp>
#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <fstream>

// shared
#include <depthai/nn_archive/NNArchive.hpp>
#include <depthai/nn_archive/NNArchiveConfig.hpp>
#include <depthai/openvino/OpenVINO.hpp>
#include <depthai/properties/DetectionParserProperties.hpp>
#include <optional>

namespace dai {
namespace node {

/**
 * @brief DetectionParser node. Parses detection results from different neural networks and is being used internally by MobileNetDetectionNetwork and
 * YoloDetectionNetwork.
 */
class DetectionParser : public DeviceNodeCRTP<DeviceNode, DetectionParser, DetectionParserProperties> {
   public:
    constexpr static const char* NAME = "DetectionParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    /**
     * Input NN results with detection data to parse
     * Default queue is blocking with size 5
     */
    Input input{*this, {"in", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::NNData, true}}}, true}};

    /**
     * Outputs image frame with detected edges
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgDetections, false}}}}};

    /**
     * Input for image that produced the detection - image size can be taken from here
     */
    Input imageIn{*this, {"imageIn", DEFAULT_GROUP, true, 5, {{{DatatypeEnum::ImgFrame, false}}}, true}};

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
     * @brief Set NNArchive for this Node. If the archive's type is SUPERBLOB, use default number of shaves.
     *
     * @param nnArchive: NNArchive to set
     */
    void setNNArchive(const NNArchive& nnArchive);

    /**
     * @brief Set NNArchive for this Node, throws if the archive's type is not SUPERBLOB
     *
     * @param nnArchive: NNArchive to set
     * @param numShaves: Number of shaves to use
     */
    void setNNArchive(const NNArchive& nnArchive, int numShaves);

    /**
     * @brief Download model from zoo and set it for this Node
     *
     * @param description: Model description to download
     * @param useCached: Use cached model if available
     */
    void setFromModelzoo(NNModelDescription description, bool useCached = true);

    /**
     * @brief Download model from zoo and set it for this node.
     *
     * @param description: Model description to download
     * @param numShaves: Number of shaves to use
     * @param useCached: Use cached model if available
     */
    void setFromModelzoo(NNModelDescription description, int numShaves, bool useCached = true);

    /**
     * Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlobPath(const dai::Path& path);

    /**
     * Retrieves some input tensor information from the blob
     *
     * @param blob OpenVINO blob to retrieve the information from
     */
    void setBlob(OpenVINO::Blob blob);

    /**
     * Same functionality as the setBlobPath(). Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlob(const dai::Path& path);

    /**
     * Set input image size
     *
     * This should only be used instead of setBlob, not besides it
     */
    void setInputImageSize(int width, int height);

    /*
     * Set preview output size, as a tuple<width, height>
     */
    void setInputImageSize(std::tuple<int, int> size);

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
    void setClasses(const std::vector<std::string>& classes);
    /// Set coordianate size
    void setCoordinateSize(int coordinates);
    /// Set anchors
    void setAnchors(std::vector<float> anchors);
    /// Set anchor masks
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);
    /// Set anchors with masks
    void setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors);
    /// Set Iou threshold
    void setIouThreshold(float thresh);

    /// Get num classes
    int getNumClasses() const;
    std::optional<std::vector<std::string>> getClasses() const;
    /// Get coordianate size
    int getCoordinateSize() const;
    /// Get anchors
    std::vector<float> getAnchors() const;
    /// Get anchor masks
    std::map<std::string, std::vector<int>> getAnchorMasks() const;
    /// Get Iou threshold
    float getIouThreshold() const;

    const NNArchiveConfig& getNNArchiveConfig() const;

   private:
    void setNNArchiveBlob(const NNArchive& nnArchive);
    void setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves);
    void setNNArchiveOther(const NNArchive& nnArchive);
    void setConfig(const dai::NNArchiveConfig& config);

    std::optional<std::vector<std::string>> mClasses;
    std::optional<NNArchive> mArchive;

    std::optional<NNArchiveConfig> archiveConfig;
};

}  // namespace node
}  // namespace dai
