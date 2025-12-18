#pragma once

#include <depthai/modelzoo/Zoo.hpp>
#include <depthai/pipeline/DeviceNode.hpp>

// standard
#include <depthai/nn_archive/NNArchive.hpp>
#include <depthai/nn_archive/NNArchiveVersionedConfig.hpp>
#include <depthai/openvino/OpenVINO.hpp>
#include <depthai/properties/DetectionParserProperties.hpp>
#include <optional>
#include <string>

#include "depthai/common/YoloDecodingFamily.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace node {

/**
 * @brief DetectionParser node. Parses detection results from Mobilenet-SSD or YOLO neural networks.
 * @note If multiple detection heads are present in the NNArchive, only one type is supported (either YOLO or Mobilenet-SSD) and the last one will be used.
 */
class DetectionParser : public DeviceNodeCRTP<DeviceNode, DetectionParser, DetectionParserProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "DetectionParser";
    using DeviceNodeCRTP::DeviceNodeCRTP;

    ~DetectionParser() override;

    /**
     * @brief Build DetectionParser node. Connect output to this node's input. Also call setNNArchive() with provided NNArchive.
     * @param nnInput: Output to link
     * @param nnArchive: Neural network archive
     */
    std::shared_ptr<DetectionParser> build(Node::Output& nnInput, const NNArchive& nnArchive);
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
     * Load network xml and bin files into assets.
     * @param xmlModelPath Path to the neural network model file.
     */
    void setModelPath(const std::filesystem::path& modelPath);

    /**
     * Load network blob into assets and use once pipeline is started.
     *
     * @throws Error if file doesn't exist or isn't a valid network blob.
     * @param path Path to network blob
     */
    void setBlobPath(const std::filesystem::path& path);

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
    void setBlob(const std::filesystem::path& path);

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
     * Sets NN Family to parse. Possible values are:
     *
     * DetectionNetworkType::YOLO - 0
     * DetectionNetworkType::MOBILENET - 1
     *
     * @warning If NN Family is set manually, user must ensure that it matches the actual model being used.
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

    /**
     * Set number of classes. This will clear any previously set class names.
     * @param numClasses Number of classes
     */
    void setNumClasses(int numClasses);

    /**
     * Set class names. This will clear any previously set number of classes.
     * @param classes Vector of class names
     */
    void setClasses(const std::vector<std::string>& classes);

    /*
     * Sets the number of coordinates per bounding box.
     * @param coordinates Number of coordinates. Default is 4
     */
    void setCoordinateSize(int coordinates);

    /**
     * Set anchors for anchor-based yolo models
     * @param anchors Flattened vector of anchors
     * @warning This method is deprecated, use setAnchorsV2 instead.
     */
    void setAnchors(std::vector<float> anchors);

    /**
     * Set anchor masks for anchor-based yolo models
     * @param anchorMasks Map of anchor masks
     */
    void setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks);

    /**
     * Set anchors for anchor-based yolo models (v2)
     * @param anchors 3D vector of anchors [layer][anchor][dim]
     */
    void setAnchors(const std::vector<std::vector<std::vector<float>>>& anchors);

    /**
     * Set IOU threshold for non-maxima suppression
     * @param thresh IOU threshold
     */
    void setIouThreshold(float thresh);

    /**
     * Set subtype for the parser.
     * @param subtype Subtype string, currently supported subtypes are:
     * yolov6r1, yolov6r2 yolov8n, yolov6, yolov8, yolov10, yolov11, yolov3, yolov3-tiny, yolov5, yolov7, yolo-p, yolov5-u
     */
    void setSubtype(const std::string& subtype);

    /**
     * Enable/disable keypoints decoding. If enabled, number of keypoints must also be set.
     */
    void setDecodeKeypoints(bool decode);

    /**
     * Enable/disable segmentation mask decoding.
     */
    void setDecodeSegmentation(bool decode);

    /**
     * Set number of keypoints to decode. Automatically enables keypoints decoding.
     */
    void setNumKeypoints(int numKeypoints);

    /**
     * Set strides for yolo models
     */
    void setStrides(const std::vector<int>& strides);

    /**
     * Set edges connections between keypoints.
     * @param edges Vector edges connections represented as pairs of keypoint indices.
     * @note This is only applicable if keypoints decoding is enabled.
     */
    void setKeypointEdges(const std::vector<dai::Edge>& edges);

    /**
     * Get number of classes to decode.
     */
    int getNumClasses() const;

    /**
     * Get class names to decode.
     */
    std::optional<std::vector<std::string>> getClasses() const;

    /**
     * Get number of coordinates per bounding box.
     */
    int getCoordinateSize() const;

    /**
     * Get anchors for anchor-based yolo models
     */
    std::vector<float> getAnchors() const;

    /**
     * Get anchor masks for anchor-based yolo models
     */
    std::map<std::string, std::vector<int>> getAnchorMasks() const;

    /**
     * Get IOU threshold for non-maxima suppression
     */
    float getIouThreshold() const;

    /**
     * Get subtype for the parser.
     */
    std::string getSubtype() const;

    /**
     * Get whether keypoints decoding is enabled.
     */
    bool getDecodeKeypoints() const;

    /**
     * Get whether segmentation mask decoding is enabled.
     */
    bool getDecodeSegmentation() const;

    /**
     * Get number of keypoints to decode.
     */
    int getNKeypoints() const;

    /**
     * Get strides for yolo models
     */
    std::vector<int> getStrides() const;

    /**
     * Get NNArchive set for this node
     */
    const NNArchiveVersionedConfig& getNNArchiveVersionedConfig() const;

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

    void decodeMobilenet(dai::NNData& nnData, dai::ImgDetections& outDetections, float confidenceThr);

   private:
    bool runOnHostVar = false;
    void setNNArchiveBlob(const NNArchive& nnArchive);
    void setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves);
    void setNNArchiveOther(const NNArchive& nnArchive);
    void setConfig(const dai::NNArchiveVersionedConfig& config);
    YoloDecodingFamily yoloDecodingFamilyResolver(const std::string& subtype);
    bool decodeSegmentationResolver(const std::vector<std::string>& outputs);

    // host runnable requirements
    void buildStage1() override;
    void decodeYolo(dai::NNData& nnData, dai::ImgDetections& outDetections);
    std::vector<dai::TensorInfo> inTensorInfo;
    uint32_t imgWidth;
    uint32_t imgHeight;
    uint32_t imgSizesSet = false;
    //

    std::optional<NNArchive> mArchive;

    std::optional<NNArchiveVersionedConfig> archiveConfig;
};

}  // namespace node
}  // namespace dai
