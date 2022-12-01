#include "depthai/pipeline/node/DetectionNetworkSub.hpp"

#include <sstream>

#include "depthai-shared/common/DetectionNetworkType.hpp"
#include "openvino/BlobReader.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

void DetectionNetworkSub::build() {
    // Default confidence threshold
    detectionParser->properties.parser.confidenceThreshold = 0.5;
    neuralNetwork->out.link(detectionParser->input);
}

// -------------------------------------------------------------------
// Neural Network API
// -------------------------------------------------------------------
void DetectionNetworkSub::setBlobPath(const dai::Path& path){
    neuralNetwork->setBlobPath(path);
}

void DetectionNetworkSub::setBlob(OpenVINO::Blob blob){
    neuralNetwork->setBlob(blob);
}

void DetectionNetworkSub::setBlob(const dai::Path& path){
    neuralNetwork->setBlob(path);
}

void DetectionNetworkSub::setXmlModelPath(const dai::Path& xmlModelPath, const dai::Path& binModelPath){
    neuralNetwork->setXmlModelPath(xmlModelPath, binModelPath);
}

void DetectionNetworkSub::setNumPoolFrames(int numFrames){
    neuralNetwork->setNumPoolFrames(numFrames);
}

void DetectionNetworkSub::setNumInferenceThreads(int numThreads){
    neuralNetwork->setNumInferenceThreads(numThreads);
}

void DetectionNetworkSub::setNumNCEPerInferenceThread(int numNCEPerThread){
    neuralNetwork->setNumNCEPerInferenceThread(numNCEPerThread);
}

void DetectionNetworkSub::setNumShavesPerInferenceThread(int numShavesPerThread){
    neuralNetwork->setNumShavesPerInferenceThread(numShavesPerThread);
}

void DetectionNetworkSub::setBackend(std::string backend){
    neuralNetwork->setBackend(backend);
}

void DetectionNetworkSub::setCustomSettings(std::map<std::string, std::string> settings){
    neuralNetwork->setCustomSettings(settings);
}

int DetectionNetworkSub::getNumInferenceThreads(){
    return neuralNetwork->getNumInferenceThreads();
}


void DetectionNetworkSub::setConfidenceThreshold(float thresh) {
    detectionParser->setConfidenceThreshold(thresh);
}

float DetectionNetworkSub::getConfidenceThreshold() const {
    return detectionParser->getConfidenceThreshold();
}

//--------------------------------------------------------------------
// MobileNet
//--------------------------------------------------------------------
void MobileNetDetectionNetworkSub::build() {
    DetectionNetworkSub::build();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::MOBILENET;
}

//--------------------------------------------------------------------
// YOLO
//--------------------------------------------------------------------
void YoloDetectionNetworkSub::build() {
    DetectionNetworkSub::build();
    detectionParser->properties.parser.nnFamily = DetectionNetworkType::YOLO;
    detectionParser->properties.parser.iouThreshold = 0.5f;
}

void YoloDetectionNetworkSub::setNumClasses(const int numClasses) {
    detectionParser->setNumClasses(numClasses);
}

void YoloDetectionNetworkSub::setCoordinateSize(const int coordinates) {
    detectionParser->setCoordinateSize(coordinates);
}

void YoloDetectionNetworkSub::setAnchors(std::vector<float> anchors) {
    detectionParser->setAnchors(anchors);
}

void YoloDetectionNetworkSub::setAnchorMasks(std::map<std::string, std::vector<int>> anchorMasks) {
    detectionParser->setAnchorMasks(anchorMasks);
}

void YoloDetectionNetworkSub::setIouThreshold(float thresh) {
    detectionParser->setIouThreshold(thresh);
}

/// Get num classes
int YoloDetectionNetworkSub::getNumClasses() const {
    return detectionParser->getNumClasses();
}

/// Get coordianate size
int YoloDetectionNetworkSub::getCoordinateSize() const {
    return detectionParser->getCoordinateSize();
}

/// Get anchors
std::vector<float> YoloDetectionNetworkSub::getAnchors() const {
    return detectionParser->getAnchors();
}

/// Get anchor masks
std::map<std::string, std::vector<int>> YoloDetectionNetworkSub::getAnchorMasks() const {
    return detectionParser->getAnchorMasks();
}

/// Get Iou threshold
float YoloDetectionNetworkSub::getIouThreshold() const {
    return detectionParser->getIouThreshold();
}

}  // namespace node
}  // namespace dai
