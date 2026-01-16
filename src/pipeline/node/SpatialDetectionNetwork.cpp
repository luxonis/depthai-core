#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

#include "../../utility/ErrorMacros.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "nn_archive/NNArchive.hpp"
#include "openvino/OpenVINO.hpp"
#include "pipeline/DeviceNodeGroup.hpp"

namespace dai {
namespace node {

//--------------------------------------------------------------------
// Base Detection Network Class
//--------------------------------------------------------------------

SpatialDetectionNetwork::SpatialDetectionNetwork(const std::shared_ptr<Device>& device)
    : DeviceNodeGroup(device, std::make_unique<Properties>(), false),
      properties(static_cast<Properties&>(*propertiesHolder))
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
      ,
      input{neuralNetwork->input},
      outNetwork{neuralNetwork->out},
      passthrough{neuralNetwork->passthrough},
      inputDepth{spatialLocationCalculator->inputDepth},
      out{spatialLocationCalculator->outputDetections},
      passthroughDepth{spatialLocationCalculator->passthroughDepth}
#endif
{
    if(device) {
        auto platform = device->getPlatform();
        if(platform == Platform::RVC4) {
            if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
        }
    }
};

SpatialDetectionNetwork::SpatialDetectionNetwork(std::unique_ptr<Properties> props)
    : DeviceNodeGroup(std::move(props), true),
      properties(static_cast<Properties&>(*propertiesHolder))
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
      ,
      input{neuralNetwork->input},
      outNetwork{neuralNetwork->out},
      passthrough{neuralNetwork->passthrough},
      inputDepth{spatialLocationCalculator->inputDepth},
      out{spatialLocationCalculator->outputDetections},
      passthroughDepth{spatialLocationCalculator->passthroughDepth}
#endif
{
    auto device = getDevice();
    if(device) {
        auto platform = device->getPlatform();
        if(platform == Platform::RVC4) {
            if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
        }
    }
};

SpatialDetectionNetwork::SpatialDetectionNetwork(std::unique_ptr<Properties> props, bool confMode)
    : DeviceNodeGroup(std::move(props), confMode),
      properties(static_cast<Properties&>(*propertiesHolder))
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
      ,
      input{neuralNetwork->input},
      outNetwork{neuralNetwork->out},
      passthrough{neuralNetwork->passthrough},
      inputDepth{spatialLocationCalculator->inputDepth},
      out{spatialLocationCalculator->outputDetections},
      passthroughDepth{spatialLocationCalculator->passthroughDepth}
#endif
{
    auto device = getDevice();
    if(device) {
        auto platform = device->getPlatform();
        if(platform == Platform::RVC4) {
            if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
        }
    }
};

SpatialDetectionNetwork::SpatialDetectionNetwork(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool confMode)
    : DeviceNodeGroup(device, std::move(props), confMode),
      properties(static_cast<Properties&>(*propertiesHolder))
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
      ,
      input{neuralNetwork->input},
      outNetwork{neuralNetwork->out},
      passthrough{neuralNetwork->passthrough},
      inputDepth{spatialLocationCalculator->inputDepth},
      out{spatialLocationCalculator->outputDetections},
      passthroughDepth{spatialLocationCalculator->passthroughDepth}
#endif
{
    if(device) {
        auto platform = device->getPlatform();
        if(platform == Platform::RVC4) {
            if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
        }
    }
};

void SpatialDetectionNetwork::buildInternal() {
    // Default confidence threshold
    detectionParser->properties.parser.confidenceThreshold = 0.5;

    // Mirror SpatialDetectionNetwork properties onto the SpatialLocationCalculator initial config
    auto& initialConfig = *spatialLocationCalculator->initialConfig;
    initialConfig.setDepthThresholds(properties.depthThresholds.lowerThreshold, properties.depthThresholds.upperThreshold);
    initialConfig.setCalculationAlgorithm(properties.calculationAlgorithm);
    initialConfig.setStepSize(properties.stepSize);

    neuralNetwork->out.link(detectionParser->input);
    detectionParser->out.link(spatialLocationCalculator->inputDetections);

    // No "internal" buffering to keep interface similar to monolithic nodes
    detectionParser->input.setBlocking(true);
    detectionParser->input.setMaxSize(1);
    spatialLocationCalculator->inputDetections.setBlocking(true);
    spatialLocationCalculator->inputDetections.setMaxSize(1);
}

std::shared_ptr<SpatialDetectionNetwork> SpatialDetectionNetwork::build(const std::shared_ptr<Camera>& inputRgb,
                                                                        const DepthSource& depthSource,
                                                                        const Model& model,
                                                                        std::optional<float> fps,
                                                                        std::optional<dai::ImgResizeMode> resizeMode) {
    ImgFrameCapability cap;
    if(fps.has_value()) cap.fps.value = *fps;
    if(resizeMode.has_value()) cap.resizeMode = *resizeMode;
    cap.enableUndistortion = true;  // default for SpatialDetectionNetwork
    return build(inputRgb, depthSource, model, cap);
}

std::shared_ptr<SpatialDetectionNetwork> SpatialDetectionNetwork::build(const std::shared_ptr<Camera>& inputRgb,
                                                                        const DepthSource& depthSource,
                                                                        const Model& model,
                                                                        const ImgFrameCapability& capability) {
    auto cap = capability;
    if(!cap.enableUndistortion.has_value()) cap.enableUndistortion = true;
    neuralNetwork->build(inputRgb, model, cap);
    auto nnArchive = neuralNetwork->getNNArchive();
    DAI_CHECK(nnArchive.has_value(), "NeuralNetwork NNArchive is not set after build.");
    detectionParser->setNNArchive(nnArchive.value());
    alignDepth(depthSource, inputRgb);
    return std::static_pointer_cast<SpatialDetectionNetwork>(shared_from_this());
}

NNArchive SpatialDetectionNetwork::createNNArchive(NNModelDescription& modelDesc) {
    // Download model from zoo
    if(modelDesc.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set.");
        modelDesc.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(modelDesc);
    auto modelType = dai::model::readModelType(path);
    DAI_CHECK(modelType == dai::model::ModelType::NNARCHIVE,
              "Model from zoo is not NNArchive - it needs to be a NNArchive to use build(Camera, NNModelDescription, float) method");
    auto nnArchive = dai::NNArchive(path);
    return nnArchive;
}

void SpatialDetectionNetwork::alignDepth(const DepthSource& depthSource, const std::shared_ptr<Camera>& camera) {
    std::visit([this, &camera](const auto& source) { alignDepthImpl(source, camera); }, depthSource);
}

void SpatialDetectionNetwork::alignDepthImpl(const std::shared_ptr<StereoDepth>& stereo, const std::shared_ptr<Camera>& camera) {
    auto device = getDevice();
    if(device) {
        auto platform = device->getPlatform();
        switch(platform) {
            case Platform::RVC4:
            case Platform::RVC2:{
                if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
                Subnode<ImageAlign>& align = *depthAlign;
                stereo->depth.link(align->input);
                neuralNetwork->passthrough.link(align->inputAlignTo);
                align->outputAligned.link(spatialLocationCalculator->inputDepth);
                // No "internal" buffering
                spatialLocationCalculator->inputDepth.setBlocking(true);
                spatialLocationCalculator->inputDepth.setMaxSize(1);
                // } break;
                //     stereo->depth.link(spatialLocationCalculator->inputDepth);
                //     neuralNetwork->passthrough.link(stereo->inputAlignTo);
            }
            break;
            case Platform::RVC3:
            default:
                throw std::runtime_error("Unsupported platform");
                break;
        }

    } else {
        stereo->depth.link(spatialLocationCalculator->inputDepth);
        stereo->setDepthAlign(camera->getBoardSocket());
    }
}

void SpatialDetectionNetwork::alignDepthImpl(const std::shared_ptr<NeuralDepth>& neuralDepth, const std::shared_ptr<Camera>& camera) {
    (void)camera;  // make compiler happy
    auto device = getDevice();
    DAI_CHECK_V(device, "Device is not set.");
    DAI_CHECK_V(device->getPlatform() == Platform::RVC4, "NeuralDepth with SpatialDetectionNetwork is only supported on RVC4 platforms");
    if(!depthAlign) depthAlign = std::make_unique<Subnode<ImageAlign>>(*this, "depthAlign");
    Subnode<ImageAlign>& align = *depthAlign;
    neuralDepth->depth.link(align->input);
    neuralNetwork->passthrough.link(align->inputAlignTo);
    align->outputAligned.link(spatialLocationCalculator->inputDepth);
    // No "internal" buffering
    spatialLocationCalculator->inputDepth.setBlocking(true);
    spatialLocationCalculator->inputDepth.setMaxSize(1);
}

void SpatialDetectionNetwork::alignDepthImpl(const std::shared_ptr<ToF>& tof, const std::shared_ptr<Camera>& camera) {
    (void)camera;  // make compiler happy
    (void)tof;     // make compiler happy
    throw std::runtime_error("ToF with SpatialDetectionNetwork is not supported yet.");
}

// -------------------------------------------------------------------
// Neural Network API
// -------------------------------------------------------------------

void SpatialDetectionNetwork::setNNArchive(const NNArchive& nnArchive) {
    constexpr int DEFAULT_SUPERBLOB_NUM_SHAVES = 8;

    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::BLOB:
            setNNArchiveBlob(nnArchive);
            break;
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, DEFAULT_SUPERBLOB_NUM_SHAVES);
            break;
        case dai::model::ModelType::OTHER:
        case dai::model::ModelType::DLC:
            setNNArchiveOther(nnArchive);
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "Cannot set NNArchive inside NNArchive. %s: %s", __FILE__, __LINE__);
            break;
    }
}

void SpatialDetectionNetwork::setNNArchive(const NNArchive& nnArchive, int numShaves) {
    switch(nnArchive.getModelType()) {
        case dai::model::ModelType::SUPERBLOB:
            setNNArchiveSuperblob(nnArchive, numShaves);
            break;
        case dai::model::ModelType::BLOB:
        case dai::model::ModelType::OTHER:
        case dai::model::ModelType::DLC:
            DAI_CHECK_V(false, "NNArchive type is not SUPERBLOB. Use setNNArchive(const NNArchive& nnArchive) instead.");
            break;
        case dai::model::ModelType::NNARCHIVE:
            DAI_CHECK_V(false, "Cannot set NNArchive inside NNArchive. %s: %s", __FILE__, __LINE__);
            break;
    }
}

void SpatialDetectionNetwork::setFromModelZoo(NNModelDescription description, bool useCached) {
    // Download model from zoo
    if(description.platform.empty()) {
        DAI_CHECK(getDevice() != nullptr, "Device is not set. Use setDevice(...) first.");
        description.platform = getDevice()->getPlatformAsString();
    }
    auto path = getModelFromZoo(description, useCached);
    setModelPath(path);
}

void SpatialDetectionNetwork::setNNArchiveBlob(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::BLOB, "NNArchive type is not BLOB");
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive);
}

void SpatialDetectionNetwork::setNNArchiveSuperblob(const NNArchive& nnArchive, int numShaves) {
    DAI_CHECK_V(nnArchive.getModelType() == dai::model::ModelType::SUPERBLOB, "NNArchive type is not SUPERBLOB");
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive, numShaves);
}

void SpatialDetectionNetwork::setNNArchiveOther(const NNArchive& nnArchive) {
    DAI_CHECK_V(nnArchive.getModelType() == model::ModelType::DLC || nnArchive.getModelType() == model::ModelType::OTHER, "NNArchive type is not DLC or OTHER");
    detectionParser->setNNArchive(nnArchive);
    neuralNetwork->setNNArchive(nnArchive);
}

void SpatialDetectionNetwork::setBlobPath(const std::filesystem::path& path) {
    neuralNetwork->setBlobPath(path);
    detectionParser->setBlobPath(path);
}

void SpatialDetectionNetwork::setBlob(OpenVINO::Blob blob) {
    neuralNetwork->setBlob(blob);
    detectionParser->setBlob(blob);
}

void SpatialDetectionNetwork::setBlob(const std::filesystem::path& path) {
    neuralNetwork->setBlob(path);
    detectionParser->setBlob(path);
}

void SpatialDetectionNetwork::setModelPath(const std::filesystem::path& modelPath) {
    neuralNetwork->setModelPath(modelPath);
    detectionParser->setModelPath(modelPath);
}

void SpatialDetectionNetwork::setNumPoolFrames(int numFrames) {
    neuralNetwork->setNumPoolFrames(numFrames);
}

void SpatialDetectionNetwork::setNumInferenceThreads(int numThreads) {
    neuralNetwork->setNumInferenceThreads(numThreads);
}

void SpatialDetectionNetwork::setNumNCEPerInferenceThread(int numNCEPerThread) {
    neuralNetwork->setNumNCEPerInferenceThread(numNCEPerThread);
}

void SpatialDetectionNetwork::setNumShavesPerInferenceThread(int numShavesPerThread) {
    neuralNetwork->setNumShavesPerInferenceThread(numShavesPerThread);
}

void SpatialDetectionNetwork::setBackend(std::string backend) {
    neuralNetwork->setBackend(backend);
}

void SpatialDetectionNetwork::setBackendProperties(std::map<std::string, std::string> props) {
    neuralNetwork->setBackendProperties(props);
}

int SpatialDetectionNetwork::getNumInferenceThreads() {
    return neuralNetwork->getNumInferenceThreads();
}

void SpatialDetectionNetwork::setConfidenceThreshold(float thresh) {
    detectionParser->setConfidenceThreshold(thresh);
}

float SpatialDetectionNetwork::getConfidenceThreshold() const {
    return detectionParser->getConfidenceThreshold();
}

void SpatialDetectionNetwork::setBoundingBoxScaleFactor(float scaleFactor) {
    spatialLocationCalculator->initialConfig->setBoundingBoxScaleFactor(scaleFactor);
    properties.detectedBBScaleFactor = scaleFactor;
}

void SpatialDetectionNetwork::setDepthLowerThreshold(uint32_t lowerThreshold) {
    properties.depthThresholds.lowerThreshold = lowerThreshold;
    spatialLocationCalculator->initialConfig->setDepthThresholds(lowerThreshold);
}

void SpatialDetectionNetwork::setDepthUpperThreshold(uint32_t upperThreshold) {
    properties.depthThresholds.upperThreshold = upperThreshold;
    uint32_t currentLower = spatialLocationCalculator->initialConfig->getDepthThresholds().first;
    spatialLocationCalculator->initialConfig->setDepthThresholds(currentLower, upperThreshold);
}

void SpatialDetectionNetwork::setSpatialCalculationAlgorithm(dai::SpatialLocationCalculatorAlgorithm calculationAlgorithm) {
    properties.calculationAlgorithm = calculationAlgorithm;
    spatialLocationCalculator->initialConfig->setCalculationAlgorithm(calculationAlgorithm);
}

void SpatialDetectionNetwork::setSpatialCalculationStepSize(int stepSize) {
    properties.stepSize = stepSize;
    spatialLocationCalculator->initialConfig->setStepSize(stepSize);
}

std::optional<std::vector<std::string>> SpatialDetectionNetwork::getClasses() const {
    return detectionParser->getClasses();
}

}  // namespace node
}  // namespace dai
