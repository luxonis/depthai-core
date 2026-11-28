#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/node/NeuralNetwork.hpp"

#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/properties/NeuralNetworkProperties.hpp"
#include "depthai/utility/LockingQueue.hpp"

#include "DlContainer/IDlContainer.hpp"
#include "DlSystem/DlEnums.hpp"
#include "DlSystem/IUserBuffer.hpp"
#include "DlSystem/PlatformConfig.hpp"
#include "DlSystem/RuntimeList.hpp"
#include "DlSystem/UserBufferMap.hpp"
#include "SNPE/SNPE.hpp"
#include "SNPE/SNPEBuilder.hpp"
#include "SNPE/SNPEFactory.hpp"

#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {

class NeuralNetwork : public node::NeuralNetwork {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    constexpr static const char* NAME = node::NeuralNetwork::NAME;

   private:
    AtomicBool threadsRunning{true};

    std::vector<uint8_t> modelData;
    const uint8_t* modelDataView = nullptr;
    std::size_t modelDataViewSize = 0;
    std::shared_ptr<std::vector<uint8_t>> modelDataViewOwner;

   public:
    NeuralNetwork() : NeuralNetwork(std::make_unique<NeuralNetwork::Properties>()) {}
    NeuralNetwork(std::unique_ptr<Properties> props) : node::NeuralNetwork(std::move(props)) {}
    void buildStage1() override;
    bool runOnHost() const override {
        return true;
    }

    std::mutex commonTaskMtx;

    uint32_t lastTaskId = 0;
    std::mutex queueMtx;
    std::queue<uint32_t> taskQueue;

    std::condition_variable taskDoneCondVar;

    std::vector<std::string> outputNames;
    std::string outputNamesAppendix = "";

    std::filesystem::path modelPath;
    zdl::DlSystem::Runtime_t runtime = zdl::DlSystem::Runtime_t::DSP;
    zdl::DlSystem::PerformanceProfile_t performanceProfile = zdl::DlSystem::PerformanceProfile_t::DEFAULT;
    zdl::DlSystem::ExecutionPriorityHint_t executionPriorityHint = zdl::DlSystem::ExecutionPriorityHint_t::NORMAL;
    zdl::DlSystem::RuntimeList runtimeList = {};
    zdl::DlSystem::PlatformConfig platformConfig;
    std::unique_ptr<zdl::DlContainer::IDlContainer> container;
    std::vector<std::unique_ptr<zdl::SNPE::SNPE>> models;
    std::unordered_map<std::string, zdl::DlSystem::PerformanceProfile_t> fromStringToPerformanceProfile;
    std::unordered_map<std::string, zdl::DlSystem::ExecutionPriorityHint_t> fromStringToExecutionPriorityHint;

    void threadedRun(int threadId);
    void SNPEBackendBuild(const uint8_t* modelData, std::size_t modelDataSize);
    void SNPEBackendRun();
    void setModelData(const std::vector<uint8_t>& modelData);
    void setModelDataView(const uint8_t* data, std::size_t size, std::shared_ptr<std::vector<uint8_t>> owner);

    void run() override;

    void getModel(std::filesystem::path& modelPath);  // Writes the model to a temporary file; modelPath is set to it

    LockingQueue<std::shared_ptr<dai::ADatatype>> passthroughQueue;
    using InputVariant = std::variant<std::shared_ptr<dai::Buffer>, std::map<std::string, std::shared_ptr<dai::Buffer>>>;
    using ConversionBuffers = std::variant<std::vector<uint8_t>, std::map<std::string, std::vector<uint8_t>>>;
    bool getInputs(InputVariant& inputs);
    zdl::DlSystem::UserBufferMap createInputBufferMap(const InputVariant& inputs,
                                                      ConversionBuffers& conversionBuffers,
                                                      const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                      std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers);
    zdl::DlSystem::UserBufferMap createInputBufferMap(const std::shared_ptr<ImgFrame>& imgFrame,
                                                      ConversionBuffers& conversionBuffers,
                                                      const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                      std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers);
    zdl::DlSystem::UserBufferMap createInputBufferMap(const std::shared_ptr<dai::NNData>& nnData,
                                                      ConversionBuffers& conversionBuffers,
                                                      const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                      std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers);
    zdl::DlSystem::UserBufferMap createInputBufferMap(const std::shared_ptr<dai::Buffer>& buffer,
                                                      ConversionBuffers& conversionBuffers,
                                                      const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                      std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers);
    zdl::DlSystem::UserBufferMap createInputBufferMap(std::map<std::string, std::shared_ptr<dai::Buffer>>& inputs,
                                                      ConversionBuffers& conversionBuffers,
                                                      const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                      std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers);

    zdl::DlSystem::UserBufferMap createOutputBufferMap(std::shared_ptr<dai::NNData>& outputData,
                                                       const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                       std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers);
    void sendOutputs(InputVariant& inputs, std::shared_ptr<dai::NNData>& outputData);
    bool validateSettings();
    std::optional<dai::span<uint8_t>> prepareImgFrameForNN(const std::shared_ptr<ImgFrame>& input,
                                                           std::vector<uint8_t>& conversionBuffer,
                                                           const DlSystem::IBufferAttributes& bufferAtributes);
    bool validateTensorInfoWithSNPE(const dai::TensorInfo& tensorInfo, const zdl::DlSystem::IBufferAttributes& bufferAttributes);
    void logModelInfo(const std::unique_ptr<zdl::SNPE::SNPE>& model);

    static const std::unordered_map<dai::DeviceModelZoo, std::vector<std::string>> modelPaths;
    static std::string resolveModelPath(dai::DeviceModelZoo modelType);
    static bool isModelAvailable(dai::DeviceModelZoo modelType);
};

}  // namespace gate
}  // namespace dai
