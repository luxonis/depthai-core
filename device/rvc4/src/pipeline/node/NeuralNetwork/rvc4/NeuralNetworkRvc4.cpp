#include <fp16/fp16.h>

#include "../CreateUserBuffer.hpp"
#include "../NeuralNetwork.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "DlSystem/IBufferAttributes.hpp"
#include "DlSystem/IUserBuffer.hpp"
#include "DlSystem/IUserBufferFactory.hpp"
#include "DlSystem/UserBufferMap.hpp"
#include "SNPE/SNPEFactory.hpp"
#include "SNPE/SNPEUtil.h"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/utility/VectorMemory.hpp"
#include "pipeline/NodeTiming.hpp"
#include "spdlog/fmt/ostr.h"
#include "utilities/fmtUtils.hpp"

namespace {


constexpr const size_t SEED_LEN = 64;

uint8_t seed[SEED_LEN] = {
    0x6c,0xcd,0x4f,0x27,0x67,0x97,0xed,0x20,0x4c,0xc9,0xa2,0x03,0xd4,0x0f,0xa9,0xd6,
    0x7a,0xb4,0x4d,0xc7,0xda,0xaf,0xf4,0x65,0xec,0x8c,0xac,0x6e,0xe7,0xe5,0xcb,0xd8,
    0x72,0xa1,0x55,0x4a,0xa8,0x31,0xf7,0x9b,0x3f,0x76,0x15,0xab,0x48,0x12,0xa6,0x25,
    0x5d,0x04,0xa1,0xc2,0xba,0x35,0x59,0x1e,0xb1,0x13,0x76,0x10,0x80,0x62,0x42,0xd6
};

std::vector<uint8_t> xor_file_with_seed(const std::string& input_path) {
    std::ifstream in(input_path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open file: " + input_path);
    }

    in.seekg(0, std::ios::end);
    std::streamsize size = in.tellg();
    in.seekg(0, std::ios::beg);

    if (size < 0) {
        throw std::runtime_error("Invalid file size");
    }

    std::vector<uint8_t> data(static_cast<size_t>(size));
    if (!in.read(reinterpret_cast<char*>(data.data()), size)) {
        throw std::runtime_error("Failed to read file");
    }

    for(size_t i = 0; i < data.size(); ++i) {
        data[i] ^= seed[i % SEED_LEN];
    }

    if(0) {
        std::string tmpPath = "/tmp/model.dlc";
        std::ofstream debug_out(tmpPath, std::ios::binary);
        if (debug_out) {
            debug_out.write(reinterpret_cast<const char*>(data.data()), data.size());
            std::cout << "[DEBUG] Wrote " << data.size() << " bytes to " << tmpPath << "\n";
        }
    }

    return data;
}

struct LensUdoPackage {
    std::filesystem::path registration;
    std::filesystem::path implementation;
};

const std::unordered_map<std::string, LensUdoPackage> lensUdoPackagesByDlc = {
    {"/etc/models/luxonis/LENS_UDO_2XL_input_600x960_output_600x960_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_2XL_input_600x960_output_600x960_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_2XL_input_600x960_output_600x960_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
    {"/etc/models/luxonis/LENS_UDO_3XL_input_660x1056_output_660x1056_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_3XL_input_660x1056_output_660x1056_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_3XL_input_660x1056_output_660x1056_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
    {"/etc/models/luxonis/LENS_UDO_4XL_input_780x1248_output_780x1248_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_4XL_input_780x1248_output_780x1248_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_4XL_input_780x1248_output_780x1248_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
    {"/etc/models/luxonis/LENS_UDO_L_input_480x768_output_480x768_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_L_input_480x768_output_480x768_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_L_input_480x768_output_480x768_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
    {"/etc/models/luxonis/LENS_UDO_M_input_360x576_output_360x576_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_M_input_360x576_output_360x576_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_M_input_360x576_output_360x576_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
    {"/etc/models/luxonis/LENS_UDO_N_input_240x384_output_240x384_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_N_input_240x384_output_240x384_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_N_input_240x384_output_240x384_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
    {"/etc/models/luxonis/LENS_UDO_S_input_300x480_output_300x480_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_S_input_300x480_output_300x480_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_S_input_300x480_output_300x480_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
    {"/etc/models/luxonis/LENS_UDO_XL_input_540x864_output_540x864_v1_3_2410.dlc",
     {"/etc/models/luxonis/LENS_UDO_XL_input_540x864_output_540x864_v1_3_2410_libUdoExactLensPackageReg.so",
      "/etc/models/luxonis/LENS_UDO_XL_input_540x864_output_540x864_v1_3_2410_libUdoExactLensPackageImplDsp.so"}},
};  // Models that are not in this map register no UDOs

std::optional<LensUdoPackage> getLensUdoPackage(const std::string& modelPath) {
    const auto it = lensUdoPackagesByDlc.find(modelPath);
    if(it == lensUdoPackagesByDlc.end()) return std::nullopt;
    return it->second;
}

bool isDeviceModelInstalled(const std::string& modelPath) {
    if(!std::filesystem::is_regular_file(modelPath)) return false;
    const auto package = getLensUdoPackage(modelPath);
    return !package || (std::filesystem::is_regular_file(package->registration) && std::filesystem::is_regular_file(package->implementation));
}

void registerModelUdoPackage(const std::string& modelPath) {
    const auto package = getLensUdoPackage(modelPath);
    if(!package) return;

    for(const auto& path : {package->registration, package->implementation}) {
        if(!std::filesystem::is_regular_file(path)) {
            throw std::runtime_error("Missing LENS UDO library for " + modelPath + ": " + path.string());
        }
    }
    const auto registrationPath = package->registration.string();

    static std::mutex registrationMutex;  // Static: SNPE op packages are process-wide, also across NN nodes
    static std::unordered_set<std::string> registeredPaths;
    std::lock_guard<std::mutex> lock(registrationMutex);
    if(registeredPaths.count(registrationPath) != 0) return;

    if(!Snpe_Util_AddOpPackage(registrationPath.c_str())) {
        const char* error = Snpe_Util_GetLastError();
        throw std::runtime_error("Could not register UDO package " + registrationPath + ": " + (error ? error : "Unknown SNPE error"));
    }
    registeredPaths.insert(registrationPath);
}

}  // namespace

static void printInputInfoImage(const std::shared_ptr<dai::ADatatype>& inputTensor, std::stringstream& out) {
    auto imageFrame = std::dynamic_pointer_cast<dai::ImgFrame>(inputTensor);
    if(imageFrame == nullptr) {
        out << "Couldn't cast the tensor to an image.\n";
        return;
    }
    out << "Input image height: " << imageFrame->getHeight() << "\n";
    out << "Input image width: " << imageFrame->getWidth() << "\n";
}

std::unique_ptr<zdl::DlContainer::IDlContainer> loadContainerFromFile(std::string containerPath) {
    std::unique_ptr<zdl::DlContainer::IDlContainer> container;
    container = zdl::DlContainer::IDlContainer::open(zdl::DlSystem::String(containerPath.c_str()));
    return container;
}

std::unique_ptr<zdl::SNPE::SNPE> setBuilderOptions(std::unique_ptr<zdl::DlContainer::IDlContainer>& container,
                                                   zdl::DlSystem::Runtime_t runtime,
                                                   zdl::DlSystem::RuntimeList runtimeList,
                                                   zdl::DlSystem::PerformanceProfile_t performanceProfile,
                                                   zdl::DlSystem::ExecutionPriorityHint_t executionPriorityHint,
                                                   bool useUserSuppliedBuffers,
                                                   zdl::DlSystem::PlatformConfig platformConfig,
                                                   bool useCaching,
                                                   bool cpuFixedPointMode) {
    std::unique_ptr<zdl::SNPE::SNPE> snpe;
    zdl::SNPE::SNPEBuilder snpeBuilder(container.get());

    if(runtimeList.empty()) {
        runtimeList.add(runtime);
    }

    snpe = snpeBuilder.setUnconsumedTensorsAsOutputs(1)
               .setSingleThreadedInit(1)
               .setOutputLayers({})
               .setDebugMode(0)
               .setRuntimeProcessorOrder(runtimeList)
               .setPerformanceProfile(performanceProfile)
               .setExecutionPriorityHint(executionPriorityHint)
               .setUseUserSuppliedBuffers(useUserSuppliedBuffers)
               .setPlatformConfig(platformConfig)
               .setInitCacheMode(useCaching)
               .setCpuFixedPointMode(cpuFixedPointMode)
               .build();
    return snpe;
}

namespace dai {
namespace gate {

std::unordered_map<dai::DeviceModelZoo, std::vector<std::string>> const NeuralNetwork::modelPaths = {
    {dai::DeviceModelZoo::NEURAL_DEPTH_1248X780,
     {"/etc/models/luxonis/LENS_UDO_4XL_input_780x1248_output_780x1248_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_4XL_input_780x1248_output_780x1248_v1.2.2410.dlc",
      "/etc/models/luxonis/SMv4_3XL_input_780x1248_output_780x1248_v9_snpe_2320.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_1056X660,
     {"/etc/models/luxonis/LENS_UDO_3XL_input_660x1056_output_660x1056_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_3XL_input_660x1056_output_660x1056_v1.2.2410.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_960X600,
     {"/etc/models/luxonis/LENS_UDO_2XL_input_600x960_output_600x960_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_2XL_input_600x960_output_600x960_v1.2.2410.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_864X540,
     {"/etc/models/luxonis/LENS_UDO_XL_input_540x864_output_540x864_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_XL_input_540x864_output_540x864_v1.2.2410.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_768X480,
     {"/etc/models/luxonis/LENS_UDO_L_input_480x768_output_480x768_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_L_input_480x768_output_480x768_v1.2.2410.dlc",
      "/etc/models/luxonis/SMv4_L_input_480x768_output_480x768_v9_snpe_2320.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_576X360,
     {"/etc/models/luxonis/LENS_UDO_M_input_360x576_output_360x576_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_M_input_360x576_output_360x576_v1.2.2410.dlc",
      "/etc/models/luxonis/SMv4_M_input_360x576_output_360x576_v9_snpe_2320.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_480X300,
     {"/etc/models/luxonis/LENS_UDO_S_input_300x480_output_300x480_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_S_input_300x480_output_300x480_v1.2.2410.dlc",
      "/etc/models/luxonis/SMv4_S_input_300x480_output_300x480_v9_snpe_2320.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_384X240,
     {"/etc/models/luxonis/LENS_UDO_N_input_240x384_output_240x384_v1_3_2410.dlc",
      "/etc/models/luxonis/LENS_N_input_240x384_output_240x384_v1.2.2410.dlc",
      "/etc/models/luxonis/SMv4_N_input_240x384_output_240x384_v9_snpe_2320.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_288X180,
     {"/etc/models/luxonis/LENS_XN_input_180x288_output_180x288_v1.2.2410.dlc"}},
    {dai::DeviceModelZoo::NEURAL_DEPTH_192X120,
     {"/etc/models/luxonis/LENS_2XN_input_120x192_output_120x192_v1.2.2410.dlc"}},
    {dai::DeviceModelZoo::TOF_NEURAL_FUSION_672X804, {"TOF_NEURAL_FUSION_804x672_M.dlc"}},
};

std::string NeuralNetwork::resolveModelPath(dai::DeviceModelZoo modelType) {
    const auto& candidatePaths = NeuralNetwork::modelPaths.at(modelType);
    const auto pathIt = std::find_if(candidatePaths.begin(), candidatePaths.end(), [](const auto& path) {
        return isDeviceModelInstalled(path);
    });

    if(pathIt == candidatePaths.end()) {
        throw std::runtime_error("No model available for requested device model (DLC or required LENS UDO libraries missing).");
    }

    return *pathIt;
}

bool NeuralNetwork::isModelAvailable(dai::DeviceModelZoo modelType) {
    const auto it = NeuralNetwork::modelPaths.find(modelType);
    if(it == NeuralNetwork::modelPaths.end()) {
        return false;
    }

    return std::any_of(it->second.begin(), it->second.end(), [](const auto& path) {
        return isDeviceModelInstalled(path);
    });
}

void NeuralNetwork::logModelInfo(const std::unique_ptr<zdl::SNPE::SNPE>& model) {
    logger->info("Model info:");
    auto inputNames = model->getInputTensorNames();
    if(!inputNames) {
        logger->error("Couldn't get the input tensor names.");
        return;
    }
    auto outputNames = model->getOutputTensorNames();
    if(!outputNames) {
        logger->error("Couldn't get the output tensor names.");
        return;
    }
    logger->info("Input tensors: ");
    for(auto& name : *inputNames) {
        logger->info("    {}", name);
        logger->info("        Shape: ");
        auto bufferAttributesOp = model->getInputOutputBufferAttributes(name);
        if(!bufferAttributesOp) {
            throw std::runtime_error("Couldn't get the buffer attributes for input tensor.");
        }
        const auto& bufferAttributes = *bufferAttributesOp;
        auto dims = bufferAttributes->getDims();
        for(size_t i = 0; i < dims.rank(); i++) {
            logger->info("            {}", static_cast<int>(dims[i]));
        }
    }

    logger->info("Output tensors: ");
    for(auto& name : *outputNames) {
        logger->info("    {}", name);
        logger->info("        Shape: ");
        auto bufferAttributesOp = model->getInputOutputBufferAttributes(name);
        if(!bufferAttributesOp) {
            throw std::runtime_error("Couldn't get the buffer attributes for output tensor.");
        }
        const auto& bufferAttributes = *bufferAttributesOp;
        auto dims = bufferAttributes->getDims();
        for(size_t i = 0; i < dims.rank(); i++) {
            logger->info("            {}", static_cast<int>(dims[i]));
        }
    }
}

void NeuralNetwork::setModelData(const std::vector<uint8_t>& modelData) {
    this->modelData = modelData;
}

void NeuralNetwork::setModelDataView(const uint8_t* data, std::size_t size, std::shared_ptr<std::vector<uint8_t>> owner) {
    modelDataView = data;
    modelDataViewSize = size;
    modelDataViewOwner = std::move(owner);
}

void NeuralNetwork::buildStage1() {
    if(properties.backend.empty()) {
        properties.backend = "snpe";  // Default to SNPE
    }
    if(properties.backend != "snpe") {
        logger->error("Only snpe is supported");
        throw std::runtime_error("ERROR: Only snpe backend is supported");
    }
    fromStringToPerformanceProfile = {{"default", zdl::DlSystem::PerformanceProfile_t::DEFAULT},

                                      {"high_performance", zdl::DlSystem::PerformanceProfile_t::HIGH_PERFORMANCE},

                                      {"power_saver", zdl::DlSystem::PerformanceProfile_t::POWER_SAVER},

                                      {"system_settings", zdl::DlSystem::PerformanceProfile_t::SYSTEM_SETTINGS},

                                      {"sustained_high_performance", zdl::DlSystem::PerformanceProfile_t::SUSTAINED_HIGH_PERFORMANCE},

                                      {"burst", zdl::DlSystem::PerformanceProfile_t::BURST},

                                      {"low_power_saver", zdl::DlSystem::PerformanceProfile_t::LOW_POWER_SAVER},

                                      {"high_power_saver", zdl::DlSystem::PerformanceProfile_t::HIGH_POWER_SAVER},

                                      {"low_balanced", zdl::DlSystem::PerformanceProfile_t::LOW_BALANCED},

                                      {"extreme_power_saver", zdl::DlSystem::PerformanceProfile_t::EXTREME_POWER_SAVER}};
    fromStringToExecutionPriorityHint = {{"low", zdl::DlSystem::ExecutionPriorityHint_t::LOW},
                                         {"normal", zdl::DlSystem::ExecutionPriorityHint_t::NORMAL},
                                         {"normal_high", zdl::DlSystem::ExecutionPriorityHint_t::NORMAL_HIGH},
                                         {"high", zdl::DlSystem::ExecutionPriorityHint_t::HIGH}};
    if(!properties.deviceModel.has_value() && properties.modelUri.empty()) {
        logger->error("Either modelData or modelUri must be set.");
        throw std::runtime_error("Either modelData or modelUri must be set.");
    }

    if(properties.deviceModel.has_value() && !properties.modelUri.empty()) {
        logger->error("Only one of modelData or modelUri can be set.");
        throw std::runtime_error("Only one of modelData or modelUri can be set.");
    }

    std::string selectedModelPath;
    if(properties.deviceModel.has_value()) {
        auto modelType = properties.deviceModel.value();
        selectedModelPath = NeuralNetwork::resolveModelPath(modelType);
        auto decoded = xor_file_with_seed(selectedModelPath);

        setModelData(decoded);
    } else if(modelDataView == nullptr && !properties.modelUri.empty()) {
        modelData = moveResource(properties.modelUri);
    }
    if(modelDataView == nullptr && modelData.empty()) {
        logger->error("No model data loaded.");
        throw std::runtime_error("No model data loaded.");
    }
    registerModelUdoPackage(selectedModelPath);
    if(modelDataView != nullptr) {
        SNPEBackendBuild(modelDataView, modelDataViewSize);
    } else {
        SNPEBackendBuild(modelData.data(), modelData.size());
    }
    // TODO: Expose all this, think a bit more on the defaults, etc
    int numInferenceThreads = getNumInferenceThreads();
    if(numInferenceThreads < 1) {
        numInferenceThreads = 1;
    }

    for(int i = 0; i < numInferenceThreads; i++) {
        models.push_back(setBuilderOptions(container, runtime, runtimeList, performanceProfile, executionPriorityHint, true, platformConfig, false, false));
        if(!models.back()) {
            logger->error("Couldn't build the model.");
            return;
        }
    }

    modelData.clear();
    modelData.shrink_to_fit();
    modelDataView = nullptr;
    modelDataViewSize = 0;
    modelDataViewOwner.reset();

    logModelInfo(models.back());
}

void NeuralNetwork::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    auto success = validateSettings();
    if(!success) {
        logger->error("Error while validating settings, returning.");
        return;
    }
    SNPEBackendRun();
}

void NeuralNetwork::threadedRun(int threadId) {
    ConversionBuffers conversionBuffers;

    InputVariant inputs;
    if(input.isConnected()) {
        inputs = nullptr;  // Initialize the main input to nullptr
    } else {
        inputs = std::map<std::string, std::shared_ptr<dai::Buffer>>();  // Initialize the inputs to an empty map
    }

    while(isRunning() && threadsRunning) {
        auto tAbsoluteBeginning = std::chrono::steady_clock::now();
        this->pipelineEventDispatcher->endTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", threadId);
        this->pipelineEventDispatcher->startTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", threadId);
        bool skipInference = false;
        uint32_t taskId = 0;
        {
            std::lock_guard<std::mutex> uTaskMtx(commonTaskMtx);
            taskId = ++lastTaskId;  // Wrap-around is fine
            auto tBeforeGetInputs = std::chrono::steady_clock::now();
            bool success = false;
            {
                auto blockEvent = this->inputBlockEvent();
                success = getInputs(inputs);
            }
            auto tAfterGetInputs = std::chrono::steady_clock::now();
            logger->debug("Getting inputs took: {} ms",
                          std::chrono::duration_cast<std::chrono::microseconds>(tAfterGetInputs - tBeforeGetInputs).count() / 1000.0);
            if(!success) {
                logger->error("Error while getting NN inputs.");
                continue;
            }
            {
                std::lock_guard<std::mutex> l(queueMtx);
                taskQueue.push(taskId);
            }
        }
        auto tGotInput = std::chrono::steady_clock::now();
        auto nnData = std::make_shared<dai::NNData>();

        auto tBeforePrepareInputs = std::chrono::steady_clock::now();
        std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>> snpeUserBackedBuffers;
        auto inputBufferMap = createInputBufferMap(inputs, conversionBuffers, models[threadId], snpeUserBackedBuffers);
        auto tAfterPrepareInputs = std::chrono::steady_clock::now();
        auto outputBufferMap = createOutputBufferMap(nnData, models[threadId], snpeUserBackedBuffers);
        auto tAfterPrepareOutputs = std::chrono::steady_clock::now();
        logger->debug("Preparing inputs took: {} ms",
                      std::chrono::duration_cast<std::chrono::microseconds>(tAfterPrepareInputs - tBeforePrepareInputs).count() / 1000.0);
        logger->debug("Preparing outputs took: {} ms",
                      std::chrono::duration_cast<std::chrono::microseconds>(tAfterPrepareOutputs - tAfterPrepareInputs).count() / 1000.0);

        auto tBeforeExecute = std::chrono::steady_clock::now();
        auto execStatus = models[threadId]->execute(inputBufferMap, outputBufferMap);  // TODO(Morato) Using the C API should allow better error handling
        if(!execStatus) {
            logger->error("Error while executing the NN model.");
            skipInference = true;
        }
        auto tAfterExecute = std::chrono::steady_clock::now();
        logger->debug("NeuralNetwork inference took '{}' ms.",
                      std::chrono::duration_cast<std::chrono::microseconds>(tAfterExecute - tBeforeExecute).count() / 1000.0);

        auto tProcessed = std::chrono::steady_clock::now();

        {
            std::unique_lock<std::mutex> uLock(queueMtx);
            auto condition = [&]() { return !taskQueue.empty() && taskQueue.front() == taskId; };

            taskDoneCondVar.wait(uLock, condition);  // Outputs leave in input order: wait until this task is first in the queue

            if(!skipInference) {
                auto tBeforeSendOutputs = std::chrono::steady_clock::now();
                {
                    auto blockEvent = this->outputBlockEvent();
                    sendOutputs(inputs, nnData);
                }
                auto tAfterSendOutputs = std::chrono::steady_clock::now();
                logger->debug("Sending outputs took: {} ms",
                              std::chrono::duration_cast<std::chrono::microseconds>(tAfterSendOutputs - tBeforeSendOutputs).count() / 1000.0);
            }

            taskQueue.pop();
            taskDoneCondVar.notify_all();
        }
        auto tAbsoluteEnd = std::chrono::steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

bool NeuralNetwork::validateSettings() {
    if(input.isConnected() && inputs.size() > 0) {
        logger->error("Either the main input or the inputs should be connected, but not both, detected main input and {} inputs", inputs.size());
        return false;
    }

    if(!input.isConnected() && inputs.size() == 0) {
        logger->error("Either the main input or the inputs should be connected, but not neither is connected");
        return false;
    }

    if(input.isConnected()) {
        if(input.getReusePreviousMessage()) {
            logger->error("Reusing previous message is not supported when using the main input, set reusePreviousMessage on the consumer node instead.");
            return false;
        }
    }
    // TODO(Morato) More checks
    return true;
}

zdl::DlSystem::UserBufferMap NeuralNetwork::createInputBufferMap(const InputVariant& inputs,
                                                                 ConversionBuffers& conversionBuffers,
                                                                 const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                                 std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers) {
    if(std::holds_alternative<std::shared_ptr<dai::Buffer>>(inputs)) {
        auto input = std::get<std::shared_ptr<dai::Buffer>>(inputs);
        if(input == nullptr) {
            throw std::runtime_error("Main input is nullptr.");
        }
        auto imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(input);
        auto nnData = std::dynamic_pointer_cast<dai::NNData>(input);
        if(imgFrame) {
            return createInputBufferMap(imgFrame, conversionBuffers, model, snpeUserBackedBuffers);
        } else if(nnData) {
            return createInputBufferMap(nnData, conversionBuffers, model, snpeUserBackedBuffers);
        } else {
            return createInputBufferMap(input, conversionBuffers, model, snpeUserBackedBuffers);
        }
    } else {
        auto inputMap = std::get<std::map<std::string, std::shared_ptr<dai::Buffer>>>(inputs);
        return createInputBufferMap(inputMap, conversionBuffers, model, snpeUserBackedBuffers);
    }
}

static size_t calcSizeFromDims(const zdl::DlSystem::Dimension* dims, size_t rank, size_t elementSize) {
    if(rank == 0) return 0;
    size_t size = elementSize;
    while(rank--) {
        if(*dims == 0) {
            throw std::runtime_error("Error while creating user buffer.");
        }
        size *= *dims;
        dims++;
    }
    return size;
}

std::optional<dai::span<uint8_t>> NeuralNetwork::prepareImgFrameForNN(const std::shared_ptr<ImgFrame>& input,
                                                                      std::vector<uint8_t>& conversionBuffer,
                                                                      const DlSystem::IBufferAttributes& bufferAtributes) {
    auto modelInputRank = bufferAtributes.getDims().rank();
    if(modelInputRank < 3) {
        throw std::runtime_error("Only 4D input shape is supported.");
    }
    uint32_t width = bufferAtributes.getDims()[modelInputRank - 2];
    uint32_t height = bufferAtributes.getDims()[modelInputRank - 3];
    uint32_t channels = bufferAtributes.getDims()[modelInputRank - 1];

    if(width != input->getWidth() || height != input->getHeight()) {
        logger->error("Input image size doesn't match the model input size.");
        logger->error("Model input size: {}x{}x{}", width, height, channels);
        logger->error("Input image size: {}x{}", input->getWidth(), input->getHeight());
        throw std::runtime_error("Input image size doesn't match the model input size.");
    }

    auto daiType = input->getType();
    auto snpeType = bufferAtributes.getEncoding();

    using Type = ImgFrame::Type;
    auto supportedTypes = {Type::BGR888i, Type::RGB888i, Type::GRAY8, Type::RAW8};
    if(std::find(supportedTypes.begin(), supportedTypes.end(), daiType) == supportedTypes.end()) {
        logger->error("Input image type is not supported.");
        return std::nullopt;
    }

    auto elementType = snpeType->getElementType();
    using ElementType = DlSystem::UserBufferEncoding::ElementType_t;
    switch(elementType) {
        case ElementType::TF8: {
            auto userBufferEncodingTf8 = dynamic_cast<const DlSystem::UserBufferEncodingTf8*>(snpeType);
            if(userBufferEncodingTf8 == nullptr) {
                logger->error("Error while casting the user buffer encoding to TF8.");
                return std::nullopt;
            }
            auto stepExactly0 = userBufferEncodingTf8->getStepExactly0();
            auto quantizedStepSize = userBufferEncodingTf8->getQuantizedStepSize();
            bool stepExactly0CloseEnough = stepExactly0 <= 1;
            bool quantizedStepSizeCloseEnough = (quantizedStepSize >= (253/255.0f)) && (quantizedStepSize <= (257/255.0f));
            if(!stepExactly0CloseEnough || !quantizedStepSizeCloseEnough) {
                logger->warn("Custom TF8 encoding is not yet supported. DAI expects stepExactly0 to be 0 and quantizedStepSize to be 1.");
                logger->warn("stepExactly0: {}, quantizedStepSize: {}", stepExactly0, quantizedStepSize);
            }
            return input->getData();
        }
        case ElementType::TF16: {
            auto userBufferEncodingTf16 = dynamic_cast<const DlSystem::UserBufferEncodingTfN*>(snpeType);
            if(userBufferEncodingTf16 == nullptr) {
                logger->error("Error while casting the user buffer encoding to TF16.");
                return std::nullopt;
            }
            const auto stepExactly0 = userBufferEncodingTf16->getStepExactly0();
            const auto quantizedStepSize = userBufferEncodingTf16->getQuantizedStepSize();
            if(!std::isfinite(quantizedStepSize) || quantizedStepSize <= 0) {
                logger->error("TF16 quantizedStepSize must be finite and positive.");
                return std::nullopt;
            }

            const bool stepExactly0CloseEnough = stepExactly0 <= 1;
            const bool quantizedStepSizeCloseEnough = (quantizedStepSize >= (253 / 65535.0f)) && (quantizedStepSize <= (257 / 65535.0f));
            if(!stepExactly0CloseEnough || !quantizedStepSizeCloseEnough) {
                logger->warn("Custom TF16 encoding is not yet supported. DAI expects stepExactly0 to be 0 and quantizedStepSize to be 1.");
                logger->warn("stepExactly0: {}, quantizedStepSize: {}", stepExactly0, quantizedStepSize);
            }

            const auto dataSpan = input->getData();
            const auto tBeforeConversion = std::chrono::steady_clock::now();
            conversionBuffer.resize(dataSpan.size() * sizeof(uint16_t));
            auto* const conversionPtr = reinterpret_cast<uint16_t*>(conversionBuffer.data());  // SNPE takes native-endian TF16 words
            if(stepExactly0 == 0 && quantizedStepSize == 255 / 65535.0f) {
                for(size_t i = 0; i < dataSpan.size(); ++i) {
                    conversionPtr[i] = static_cast<uint16_t>(dataSpan[i] * 257u);
                }
            } else {
                for(size_t i = 0; i < dataSpan.size(); ++i) {
                    conversionPtr[i] = static_cast<uint16_t>(std::round((static_cast<float>(dataSpan[i]) / quantizedStepSize) + stepExactly0));
                }
            }
            const auto tAfterConversion = std::chrono::steady_clock::now();
            logger->debug("Converting the image to uint16 took: {} ms",
                          std::chrono::duration_cast<std::chrono::microseconds>(tAfterConversion - tBeforeConversion).count() / 1000.0);
            return conversionBuffer;
        }
        case ElementType::FLOAT16: {
            if(input->getBytesPerPixel() != 1 && input->getBytesPerPixel() != 3) {
                logger->error("Only 1 byte per (sub)pixel is supported as the NN input");
                return std::nullopt;
            }
            auto dataSpan = input->getData();
            auto tBeforeConversion = std::chrono::steady_clock::now();
            conversionBuffer.resize(dataSpan.size() * sizeof(uint16_t));
            auto conversionPtr = reinterpret_cast<uint16_t*>(conversionBuffer.data());
            auto numPixels = input->getData().size();
            for(size_t i = 0; i < numPixels; i++) {
                conversionPtr[i] = fp16_ieee_from_fp32_value(static_cast<float>(dataSpan[i]));
            }
            auto tAfterConversion = std::chrono::steady_clock::now();
            logger->debug("Converting the image to float16 took: {} ms",
                          std::chrono::duration_cast<std::chrono::microseconds>(tAfterConversion - tBeforeConversion).count() / 1000.0);
            return conversionBuffer;
        }

        case ElementType::FLOAT: {
            if(input->getBytesPerPixel() != 1 && input->getBytesPerPixel() != 3) {
                logger->error("Only 1 byte per (sub)pixel is supported as NN input");
                return std::nullopt;
            }
            auto dataSpan = input->getData();
            auto tBeforeConversion = std::chrono::steady_clock::now();
            conversionBuffer.resize(dataSpan.size() * sizeof(float));
            auto conversionPtr = reinterpret_cast<float*>(conversionBuffer.data());
            auto numPixels = input->getData().size();
            for(size_t i = 0; i < numPixels; i++) {
                conversionPtr[i] = static_cast<float>(dataSpan[i]);
            }
            auto tAfterConversion = std::chrono::steady_clock::now();
            logger->debug("Converting the image to float took: {} ms",
                          std::chrono::duration_cast<std::chrono::microseconds>(tAfterConversion - tBeforeConversion).count() / 1000.0);
            return conversionBuffer;
        }

        default:
            logger->error("Unsupported element type.");
            return std::nullopt;
    }
}

zdl::DlSystem::UserBufferMap NeuralNetwork::createInputBufferMap(const std::shared_ptr<ImgFrame>& input,
                                                                 ConversionBuffers& conversionBuffers,
                                                                 const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                                 std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers) {
    zdl::DlSystem::UserBufferMap inputBufferMap;
    auto inputNamesOp = model->getInputTensorNames();
    if(!inputNamesOp) {
        throw std::runtime_error("Couldn't get the input tensor names.");
    }
    const auto& inputNames = *inputNamesOp;
    if(inputNames.size() != 1) {
        throw std::runtime_error("Only single input model is supported when using ImgFrame as main input.");
    }
    const auto& inputName = inputNames.at(0);
    auto bufferAttributesOp = model->getInputOutputBufferAttributes(inputName);
    if(!bufferAttributesOp) {
        throw std::runtime_error("Couldn't get the buffer attributes for input tensor.");
    }
    const auto& bufferAttributes = *bufferAttributesOp;
    if(!std::holds_alternative<std::vector<uint8_t>>(conversionBuffers)) {
        conversionBuffers = std::vector<uint8_t>();
    }

    auto& conversionBuffer = std::get<std::vector<uint8_t>>(conversionBuffers);
    auto inputBuffer = prepareImgFrameForNN(input, conversionBuffer, *bufferAttributes);
    if(!inputBuffer) {
        logger->error("Error while preparing the input buffer.");
        throw std::runtime_error("Error while preparing the input buffer.");
    }

    auto requiredSize = calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());
    if(inputBuffer->size_bytes() < requiredSize) {
        logger->error("Input buffer size is smaller than the required size.");
        logger->error("Input buffer size: {}, required size: {}", inputBuffer->size_bytes(), requiredSize);
        throw std::runtime_error("Input buffer size is smaller than the required size.");
    }
    if(input->getType() != ImgFrame::Type::BGR888i) {
        throw std::runtime_error("Only BGR888i input is supported when using ImgFrame as main input.");
    }
    auto bufferShape = bufferAttributes->getDims();
    auto bufferElementSize = bufferAttributes->getElementSize();
    std::vector<size_t> strides(bufferShape.rank());
    strides[strides.size() - 1] = bufferElementSize;
    size_t stride = strides[strides.size() - 1];

    for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
        if(bufferShape[i] == 0) {
            throw std::runtime_error("Error while creating user buffer.");
        }
        stride *= bufferShape[i];
        strides[i - 1] = stride;
    }

    zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
    auto buffer = ubFactory.createUserBuffer(inputBuffer->data(), requiredSize, strides, bufferAttributes->getEncoding());
    inputBufferMap.add(inputName, buffer.get());
    snpeUserBackedBuffers.push_back(std::move(buffer));
    return inputBufferMap;
}

bool NeuralNetwork::validateTensorInfoWithSNPE(const dai::TensorInfo& tensorInfo, const zdl::DlSystem::IBufferAttributes& bufferAttributes) {
    auto dims = bufferAttributes.getDims();
    if(tensorInfo.dims.size() != dims.rank()) {
        logger->error("Rank of the tensor info doesn't match the rank of the buffer attributes.");
        logger->error("Tensor info rank: {}, buffer attributes rank: {}", tensorInfo.dims.size(), dims.rank());
        return false;
    }

    for(size_t i = 0; i < tensorInfo.dims.size(); i++) {
        if(tensorInfo.dims[i] != dims[i]) {
            logger->error("Dimension of the tensor info doesn't match the dimension of the buffer attributes.");
            logger->error("Tensor info dimension: {}, buffer attributes dimension: {}", static_cast<int>(tensorInfo.dims[i]), static_cast<int>(dims[i]));
            return false;
        }
    }

    auto elementType = bufferAttributes.getEncoding()->getElementType();
    using SNPEType = zdl::DlSystem::UserBufferEncoding::ElementType_t;
    using DAIType = dai::TensorInfo::DataType;
    switch(elementType) {
        case SNPEType::UINT8:
        case SNPEType::BOOL8:
        case SNPEType::UNSIGNED8BIT:
        case SNPEType::TF8: {
            if(tensorInfo.dataType != DAIType::U8F && tensorInfo.dataType != DAIType::I8) {
                logger->error("Tensor info type doesn't match the buffer attributes type.");
                logger->error("Tensor info type: {}, buffer attributes type: {}", tensorInfo.dataType, DAIType::U8F);
                return false;
            }
            break;
        }
        case SNPEType::FLOAT16: {
            if(tensorInfo.dataType != DAIType::FP16) {
                logger->error("Tensor info type doesn't match the buffer attributes type.");
                logger->error("Tensor info type: {}, buffer attributes type: {}", tensorInfo.dataType, DAIType::FP16);
                return false;
            }
            break;
        }
        case SNPEType::FLOAT: {
            if(tensorInfo.dataType != DAIType::FP32) {
                logger->error("Tensor info type doesn't match the buffer attributes type.");
                logger->error("Tensor info type: {}, buffer attributes type: {}", tensorInfo.dataType, DAIType::FP32);
                return false;
            }
            break;
        }
        case SNPEType::INT32: {
            if(tensorInfo.dataType != DAIType::INT) {
                logger->error("Tensor info type doesn't match the buffer attributes type.");
                logger->error("Tensor info type: {}, buffer attributes type: {}", tensorInfo.dataType, DAIType::INT);
                return false;
            }
            break;
        }
        case SNPEType::TF16: {
            if(tensorInfo.dataType != DAIType::U16F) {
                logger->error("Tensor info type doesn't match the buffer attributes type.");
                logger->error("Tensor info type: {}, buffer attributes type: {}", tensorInfo.dataType, DAIType::U16F);
                return false;
            }
            break;
        }
        default:
            logger->error("Unsupported element type in tensor {}", tensorInfo.name);
            return false;
    }
    return true;
}

zdl::DlSystem::UserBufferMap NeuralNetwork::createInputBufferMap(const std::shared_ptr<NNData>& input,
                                                                 ConversionBuffers& conversionBuffers,
                                                                 const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                                 std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers) {
    auto inputNamesOp = model->getInputTensorNames();
    if(!inputNamesOp) {
        throw std::runtime_error("Couldn't get the input tensor names.");
    }
    const auto& inputNames = *inputNamesOp;
    if(input->getAllLayerNames().size() == 1) {
        dai::TensorInfo tensorInfo;
        auto success = input->getLayer(input->getAllLayerNames()[0], tensorInfo);
        if(!success) {
            throw std::runtime_error("Couldn't get the layer from the NNData.");
        }

        if(inputNames.size() != 1) {
            throw std::runtime_error("Only single input model is supported when using NNData as main input.");
        }
        const auto& inputName = inputNames.at(0);
        auto bufferAttributesOp = model->getInputOutputBufferAttributes(inputName);
        if(!bufferAttributesOp) {
            throw std::runtime_error("Couldn't get the buffer attributes for input tensor.");
        }
        const auto& bufferAttributes = *bufferAttributesOp;
        if(!validateTensorInfoWithSNPE(tensorInfo, *bufferAttributes)) {
            throw std::runtime_error("Tensor info doesn't match the buffer attributes.");
        }
        auto bufferCast = std::dynamic_pointer_cast<dai::Buffer>(input);
        return createInputBufferMap(bufferCast, conversionBuffers, model, snpeUserBackedBuffers);
    }

    auto nnDataLayerNames = input->getAllLayerNames();
    if(nnDataLayerNames.size() != inputNames.size()) {
        auto error =
            fmt::format("Number of layers in the NNData doesn't match the number of input tensors in the model. NNData layers: {}, model input tensors: {}",
                        nnDataLayerNames.size(),
                        inputNames.size());
        throw std::runtime_error(error);
    }

    for(size_t i = 0; i < nnDataLayerNames.size(); i++) {
        if(std::find(inputNames.begin(), inputNames.end(), nnDataLayerNames[i]) == inputNames.end()) {
            auto error = fmt::format("Layer name from NNData is not in the model input tensor names. Layer name: {}", nnDataLayerNames[i]);
            throw std::runtime_error(error);
        }
    }

    zdl::DlSystem::UserBufferMap inputBufferMap;
    for(size_t i = 0; i < nnDataLayerNames.size(); i++) {
        auto layerName = nnDataLayerNames[i];
        auto layer = input->getTensorInfo(layerName);
        if(!layer) {
            auto error = fmt::format("Couldn't get the layer from the NNData. Layer name: {}", layerName);
            throw std::runtime_error(error);
        }
        auto bufferAttributesOp = model->getInputOutputBufferAttributes(layerName.c_str());
        if(!bufferAttributesOp) {
            throw std::runtime_error("Couldn't get the buffer attributes for input tensor.");
        }
        const auto& bufferAttributes = *bufferAttributesOp;
        if(!validateTensorInfoWithSNPE(*layer, *bufferAttributes)) {
            throw std::runtime_error("Tensor info doesn't match the buffer attributes.");
        }
        dai::span<uint8_t> tensorData = {input->getData().data() + layer->offset, layer->getTensorSize()};
        auto bufferShape = bufferAttributes->getDims();
        auto bufferElementSize = bufferAttributes->getElementSize();
        std::vector<size_t> strides(bufferShape.rank());
        strides[strides.size() - 1] = bufferElementSize;
        size_t stride = strides[strides.size() - 1];
        for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
            if(bufferShape[i] == 0) {
                throw std::runtime_error("Error while creating user buffer.");
            }
            stride *= bufferShape[i];
            strides[i - 1] = stride;
        }
        auto requiredSize =
            calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());

        if(tensorData.size_bytes() < requiredSize) {
            auto error = fmt::format(
                "Input buffer size is smaller than the required size. Input buffer size: {}, required size: {}", tensorData.size_bytes(), requiredSize);
            throw std::runtime_error(error);
        }
        zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
        auto buffer = ubFactory.createUserBuffer(tensorData.data(), requiredSize, strides, bufferAttributes->getEncoding());

        inputBufferMap.add(layerName.c_str(), buffer.get());
        snpeUserBackedBuffers.push_back(std::move(buffer));
    }
    return inputBufferMap;
}

zdl::DlSystem::UserBufferMap NeuralNetwork::createInputBufferMap(const std::shared_ptr<Buffer>& input,
                                                                 ConversionBuffers& conversionBuffers,
                                                                 const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                                 std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers) {
    auto inputNamesOp = model->getInputTensorNames();
    if(!inputNamesOp) {
        throw std::runtime_error("Couldn't get the input tensor names.");
    }
    const auto& inputNames = *inputNamesOp;
    if(inputNames.size() != 1) {
        auto error = fmt::format("Only single input model is supported when using Buffer as main input, detected {} inputs.", inputNames.size());
        throw std::runtime_error(error);
    }
    const auto& inputName = inputNames.at(0);
    auto bufferAttributesOp = model->getInputOutputBufferAttributes(inputName);
    if(!bufferAttributesOp) {
        throw std::runtime_error("Couldn't get the buffer attributes for input tensor.");
    }
    const auto& bufferAttributes = *bufferAttributesOp;
    auto requiredSize = calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());
    if(input->getData().size() < requiredSize) {
        auto error =
            fmt::format("Input buffer size is smaller than the required size. Input buffer size: {}, required size: {}", input->getData().size(), requiredSize);
        throw std::runtime_error(error);
    }
    auto bufferShape = bufferAttributes->getDims();
    auto bufferElementSize = bufferAttributes->getElementSize();
    std::vector<size_t> strides(bufferShape.rank());
    strides[strides.size() - 1] = bufferElementSize;
    size_t stride = strides[strides.size() - 1];
    for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
        if(bufferShape[i] == 0) {
            throw std::runtime_error("Error while creating user buffer.");
        }
        stride *= bufferShape[i];
        strides[i - 1] = stride;
    }
    zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
    auto buffer = ubFactory.createUserBuffer(input->getData().data(), requiredSize, strides, bufferAttributes->getEncoding());

    zdl::DlSystem::UserBufferMap inputBufferMap;
    inputBufferMap.add(inputName, buffer.get());
    snpeUserBackedBuffers.push_back(std::move(buffer));
    return inputBufferMap;
}

zdl::DlSystem::UserBufferMap NeuralNetwork::createInputBufferMap(std::map<std::string, std::shared_ptr<dai::Buffer>>& inputs,
                                                                 ConversionBuffers& conversionBuffers,
                                                                 const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                                 std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers) {
    zdl::DlSystem::UserBufferMap inputBufferMap;

    auto inputNamesOp = model->getInputTensorNames();
    if(!inputNamesOp) {
        throw std::runtime_error("Couldn't get the input tensor names.");
    }
    const auto& inputNames = *inputNamesOp;
    if(inputs.size() != inputNames.size()) {
        auto error = fmt::format(
            "Number of inputs doesn't match the number of input tensors in the model. Inputs: {}, model input tensors: {}", inputs.size(), inputNames.size());
        throw std::runtime_error(error);
    }

    for(auto& inputName : inputNames) {
        if(inputs.find(inputName) == inputs.end()) {
            auto error = fmt::format("Input tensor name from the model is not in the inputs. Input tensor name: {}", inputName);
            throw std::runtime_error(error);
        }
    }

    for(auto& [inputName, input] : inputs) {
        auto bufferAttributesOp = model->getInputOutputBufferAttributes(inputName.c_str());
        if(!bufferAttributesOp) {
            throw std::runtime_error("Couldn't get the buffer attributes for input tensor.");
        }
        const auto& bufferAttributes = *bufferAttributesOp;
        if(std::dynamic_pointer_cast<dai::ImgFrame>(input)) {
            auto imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(input);
            // Error checking
            // TODO - Validate the input
            if(!std::holds_alternative<std::map<std::string, std::vector<uint8_t>>>(conversionBuffers)) {
                conversionBuffers = std::map<std::string, std::vector<uint8_t>>();
            }

            auto& conversionBuffer = std::get<std::map<std::string, std::vector<uint8_t>>>(conversionBuffers);
            auto inputBuffer = prepareImgFrameForNN(imgFrame, conversionBuffer[inputName], *bufferAttributes);
            if(!inputBuffer) {
                throw std::runtime_error("Error while preparing the input buffer.");
            }

            auto requiredSize =
                calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());
            if(inputBuffer->size_bytes() < requiredSize) {
                auto error = fmt::format(
                    "Input buffer size is smaller than the required size. Input buffer size: {}, required size: {}", inputBuffer->size_bytes(), requiredSize);
                throw std::runtime_error(error);
            }
            auto bufferShape = bufferAttributes->getDims();
            auto bufferElementSize = bufferAttributes->getElementSize();
            std::vector<size_t> strides(bufferShape.rank());
            strides[strides.size() - 1] = bufferElementSize;
            size_t stride = strides[strides.size() - 1];
            for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
                if(bufferShape[i] == 0) {
                    throw std::runtime_error("Error while creating user buffer.");
                }
                stride *= bufferShape[i];
                strides[i - 1] = stride;
            }
            zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
            auto buffer = ubFactory.createUserBuffer(inputBuffer->data(), requiredSize, strides, bufferAttributes->getEncoding());
            inputBufferMap.add(inputName.c_str(), buffer.get());
            snpeUserBackedBuffers.push_back(std::move(buffer));
        } else if(std::dynamic_pointer_cast<dai::NNData>(input)) {
            auto nnData = std::dynamic_pointer_cast<dai::NNData>(input);
            if(nnData->getAllLayerNames().size() != 1) {
                auto error = fmt::format("Only single layer NNData is supported, detected {} layers on input {}", nnData->getAllLayerNames().size(), inputName);
                throw std::runtime_error(error);
            }
            dai::TensorInfo tensorInfo;
            auto success = nnData->getLayer(nnData->getAllLayerNames()[0], tensorInfo);
            if(!success) {
                throw std::runtime_error("Couldn't get the layer from the NNData.");
            }
            if(!validateTensorInfoWithSNPE(tensorInfo, *bufferAttributes)) {
                throw std::runtime_error("Tensor info doesn't match the buffer attributes.");
            }
            auto bufferShape = bufferAttributes->getDims();
            auto bufferElementSize = bufferAttributes->getElementSize();
            std::vector<size_t> strides(bufferShape.rank());
            strides[strides.size() - 1] = bufferElementSize;
            size_t stride = strides[strides.size() - 1];
            for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
                if(bufferShape[i] == 0) {
                    throw std::runtime_error("Error while creating user buffer.");
                }
                stride *= bufferShape[i];
                strides[i - 1] = stride;
            }
            auto requiredSize =
                calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());
            if(nnData->getData().size() < requiredSize) {
                auto error = fmt::format(
                    "Input buffer size is smaller than the required size. Input buffer size: {}, required size: {}", nnData->getData().size(), requiredSize);
                throw std::runtime_error(error);
            }
            zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
            auto buffer = ubFactory.createUserBuffer(nnData->getData().data(), requiredSize, strides, bufferAttributes->getEncoding());
            inputBufferMap.add(inputName.c_str(), buffer.get());
            snpeUserBackedBuffers.push_back(std::move(buffer));
        } else {
            auto bufferShape = bufferAttributes->getDims();
            auto bufferElementSize = bufferAttributes->getElementSize();
            std::vector<size_t> strides(bufferShape.rank());
            strides[strides.size() - 1] = bufferElementSize;
            size_t stride = strides[strides.size() - 1];
            for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
                if(bufferShape[i] == 0) {
                    throw std::runtime_error("Error while creating user buffer.");
                }
                stride *= bufferShape[i];
                strides[i - 1] = stride;
            }
            auto requiredSize =
                calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());
            if(input->getData().size() < requiredSize) {
                auto error = fmt::format(
                    "Input buffer size is smaller than the required size. Input buffer size: {}, required size: {}", input->getData().size(), requiredSize);
                throw std::runtime_error(error);
            }
            zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
            auto buffer = ubFactory.createUserBuffer(input->getData().data(), requiredSize, strides, bufferAttributes->getEncoding());
            inputBufferMap.add(inputName.c_str(), buffer.get());
            snpeUserBackedBuffers.push_back(std::move(buffer));
        }
    }
    return inputBufferMap;
}

std::optional<dai::TensorInfo::DataType> snpeToDaiDatatype(DlSystem::UserBufferEncoding::ElementType_t encoding) {
    using ElementType = zdl::DlSystem::UserBufferEncoding::ElementType_t;
    using DataType = dai::TensorInfo::DataType;
    DataType dataType;
    switch((encoding)) {
        case DlSystem::UserBufferEncoding::ElementType_t::FLOAT:
            dataType = DataType::FP32;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::FLOAT16:
            dataType = DataType::FP16;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::INT8:
            dataType = DataType::I8;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::UINT8:
        case DlSystem::UserBufferEncoding::ElementType_t::UNSIGNED8BIT:
        case DlSystem::UserBufferEncoding::ElementType_t::TF8:
        case DlSystem::UserBufferEncoding::ElementType_t::BOOL8:
            dataType = DataType::U8F;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::TF16:
            dataType = DataType::U16F;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::INT32:
        case DlSystem::UserBufferEncoding::ElementType_t::UINT32:
            dataType = DataType::INT;
            break;
        case DlSystem::UserBufferEncoding::ElementType_t::INT16:
        case DlSystem::UserBufferEncoding::ElementType_t::UINT16:
        case DlSystem::UserBufferEncoding::ElementType_t::INT64:
        case DlSystem::UserBufferEncoding::ElementType_t::UINT64:
        case DlSystem::UserBufferEncoding::ElementType_t::UNKNOWN:
            return std::nullopt;
    }
    return dataType;
}

void fillOutTensorInfoFromSNPE(const DlSystem::IBufferAttributes& bufferAttributes, TensorInfo& tensorInfo) {
    auto bufferShape = bufferAttributes.getDims();
    auto bufferElementSize = bufferAttributes.getElementSize();
    std::vector<unsigned> strides(bufferShape.rank());
    std::vector<unsigned> dimensions(bufferShape.rank());
    strides[strides.size() - 1] = bufferElementSize;
    size_t stride = strides[strides.size() - 1];
    for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
        if(bufferShape[i] == 0) {
            throw std::runtime_error("Error while creating user buffer.");
        }
        stride *= bufferShape[i];
        strides[i - 1] = stride;
    }
    for(size_t i = 0; i < bufferShape.rank(); i++) {
        dimensions[i] = bufferShape[i];
    }

    tensorInfo.dims = dimensions;
    tensorInfo.strides = strides;
    if(tensorInfo.dims.size() == 4) {
        tensorInfo.order = dai::TensorInfo::StorageOrder::NHWC;
    } else if(tensorInfo.dims.size() == 3) {
        tensorInfo.order = dai::TensorInfo::StorageOrder::HWC;
    } else if(tensorInfo.dims.size() == 2) {
        tensorInfo.order = dai::TensorInfo::StorageOrder::CN;
    } else if(tensorInfo.dims.size() == 1) {
        tensorInfo.order = dai::TensorInfo::StorageOrder::C;
    }

    tensorInfo.numDimensions = bufferShape.rank() - 1;  // The first dimension is the batch size, always 1

    auto dataTypeOpt = snpeToDaiDatatype(bufferAttributes.getEncodingType());
    if(!dataTypeOpt) {
        throw std::runtime_error("Couldn't convert the SNPE datatype to a DepthAI datatype.");
    }
    tensorInfo.dataType = *dataTypeOpt;
    tensorInfo.quantization = false;
    if(bufferAttributes.getEncodingType() == DlSystem::UserBufferEncoding::ElementType_t::TF8
       || bufferAttributes.getEncodingType() == DlSystem::UserBufferEncoding::ElementType_t::TF16) {
        tensorInfo.quantization = true;
        auto encoding = bufferAttributes.getEncoding();
        zdl::DlSystem::UserBufferEncodingTfN ubetfN = dynamic_cast<zdl::DlSystem::UserBufferEncodingTfN&>(*encoding);
        tensorInfo.qpScale = ubetfN.getQuantizedStepSize();
        tensorInfo.qpZp = ubetfN.getStepExactly0();
    }
}
zdl::DlSystem::UserBufferMap NeuralNetwork::createOutputBufferMap(std::shared_ptr<dai::NNData>& nnData,
                                                                  const std::unique_ptr<zdl::SNPE::SNPE>& model,
                                                                  std::vector<std::unique_ptr<zdl::DlSystem::IUserBuffer>>& snpeUserBackedBuffers) {
    zdl::DlSystem::UserBufferMap outputBufferMap;
    const auto& outputNames = model->getOutputTensorNames();
    if(!outputNames) {
        logger->error("Couldn't get the output tensor names.");
        return outputBufferMap;
    }
    size_t totalSize = 0;
    for(const auto& outputName : *outputNames) {
        auto bufferAttributesOpt = model->getInputOutputBufferAttributes(outputName);
        if(!bufferAttributesOpt) {
            logger->error("Couldn't get the buffer attributes for output tensor {}", outputName);
            continue;
        }
        const auto& bufferAttributes = *bufferAttributesOpt;
        totalSize += calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());
        totalSize = (totalSize + 63) & ~63;
    }
    auto dataHolder = nnData->data;
    auto vectorHolder = std::dynamic_pointer_cast<VectorMemory>(dataHolder);
    if(vectorHolder == nullptr) {
        throw std::runtime_error("Data holder is not a vector memory.");
    }
    vectorHolder->reserve(totalSize);
    for(const auto& outputName : *outputNames) {
        TensorInfo tensorInfo;
        tensorInfo.name = outputName;

        auto bufferAttributesOpt = model->getInputOutputBufferAttributes(outputName);
        if(!bufferAttributesOpt) {
            logger->error("Couldn't get the buffer attributes for output tensor {}", outputName);
            continue;
        }
        const auto& bufferAttributes = *bufferAttributesOpt;
        fillOutTensorInfoFromSNPE(*bufferAttributes, tensorInfo);
        auto allocatedSpan = nnData->emplaceTensor(tensorInfo);
        auto requiredSize =
            calcSizeFromDims(bufferAttributes->getDims().getDimensions(), bufferAttributes->getDims().rank(), bufferAttributes->getElementSize());
        if(allocatedSpan.size_bytes() < requiredSize) {
            logger->error("Allocated span size is smaller than the required size.");
            logger->error("Allocated span size: {}, required size: {}", allocatedSpan.size_bytes(), requiredSize);
            continue;
        }
        zdl::DlSystem::IUserBufferFactory& ubFactory = zdl::SNPE::SNPEFactory::getUserBufferFactory();
        auto bufferShape = bufferAttributes->getDims();
        auto bufferElementSize = bufferAttributes->getElementSize();
        std::vector<size_t> strides(bufferShape.rank());
        strides[strides.size() - 1] = bufferElementSize;
        size_t stride = strides[strides.size() - 1];
        for(size_t i = bufferShape.rank() - 1; i > 0; i--) {
            if(bufferShape[i] == 0) {
                throw std::runtime_error("Error while creating user buffer.");
            }
            stride *= bufferShape[i];
            strides[i - 1] = stride;
        }
        auto userBuffer = ubFactory.createUserBuffer(allocatedSpan.data(), allocatedSpan.size_bytes(), strides, bufferAttributes->getEncoding());
        if(userBuffer == nullptr) {
            logger->error("Couldn't create the user buffer for output tensor {}", outputName);
            continue;
        }
        outputBufferMap.add(outputName, userBuffer.get());
        snpeUserBackedBuffers.push_back(std::move(userBuffer));
    }
    return outputBufferMap;
}

void NeuralNetwork::sendOutputs(InputVariant& inputs, std::shared_ptr<dai::NNData>& nnData) {
    if(std::holds_alternative<std::shared_ptr<dai::Buffer>>(inputs)) {
        auto inputBuffer = std::get<std::shared_ptr<dai::Buffer>>(inputs);
        nnData->setBufferMetadataFrom(inputBuffer);
        if(std::dynamic_pointer_cast<dai::ImgFrame>(inputBuffer)) {
            auto imgInput = std::dynamic_pointer_cast<dai::ImgFrame>(inputBuffer);
            nnData->transformation = imgInput->transformation;
        } else {
            auto nnInput = std::dynamic_pointer_cast<dai::NNData>(inputBuffer);
            nnData->transformation = nnInput->transformation;
        }
        passthrough.send(inputBuffer);
    } else if(std::holds_alternative<std::map<std::string, std::shared_ptr<dai::Buffer>>>(inputs)) {
        auto inputBuffers = std::get<std::map<std::string, std::shared_ptr<dai::Buffer>>>(inputs);
        for(auto& passthroughInput : passthroughs) {
            if(inputBuffers.find(passthroughInput.first.second) == inputBuffers.end()) {
                logger->error("Passthrough input with name {} not found in the inputs map.", passthroughInput.first.first);
                continue;
            }
            passthroughInput.second.send(inputBuffers[passthroughInput.first.second]);
        }

        std::shared_ptr<Buffer> latestBuffer;
        std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> latestTimestamp = std::chrono::steady_clock::time_point::min();
        ImgTransformation transform;
        int transfCount = 0;
        for(auto& [name, buffer] : inputBuffers) {
            if(std::dynamic_pointer_cast<dai::ImgFrame>(buffer)) {
                transform = std::dynamic_pointer_cast<dai::ImgFrame>(buffer)->transformation;
                transfCount++;
            } else if(std::dynamic_pointer_cast<dai::NNData>(buffer)) {
                auto bufNNData = std::dynamic_pointer_cast<dai::NNData>(buffer);
                if(bufNNData->transformation.has_value()) {
                    transform = bufNNData->transformation.value();
                    transfCount++;
                }
            }
            if(buffer->getTimestamp() > latestTimestamp) {
                latestBuffer = buffer;
                latestTimestamp = buffer->getTimestamp();
            }
        }
        if(latestBuffer != nullptr) {
            nnData->setBufferMetadataFrom(latestBuffer);
            if(transfCount == 1) {
                nnData->transformation = transform;
            }
        }

    } else {
        throw std::runtime_error("InputVariant is not a map of strings to buffers.");
    }

    out.send(nnData);
}

bool NeuralNetwork::getInputs(InputVariant& inputVariant) {
    if(input.isConnected()) {
        auto inputBuffer = input.get<dai::Buffer>();
        if(inputBuffer == nullptr) {
            logger->error("Couldn't get the main input tensor.");
            return false;
        }
        inputVariant = inputBuffer;
    } else {
        if(inputs.size() == 0) {
            logger->error("No inputs connected.");
            return false;
        }
        if(!std::holds_alternative<std::map<std::string, std::shared_ptr<dai::Buffer>>>(inputVariant)) {
            logger->error("InputVariant is not a map of strings to buffers.");
            return false;
        }
        auto& inputTensors = std::get<std::map<std::string, std::shared_ptr<dai::Buffer>>>(inputVariant);
        for(auto& input : inputs) {
            auto name = input.first.second;
            if(inputTensors.find(name) == inputTensors.end() || inputTensors[name] == nullptr) {
                inputTensors[name] = input.second.get<dai::Buffer>();
                if(inputTensors[name] == nullptr) {
                    logger->error("Couldn't get the input tensor with name {}", name);
                    return false;
                }
                continue;
            }

            if(input.second.getWaitForMessage()) {
                auto inputBuffer = input.second.get<dai::Buffer>();
                if(inputBuffer == nullptr) {
                    logger->error("Couldn't get the input tensor with name {}", name);
                    return false;
                }
                inputTensors[name] = inputBuffer;
            } else {
                auto inputBuffer = input.second.tryGet<dai::Buffer>();
                if(inputBuffer != nullptr) {
                    inputTensors[name] = inputBuffer;
                }
            }
        }
    }
    return true;
}

void NeuralNetwork::SNPEBackendBuild(const uint8_t* modelData, std::size_t modelDataSize) {
    logger->debug("Building SNPE backend");
    logger->info("SNPE version is {}", zdl::SNPE::SNPEFactory::getLibraryVersion().toString());
    try {
        zdl::SNPE::SNPEFactory::initializeLogging(zdl::DlSystem::LogLevel_t::LOG_ERROR);

        logger->info("Using the xml interface to get the model");

        container = zdl::DlContainer::IDlContainer::open(modelData, modelDataSize);
        if(!container) {
            logger->error("Could not load the container!");
            throw std::runtime_error("ERROR: Could not load the container!");
        }

        if(properties.backendProperties.count("runtime") > 0) {
            auto runtimeBackend = properties.backendProperties["runtime"];
            runtime = zdl::DlSystem::RuntimeList::stringToRuntime(runtimeBackend.c_str());

            if(runtime != zdl::DlSystem::Runtime_t::UNSET) {
                logger->debug("Runtime is set to: {}", runtimeBackend);
            } else {
                runtime = zdl::DlSystem::Runtime_t::DSP;
            }
        }

        if(properties.backendProperties.count("performance_profile") > 0) {
            auto performanceProfileBackend = properties.backendProperties["performance_profile"];

            if(auto it = fromStringToPerformanceProfile.find(performanceProfileBackend.c_str()); it != fromStringToPerformanceProfile.end()) {
                performanceProfile = it->second;
                logger->debug("Performance profile is set to: {}", performanceProfileBackend);
            }
        }

        if(properties.backendProperties.count("priority_hint") > 0) {
            auto priorityHintBackend = properties.backendProperties["priority_hint"];

            auto it = fromStringToExecutionPriorityHint.find(priorityHintBackend.c_str());
            if(it != fromStringToExecutionPriorityHint.end()) {
                executionPriorityHint = it->second;
                logger->debug("Execution priority hint setting is set to: {}", priorityHintBackend);
            } else {
                logger->debug("Invalid execution priority hint setting. Defaulting to normal");
            }
        }
    } catch(const std::exception& ex) {
        logger->info("Crash exception: {}", ex.what());
        abort();
    }
}

void NeuralNetwork::SNPEBackendRun() {
    logger->debug("Running SNPE backend");
    try {
        std::vector<std::thread> runningThreads;

        int numThreads = getNumInferenceThreads();
        if(numThreads == 0) {
            numThreads = 1;
        }
        auto runAndCatchExceptions = [this](int threadId) {
            try {
                this->threadedRun(threadId);
            } catch(const dai::MessageQueue::QueueException& e) {
                logger->info("Queue closed - {}", e.what());
                this->threadsRunning = false;
            } catch(const std::exception& e) {
                logger->error("Node threw exception {}", e.what());
                this->threadsRunning = false;
            }
        };

        for(int i = 0; i < numThreads; i++) {
            std::thread runningThread(runAndCatchExceptions, i);
            runningThreads.push_back(std::move(runningThread));
        }
        for(auto& thread : runningThreads) {
            thread.join();
        }

    } catch(const std::exception& ex) {
        logger->info("Crash exception: {}", ex.what());
        abort();
    }
}

void NeuralNetwork::getModel(std::filesystem::path& modelPath) {
    auto assetXml = loadResource(properties.modelUri);

    modelPath = "/tmp/depthai_nn_" + std::to_string(id) + "_model.dlc";
    std::ofstream modelFile{modelPath, std::ios::out | std::ios::trunc | std::ios::binary};
    modelFile.write(reinterpret_cast<const char*>(assetXml.data()), assetXml.size());
}

}  // namespace gate
}  // namespace dai
