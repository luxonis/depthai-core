#include "../utility/HolisticRecordReplay.hpp"

#include <spdlog/spdlog.h>

#include <filesystem>
#include <memory>
#include <stdexcept>

#include "../utility/Platform.hpp"
#include "../utility/RecordReplayImpl.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"
#include "depthai/utility/Compression.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "pipeline/Node.hpp"
#include "utility/CompilerWarnings.hpp"

#define UNUSED(x) (void)(x)

namespace dai {
namespace utility {

static std::map<std::string, std::vector<std::pair<uint32_t, uint32_t>>> bestResolutionsRVC4 = {
    {"IMX582", {{320, 240}, {640, 480}, {960, 720}, {1280, 960}, {1440, 1080}, {1920, 1440}, {4000, 3000}, {8000, 6000}}},
    {"IMX586", {{320, 240}, {640, 480}, {960, 720}, {1280, 960}, {1440, 1080}, {1920, 1440}, {4000, 3000}, {8000, 6000}}},
    {"OV9282", {{640, 400}, {1280, 800}}},
    {"OV9782", {{640, 400}, {1280, 800}}},
    {"IMX766", {{320, 240}, {640, 480}, {960, 720}, {1280, 960}, {1440, 1080}, {1920, 1440}, {4000, 3000}}},   // HDK sensor
    {"OV64B40", {{320, 240}, {640, 480}, {960, 720}, {1280, 960}, {1440, 1080}, {1920, 1440}, {4000, 3000}}},  // HDK sensor
    {"AR0234", {{1920, 1080}}},

};

inline size_t roundDown(size_t numToRound, size_t multiple) {
    return numToRound - numToRound % multiple;
}
inline size_t roundUp(size_t numToRound, size_t multiple) {
    return roundDown(numToRound + multiple - 1UL, multiple);
}

Node::Output* setupHolistiRecordCamera(
    std::shared_ptr<dai::node::Camera> cam, Pipeline& pipeline, bool legacy, size_t& camWidth, size_t& camHeight, RecordConfig& recordConfig) {
    size_t requestWidth = cam->getMaxRequestedWidth();
    size_t requestHeight = cam->getMaxRequestedHeight();
    size_t width = cam->getMaxWidth();
    size_t height = cam->getMaxHeight();
    auto cams = pipeline.getDefaultDevice()->getConnectedCameraFeatures();
    for(const auto& cf : cams) {
        if(cf.socket == cam->getBoardSocket()) {
            if(legacy) {  // RVC2
                for(const auto& cfg : cf.configs) {
                    if(cfg.width > (int32_t)requestWidth && cfg.height > (int32_t)requestHeight) {
                        width = std::min((size_t)cfg.width, width);
                        height = std::min((size_t)cfg.height, height);
                    }
                }
            } else {  // RVC4
                auto res = bestResolutionsRVC4.find(cf.sensorName);
                if(res != bestResolutionsRVC4.end()) {
                    for(auto& r : res->second) {
                        if(r.first >= requestWidth && r.second >= requestHeight) {
                            width = std::min((size_t)r.first, width);
                            height = std::min((size_t)r.second, height);
                        }
                    }
                }
            }
            break;
        }
    }
    if(legacy) {
        if(width % 32 != 0UL) {
            auto down = roundDown(width, 32);
            width = down < requestWidth ? roundUp(width, 32) : down;
        }
        if(height % 8 != 0UL) {
            auto down = roundDown(height, 8);
            height = down < requestHeight ? roundUp(height, 8) : down;
        }
    }
    camWidth = width;
    camHeight = height;
    return cam->requestOutput(std::make_pair<uint32_t, uint32_t>(width, height), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP);
}

bool setupHolisticRecord(Pipeline& pipeline,
                         const std::string& deviceId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::filesystem::path>& outFilenames,
                         bool legacy) {
    auto sources = pipeline.getSourceNodes();
    const std::filesystem::path recordPath = recordConfig.outputDir;
    try {
        for(auto& node : sources) {
            auto nodeS = std::dynamic_pointer_cast<SourceNode>(node);
            if(nodeS == nullptr) {
                throw std::runtime_error("Node " + std::string(node->getName())
                                         + " is listed as a source node but does not implement the SourceNode interface.");
            }
            NodeRecordParams nodeParams = nodeS->getNodeRecordParams();
            std::string nodeName = (nodeParams.video ? "v_" : "b_") + nodeParams.name;
            std::filesystem::path filePath = platform::joinPaths(recordPath, deviceId + "_" + nodeName);
            outFilenames[nodeName] = filePath;
            DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
            if(std::dynamic_pointer_cast<node::Camera>(node) != nullptr || std::dynamic_pointer_cast<node::ColorCamera>(node) != nullptr
               || std::dynamic_pointer_cast<node::MonoCamera>(node) != nullptr) {
                Node::Output* output;
                size_t camWidth = 1920, camHeight = 1080;
                if(std::dynamic_pointer_cast<node::Camera>(node) != nullptr) {
                    output = setupHolistiRecordCamera(std::dynamic_pointer_cast<dai::node::Camera>(node), pipeline, legacy, camWidth, camHeight, recordConfig);
                } else {
                    output = &nodeS->getRecordOutput();
                }
                auto recordNode = pipeline.create<dai::node::RecordVideo>();
                recordNode->setRecordMetadataFile(std::filesystem::path(filePath).concat(".mcap"));
                recordNode->setRecordVideoFile(std::filesystem::path(filePath).concat(".mp4"));
                // TODO - once we allow for a lossless code, conditionally change the file extension
                // recordNode->setRecordVideoFile(std::filesystem::path(filePath).concat(".avi"));
                recordNode->setCompressionLevel((dai::RecordConfig::CompressionLevel)recordConfig.compressionLevel);
                if(recordConfig.videoEncoding.enabled) {
                    auto videnc = pipeline.create<dai::node::VideoEncoder>();
                    videnc->setProfile(recordConfig.videoEncoding.profile);
                    videnc->setLossless(recordConfig.videoEncoding.lossless);
                    videnc->setBitrate(recordConfig.videoEncoding.bitrate);
                    videnc->setQuality(recordConfig.videoEncoding.quality);
                    if((std::dynamic_pointer_cast<node::Camera>(node) != nullptr || std::dynamic_pointer_cast<node::ColorCamera>(node) != nullptr) && legacy) {
                        int maxOutputFrameSize = camWidth * camHeight * 3;
                        if(std::dynamic_pointer_cast<node::ColorCamera>(node) != nullptr) {
                            auto cam = std::dynamic_pointer_cast<dai::node::ColorCamera>(node);
                            maxOutputFrameSize = std::get<0>(cam->getIspSize()) * std::get<1>(cam->getIspSize()) * 3;
                        }
                        DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
                        auto imageManip = pipeline.create<dai::node::ImageManip>();
                        imageManip->initialConfig->setFrameType(ImgFrame::Type::NV12);
                        imageManip->setMaxOutputFrameSize(maxOutputFrameSize);

                        output->link(imageManip->inputImage);
                        imageManip->out.link(videnc->input);
                    } else {
                        output->link(videnc->input);
                    }
                    videnc->out.link(recordNode->input);
                } else {
                    output->link(recordNode->input);
                }
            } else {
                auto recordNode = pipeline.create<dai::node::RecordMetadataOnly>();
                recordNode->setRecordFile(std::filesystem::path(filePath).concat(".mcap"));
                recordNode->setCompressionLevel((dai::RecordConfig::CompressionLevel)recordConfig.compressionLevel);
                nodeS->getRecordOutput().link(recordNode->input);
            }
        }
        outFilenames["record_config"] = platform::joinPaths(recordPath, deviceId + "_record_config.json");
        outFilenames["calibration"] = platform::joinPaths(recordPath, deviceId + "_calibration.json");
    } catch(const std::runtime_error& e) {
        recordConfig.state = RecordConfig::RecordReplayState::NONE;
        spdlog::warn("Record disabled: {}", e.what());
        return false;
    }
    // Write recordConfig and calibration to output dir
    try {
        // Config
        std::filesystem::path pathConfig = platform::joinPaths(recordPath, deviceId + "_record_config.json");
        std::ofstream file(pathConfig);
        json j = recordConfig;
        file << j.dump(4);
        file.close();
        // Calibration
        std::filesystem::path pathCalib = platform::joinPaths(recordPath, deviceId + "_calibration.json");
        auto calibData = pipeline.getDefaultDevice()->readCalibration().eepromToJson();
        std::ofstream calibFile(pathCalib);
        calibFile << calibData.dump(4);
        calibFile.close();
    } catch(const std::exception& e) {
        spdlog::warn("Error while writing DEPTHAI_RECORD json file: {}", e.what());
        return false;
    }
    return true;
}

bool setupHolisticReplay(Pipeline& pipeline,
                         std::filesystem::path replayPath,
                         const std::string& deviceId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::filesystem::path>& outFilenames,
                         bool legacy) {
    UNUSED(deviceId);
    auto sources = pipeline.getSourceNodes();
    try {
        bool useTar = !platform::checkPathExists(replayPath, true);
        bool hasCalibration = false;
        std::vector<std::string> tarNodenames;
        std::string tarRoot = ".";
        std::filesystem::path rootPath = useTar ? platform::getTempPath() : replayPath;
        if(useTar)
            tarNodenames = filenamesInTar(replayPath);
        else
            tarNodenames = platform::getFilenamesInDirectory(replayPath);
        hasCalibration = std::any_of(tarNodenames.begin(), tarNodenames.end(), [](const std::string& path) {
            auto pathDelim = path.find_last_of("/\\");
            auto filename = pathDelim == std::string::npos ? path : path.substr(path.find_last_of("/\\") + 1);
            return filename == "calibration.json";
        });
        tarNodenames.erase(std::remove_if(tarNodenames.begin(),
                                          tarNodenames.end(),
                                          [](const std::string& path) {
                                              auto pathDelim = path.find_last_of("/\\");
                                              auto filename = pathDelim == std::string::npos ? path : path.substr(path.find_last_of("/\\") + 1);
                                              return filename.size() < 5 || filename.substr(filename.size() - 4, filename.size()) != "mcap";
                                          }),
                           tarNodenames.end());
        if(useTar) tarRoot = tarNodenames.empty() ? "" : tarNodenames[0].substr(0, tarNodenames[0].find_last_of("/\\") + 1);
        for(auto& path : tarNodenames) {
            auto pathDelim = path.find_last_of("/\\");
            path = pathDelim == std::string::npos ? path : path.substr(path.find_last_of("/\\") + 1);
            path = path.substr(0, path.find_last_of('.'));
        }

        std::vector<std::string> nodeNames;
        std::vector<std::string> pipelineFilenames;
        pipelineFilenames.reserve(sources.size());
        nodeNames.reserve(sources.size());
        for(auto& node : sources) {
            auto nodeS = std::dynamic_pointer_cast<SourceNode>(node);
            if(nodeS == nullptr) {
                throw std::runtime_error("Node is listed as a source node but does not implement the SourceNode interface.");
            }
            NodeRecordParams nodeParams = nodeS->getNodeRecordParams();
            // Needed for muti-device recordings, not yet supported
            // std::string nodeName = (deviceId + "_").append(nodeParams.name);
            nodeNames.push_back(nodeParams.name);
            pipelineFilenames.push_back(nodeParams.name);
        }
        std::filesystem::path configPath;
        std::filesystem::path calibrationPath;
        std::vector<std::string> inFiles;
        std::vector<std::filesystem::path> outFiles;
        inFiles.reserve(sources.size() + 1);
        outFiles.reserve(sources.size() + 1);
        if(allMatch(pipelineFilenames, tarNodenames)) {
            for(auto& nodeName : nodeNames) {
                // auto filename = (deviceId + "_").append(nodeName);
                auto filename = nodeName;
                if(useTar) {
                    inFiles.push_back(tarRoot + filename + ".mp4");
                    inFiles.push_back(tarRoot + filename + ".mcap");
                }
                std::filesystem::path filePath = platform::joinPaths(rootPath, filename);
                outFiles.push_back(std::filesystem::path(filePath).concat(".mp4"));
                outFiles.push_back(std::filesystem::path(filePath).concat(".mcap"));
                outFilenames[nodeName] = filePath;
            }
            if(useTar) {
                inFiles.emplace_back(tarRoot + "record_config.json");
                inFiles.emplace_back(tarRoot + "calibration.json");
            }
            configPath = platform::joinPaths(rootPath, "record_config.json");
            calibrationPath = platform::joinPaths(rootPath, "calibration.json");
            outFiles.push_back(configPath);
            outFiles.push_back(calibrationPath);
            outFilenames["record_config"] = configPath;
            outFilenames["calibration"] = calibrationPath;
            if(useTar) untarFiles(replayPath, inFiles, outFiles);
        } else {
            throw std::runtime_error("Recording does not match the pipeline configuration.");
            // For multi-device recordings, where devices are not the same
            // std::vector<std::string> mxIds;
            // mxIds.reserve(tarFilenames.size());
            // for(auto& filename : tarFilenames) {
            //     mxIds.push_back(filename.substr(0, filename.find_first_of('_')));
            // }
            // // Get unique mxIds
            // std::sort(mxIds.begin(), mxIds.end());
            // mxIds.erase(std::unique(mxIds.begin(), mxIds.end()), mxIds.end());
            // std::string mxIdRec = matchTo(mxIds, tarFilenames, nodeNames);
            // if(mxIdRec.empty()) {
            //     throw std::runtime_error("No recordings match the pipeline configuration.");
            // }
            // for(auto& nodeName : nodeNames) {
            //     auto inFilename = (mxIdRec + "_").append(nodeName).append(".rec");
            //     auto outFilename = (mxId + "_").append(nodeName).append(".rec");
            //     inFiles.push_back(inFilename + ".mp4");
            //     inFiles.push_back(inFilename + ".mcap");
            //     std::string filePath = platform::joinPaths(rootPath, outFilename);
            //     outFiles.push_back(filePath + ".mp4");
            //     outFiles.push_back(filePath + ".mcap");
            //     outFilenames[nodeName] = filePath;
            // }
            // inFiles.emplace_back("record_config.json");
            // configPath = platform::joinPaths(rootPath, mxId + "_record_config.json");
            // outFiles.push_back(configPath);
            // outFilenames["record_config"] = configPath;
            // untarFiles(replayPath, inFiles, outFiles);
        }

        std::ifstream file(configPath);
        json j = json::parse(file);
        recordConfig = j.get<RecordConfig>();
        recordConfig.state = RecordConfig::RecordReplayState::REPLAY;

        if(hasCalibration) {
            std::ifstream calibFile(calibrationPath);
            json jCalib = json::parse(calibFile);
            CalibrationHandler calib;
            try {
                calib.fromJson(jCalib, true);
                pipeline.getDefaultDevice()->setCalibration(calib);
            } catch(const std::runtime_error& e) {
                spdlog::warn("Recorded calibration is invalid: {}", e.what());
                hasCalibration = false;
            }
        }

        for(auto& node : sources) {
            auto nodeS = std::dynamic_pointer_cast<SourceNode>(node);
            if(nodeS == nullptr) {
                throw std::runtime_error("Node is listed as a source node but does not implement the SourceNode interface.");
            }
            NodeRecordParams nodeParams = nodeS->getNodeRecordParams();
            std::string nodeName = nodeParams.name;
            DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
            auto cameraNode = std::dynamic_pointer_cast<node::Camera>(node);
            auto colorCameraNode = std::dynamic_pointer_cast<node::ColorCamera>(node);
            auto monoCameraNode = std::dynamic_pointer_cast<node::MonoCamera>(node);
            if(cameraNode || colorCameraNode || monoCameraNode) {
                auto replay = pipeline.create<dai::node::ReplayVideo>();
                // replay->setReplayFile(platform::joinPaths(rootPath, (mxId + "_").append(nodeName).append(".mcap")));
                replay->setReplayMetadataFile(platform::joinPaths(rootPath, nodeName + ".mcap"));
                // replay->setReplayVideo(platform::joinPaths(rootPath, (mxId + "_").append(nodeName).append(".mp4")));
                replay->setReplayVideoFile(platform::joinPaths(rootPath, nodeName + ".mp4"));
                replay->setOutFrameType(legacy ? ImgFrame::Type::YUV420p : ImgFrame::Type::NV12);

                auto videoSize = BytePlayer::getVideoSize(replay->getReplayMetadataFile().string());
                if(videoSize.has_value()) {
                    auto [width, height] = videoSize.value();
                    if(cameraNode) {
                        cameraNode->properties.mockIspWidth = width;
                        cameraNode->properties.mockIspHeight = height;
                    } else if(colorCameraNode) {
                        colorCameraNode->setMockIspSize(width, height);
                    } else if(monoCameraNode) {
                        monoCameraNode->setMockIspSize(width, height);
                    }
                }
                DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
                replay->out.link(nodeS->getReplayInput());
            } else {
                auto replay = pipeline.create<dai::node::ReplayMetadataOnly>();
                replay->setReplayFile(platform::joinPaths(rootPath, nodeName + ".mcap"));
                replay->out.link(nodeS->getReplayInput());
            }
        }
        return true;
    } catch(const std::exception& e) {
        spdlog::warn("Replay disabled: {}", e.what());
    }
    return false;
}

}  // namespace utility
}  // namespace dai
