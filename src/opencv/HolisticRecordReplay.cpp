#include "../utility/HolisticRecordReplay.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>

#include "../utility/Platform.hpp"
#include "../utility/RecordReplayImpl.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/MessageDemux.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"
#include "depthai/utility/Compression.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "depthai/utility/RecordingStore.hpp"
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

void getCameraRecordSize(const std::shared_ptr<dai::node::Camera>& cam, Pipeline& pipeline, bool legacy, size_t& camWidth, size_t& camHeight) {
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
        width = std::min((size_t)cam->getMaxWidth(), width);
        height = std::min((size_t)cam->getMaxHeight(), height);
    }
    camWidth = width;
    camHeight = height;
}

Node::Output* setupHolisticRecordCamera(
    const std::shared_ptr<dai::node::Camera>& cam, Pipeline& pipeline, bool legacy, float recordingFps, size_t& camWidth, size_t& camHeight) {
    getCameraRecordSize(cam, pipeline, legacy, camWidth, camHeight);
    if(recordingFps > 0.0f) {
        return cam->requestOutput(std::pair<uint32_t, uint32_t>(camWidth, camHeight), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, recordingFps);
    }
    return cam->requestOutput(
        std::pair<uint32_t, uint32_t>(camWidth, camHeight), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, cam->getMaxRequestedFps());
}

bool setupHolisticRecord(Pipeline pipeline,
                         const std::string& deviceId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::filesystem::path>& outFilenames,
                         bool legacy) {
    auto sources = pipeline.getSourceNodes();
    const std::filesystem::path recordPath = recordConfig.outputDir;
    try {
        if(!std::filesystem::is_directory(recordPath)) {
            throw std::runtime_error("Record output path " + recordPath.string() + " is not a directory.");
        }
        // Get max requested fps & create sync and demux nodes
        float maxRequestedFps = 0.0f;
        const bool syncCameraOutputs = recordConfig.syncCameraOutputs;
        std::shared_ptr<dai::node::Sync> sync = nullptr;
        std::shared_ptr<dai::node::MessageDemux> demux = nullptr;
        if(syncCameraOutputs) {
            for(auto& node : sources) {
                DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
                auto cameraNode = std::dynamic_pointer_cast<node::Camera>(node);
                auto colorCameraNode = std::dynamic_pointer_cast<node::ColorCamera>(node);
                auto monoCameraNode = std::dynamic_pointer_cast<node::MonoCamera>(node);
                if(cameraNode) {
                    maxRequestedFps = std::max(maxRequestedFps, cameraNode->getMaxRequestedFps());
                } else if(colorCameraNode) {
                    if(colorCameraNode->getFps() > 0.0f) {
                        maxRequestedFps = std::max(maxRequestedFps, colorCameraNode->getFps());
                    }
                } else if(monoCameraNode) {
                    if(monoCameraNode->getFps() > 0.0f) {
                        maxRequestedFps = std::max(maxRequestedFps, monoCameraNode->getFps());
                    }
                }
                DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
            }
            sync = pipeline.create<dai::node::Sync>();
            demux = pipeline.create<dai::node::MessageDemux>();
            sync->out.link(demux->input);
            if(maxRequestedFps > 0.0f) {
                sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.9 / (double)maxRequestedFps)));
            }
        }

        // Check if video encoding can be used (recorded output sizes must be aligned)
        for(auto& node : sources) {
            if(std::dynamic_pointer_cast<node::Camera>(node) != nullptr) {
                size_t camWidth = 1920;
                size_t camHeight = 1080;
                getCameraRecordSize(std::dynamic_pointer_cast<dai::node::Camera>(node), pipeline, legacy, camWidth, camHeight);
                if(legacy && recordConfig.videoEncoding.enabled && (camWidth % 32 != 0 || camHeight % 8 != 0)) {
                    spdlog::warn("Holistic record: video encoding disabled due to incompatible stream size ({}x{})", camWidth, camHeight);
                    recordConfig.videoEncoding.enabled = false;
                    break;
                }
            }
        }

        // RVC4 cannot write an AVI on the device. Keep the host nodes and say so.
        const bool hostVideoFallback = !legacy && recordConfig.format == RecordFormat::VIDEO && !recordConfig.videoEncoding.enabled;
        if(hostVideoFallback) {
            spdlog::warn("Record holistic: video encoding off, host nodes used");
        }
        const bool useLegacyLayout = legacy || hostVideoFallback;
        const std::string sharedRecordingName = "holistic_" + deviceId + "_" + makeRecordingTimestamp();
        if(!useLegacyLayout) {
            outFilenames["holistic_recording"] = platform::joinPaths(recordPath, sharedRecordingName);
        }
        if(demux != nullptr) {
            demux->setRunOnHost(useLegacyLayout);
        }

        for(auto& node : sources) {
            auto nodeS = std::dynamic_pointer_cast<SourceNode>(node);
            if(nodeS == nullptr) {
                throw std::runtime_error("Node " + std::string(node->getName())
                                         + " is listed as a source node but does not implement the SourceNode interface.");
            }
            NodeRecordParams nodeParams = nodeS->getNodeRecordParams();
            std::string nodeName = (nodeParams.video ? "v_" : "b_") + nodeParams.name;
            std::filesystem::path filePath = platform::joinPaths(recordPath, deviceId + "_" + nodeName);
            DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
            const bool cameraSource = std::dynamic_pointer_cast<node::Camera>(node) != nullptr || std::dynamic_pointer_cast<node::ColorCamera>(node) != nullptr
                                      || std::dynamic_pointer_cast<node::MonoCamera>(node) != nullptr;
            if(cameraSource) {
                if(useLegacyLayout) outFilenames[nodeName] = filePath;
                Node::Output* output = nullptr;
                size_t camWidth = 1920;
                size_t camHeight = 1080;
                float streamFps = 30.f;
                if(auto cameraNode = std::dynamic_pointer_cast<node::Camera>(node)) {
                    const float requested = syncCameraOutputs ? maxRequestedFps : 0.0f;
                    output = setupHolisticRecordCamera(cameraNode, pipeline, legacy, requested, camWidth, camHeight);
                    streamFps = requested > 0.0f ? requested : cameraNode->getMaxRequestedFps();
                } else {
                    output = &nodeS->getRecordOutput();
                    if(auto colorCameraNode = std::dynamic_pointer_cast<node::ColorCamera>(node)) {
                        streamFps = colorCameraNode->getFps();
                    } else if(auto monoCameraNode = std::dynamic_pointer_cast<node::MonoCamera>(node)) {
                        streamFps = monoCameraNode->getFps();
                    }
                }
                if(!(streamFps > 0.0f)) streamFps = 30.f;
                recordConfig.streams[nodeParams.name] = RecordConfig::StreamConfig{streamFps};

                auto recordNode = pipeline.create<dai::node::RecordVideo>();
                recordNode->setStreamFps(streamFps);
                recordNode->setCompressionLevel(recordConfig.compressionLevel);
                recordNode->setPreTriggerFrames(0);
                recordNode->input.setBlocking(true);
                if(!useLegacyLayout) recordNode->setKeepOnDevice(recordConfig.keepOnDevice);
                Node::Output* recordOutput = output;
                const bool encode = recordConfig.videoEncoding.enabled && (useLegacyLayout || recordConfig.format == RecordFormat::VIDEO);
                if(useLegacyLayout) {
                    recordNode->setRecordMetadataFile(std::filesystem::path(filePath).concat(".mcap"));
                    recordNode->setRecordVideoFile(std::filesystem::path(filePath).concat(encode ? ".mp4" : ".avi"));
                } else {
                    recordNode->setFormat(recordConfig.format);
                    recordNode->setRecordingName(sharedRecordingName);
                    recordNode->setStreamName(nodeParams.name);
                    recordNode->setOutputDirectory(recordPath);
                }
                if(encode) {
                    auto videnc = pipeline.create<dai::node::VideoEncoder>();
                    videnc->setProfile(recordConfig.videoEncoding.profile);
                    videnc->setLossless(recordConfig.videoEncoding.lossless);
                    videnc->setBitrate(recordConfig.videoEncoding.bitrate);
                    videnc->setQuality(recordConfig.videoEncoding.quality);
                    if((std::dynamic_pointer_cast<node::Camera>(node) != nullptr || std::dynamic_pointer_cast<node::ColorCamera>(node) != nullptr) && legacy) {
                        int maxOutputFrameSize = static_cast<int>(camWidth * camHeight * 3);
                        if(std::dynamic_pointer_cast<node::ColorCamera>(node) != nullptr) {
                            auto cam = std::dynamic_pointer_cast<dai::node::ColorCamera>(node);
                            maxOutputFrameSize = std::get<0>(cam->getIspSize()) * std::get<1>(cam->getIspSize()) * 3;
                        }
                        auto imageManip = pipeline.create<dai::node::ImageManip>();
                        imageManip->initialConfig->setFrameType(ImgFrame::Type::NV12);
                        imageManip->setMaxOutputFrameSize(maxOutputFrameSize);

                        output->link(imageManip->inputImage);
                        imageManip->out.link(videnc->input);
                    } else {
                        output->link(videnc->input);
                    }
                    recordOutput = &videnc->out;
                }
                if(syncCameraOutputs) {
                    recordOutput->link(sync->inputs[nodeName]);
                    demux->outputs[nodeName].link(recordNode->input);
                } else {
                    recordOutput->link(recordNode->input);
                }
            } else {
                outFilenames[nodeName] = filePath;
                auto recordNode = pipeline.create<dai::node::RecordMetadataOnly>();
                recordNode->setRecordFile(std::filesystem::path(filePath).concat(".mcap"));
                recordNode->setCompressionLevel((dai::RecordConfig::CompressionLevel)recordConfig.compressionLevel);
                nodeS->getRecordOutput().link(recordNode->input);
            }
            DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
        }
        outFilenames["record_config"] = platform::joinPaths(recordPath, deviceId + "_record_config.json");
        outFilenames["camera_info"] = platform::joinPaths(recordPath, deviceId + "_camera_info.json");
    } catch(const std::runtime_error& e) {
        recordConfig.state = RecordConfig::RecordReplayState::NONE;
        spdlog::warn("Record disabled: {}", e.what());
        return false;
    }
    // Write recordConfig and camera info to output dir
    try {
        // Config
        std::filesystem::path pathConfig = platform::joinPaths(recordPath, deviceId + "_record_config.json");
        std::ofstream file(pathConfig);
        json j = recordConfig;
        file << j.dump(4);
        file.close();
        // Camera info
        std::filesystem::path pathCamInfo = platform::joinPaths(recordPath, deviceId + "_camera_info.json");
        auto calibData = pipeline.getDefaultDevice()->getCalibration().eepromToJson();
        auto cameraFeatures = pipeline.getDefaultDevice()->getConnectedCameraFeatures();
        auto imu = pipeline.getDefaultDevice()->getConnectedIMU();
        nlohmann::json cameraInfo;
        cameraInfo["calibration"] = calibData;
        cameraInfo["camera_features"] = cameraFeatures;
        cameraInfo["imu"] = imu;
        std::ofstream camInfoFile(pathCamInfo);
        camInfoFile << cameraInfo.dump(4);
        camInfoFile.close();
    } catch(const std::exception& e) {
        spdlog::warn("Error while writing DEPTHAI_RECORD json file: {}", e.what());
        return false;
    }
    return true;
}

bool mockCameraFeatures(DeviceBase& device, std::filesystem::path replayPath) {
    if(!platform::checkPathExists(replayPath)) {
        spdlog::warn("Replay path does not exist: {}", replayPath.string());
        return false;
    }
    bool useTar = !platform::checkPathExists(replayPath, true);
    std::vector<std::string> tarNodenames;
    std::string tarRoot = ".";
    if(useTar)
        tarNodenames = filenamesInArchive(replayPath);
    else
        tarNodenames = platform::getFilenamesInDirectory(replayPath);
    bool hasCameraInfo = std::any_of(tarNodenames.begin(), tarNodenames.end(), [](const std::string& path) {
        auto pathObj = std::filesystem::path(path);
        auto filename = pathObj.filename().string();
        return filename == "camera_info.json";
    });
    bool hasCalibration = std::any_of(tarNodenames.begin(), tarNodenames.end(), [](const std::string& path) {
        auto pathObj = std::filesystem::path(path);
        auto filename = pathObj.filename().string();
        return filename == "calibration.json";
    });
    if(useTar) tarRoot = tarNodenames.empty() ? "." : tarNodenames[0].substr(0, tarNodenames[0].find_last_of("/\\") + 1);
    if(hasCameraInfo || hasCalibration) {
        std::vector<uint8_t> cameraInfoData;
        if(useTar) {
            try {
                cameraInfoData = readFileInArchive(replayPath, tarRoot + (hasCameraInfo ? "camera_info.json" : "calibration.json"));
            } catch(const std::exception& e) {
                spdlog::warn("Error while reading from tar file: {}", e.what());
                return false;
            }
        } else {
            std::filesystem::path cameraInfoPath = platform::joinPaths(replayPath, (hasCameraInfo ? "camera_info.json" : "calibration.json"));
            if(!std::filesystem::exists(cameraInfoPath)) {
                spdlog::warn("camera_info.json not found in replay path.");
                return false;
            }

            std::ifstream cameraInfoFile(cameraInfoPath);
            if(!cameraInfoFile) {
                spdlog::warn("Could not open camera_info.json.");
                return false;
            }
            cameraInfoData = std::vector<uint8_t>((std::istreambuf_iterator<char>(cameraInfoFile)), std::istreambuf_iterator<char>());
        }

        CalibrationHandler calib;
        try {
            json jCamInfo = json::parse(cameraInfoData);
            calib = CalibrationHandler::fromJson(hasCameraInfo ? jCamInfo["calibration"] : jCamInfo, true);
            device.setCalibration(calib);
            if(hasCameraInfo) {
                std::vector<CameraFeatures> camFeatures = jCamInfo["camera_features"];
                std::string imu = jCamInfo["imu"];
                device.overrideCameraFeatures(camFeatures, imu);
            }
        } catch(const std::runtime_error& e) {
            spdlog::warn("Recorded camera information is invalid: {}", e.what());
            return false;
        }

        return hasCameraInfo;
    }
    return false;
}

namespace {

std::string normalizeArchiveName(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    while(path.rfind("./", 0) == 0) path.erase(0, 2);
    return path;
}

std::string lowerExtOf(const std::string& name) {
    auto ext = std::filesystem::path(name).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return ext;
}

struct RecordingIndex {
    bool archive = false;
    /// Prefix removed from every entry, including the trailing slash. Empty when the archive is flat.
    std::string prefix;
    std::vector<std::string> names;
};

RecordingIndex indexRecording(const std::filesystem::path& replayPath, bool archive) {
    RecordingIndex index;
    index.archive = archive;
    std::vector<std::string> raw;
    if(archive) {
        raw = filenamesInArchive(replayPath);
    } else {
        for(const auto& entry : std::filesystem::recursive_directory_iterator(replayPath)) {
            if(!entry.is_regular_file()) continue;
            raw.push_back(std::filesystem::relative(entry.path(), replayPath).generic_string());
        }
    }
    for(auto& name : raw) name = normalizeArchiveName(std::move(name));
    std::string common;
    bool first = true;
    bool mixed = false;
    for(const auto& name : raw) {
        if(name.empty() || name.back() == '/') continue;
        const auto slash = name.find('/');
        const std::string top = slash == std::string::npos ? std::string() : name.substr(0, slash + 1);
        if(first) {
            common = top;
            first = false;
        } else if(common != top) {
            mixed = true;
            break;
        }
    }
    if(!mixed && !common.empty()) {
        index.prefix = common;
        for(auto& name : raw) {
            if(name.rfind(common, 0) == 0) name = name.substr(common.size());
        }
    }
    raw.erase(std::remove_if(raw.begin(), raw.end(), [](const std::string& name) { return name.empty() || name.back() == '/'; }), raw.end());
    index.names = std::move(raw);
    return index;
}

bool hasExact(const RecordingIndex& index, const std::string& name) {
    return std::find(index.names.begin(), index.names.end(), name) != index.names.end();
}

bool hasFrameDirectory(const RecordingIndex& index, const std::string& name) {
    const std::string prefix = name + "/";
    const std::string marker = name + "_";
    for(const auto& entry : index.names) {
        if(entry.rfind(prefix, 0) != 0) continue;
        const auto file = entry.substr(prefix.size());
        if(file.find('/') != std::string::npos) continue;
        if(file.rfind(marker, 0) != 0) continue;
        if(lowerExtOf(file) == ".dai") return true;
    }
    return false;
}

enum class HolisticLayout { Video, Frames, Mcap, Missing };

HolisticLayout cameraLayout(const RecordingIndex& index, const std::string& name, std::string& videoExt) {
    if(hasFrameDirectory(index, name)) return HolisticLayout::Frames;
    const bool mp4 = hasExact(index, name + ".mp4");
    const bool avi = hasExact(index, name + ".avi");
    const bool mcap = hasExact(index, name + ".mcap");
    if(mp4 && avi) throw std::runtime_error("Recording contains both .mp4 and .avi files.");
    if((mp4 || avi) && mcap) {
        videoExt = mp4 ? ".mp4" : ".avi";
        return HolisticLayout::Video;
    }
    if(mcap) return HolisticLayout::Mcap;
    return HolisticLayout::Missing;
}

std::string archiveEntryName(const RecordingIndex& index, const std::string& name) {
    return index.prefix + name;
}

std::optional<nlohmann::json> readRecordConfig(const std::filesystem::path& replayPath, const RecordingIndex& index) {
    std::string configName;
    for(const auto& name : index.names) {
        if(std::filesystem::path(name).filename() == "record_config.json") {
            configName = name;
            break;
        }
    }
    if(configName.empty()) return std::nullopt;
    std::vector<std::uint8_t> bytes;
    if(index.archive) {
        bytes = readFileInArchive(replayPath, archiveEntryName(index, configName));
    } else {
        std::ifstream input(replayPath / configName, std::ios::binary);
        if(!input) return std::nullopt;
        bytes.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    }
    return nlohmann::json::parse(bytes);
}

std::optional<float> configuredStreamFps(const std::optional<nlohmann::json>& config, const std::string& name) {
    if(!config.has_value() || !config->contains("streams") || !(*config)["streams"].contains(name)) return std::nullopt;
    const auto& stream = (*config)["streams"][name];
    if(!stream.contains("fps")) return std::nullopt;
    return stream["fps"].get<float>();
}

std::filesystem::path materialize(const std::filesystem::path& replayPath,
                                  const RecordingIndex& index,
                                  const std::string& entry,
                                  const std::filesystem::path& directory) {
    auto destination = directory / std::filesystem::path(entry).filename();
    if(index.archive) {
        extractFiles(replayPath, {archiveEntryName(index, entry)}, {destination});
        return destination;
    }
    return replayPath / entry;
}

double nominalFromDai(const std::vector<std::filesystem::path>& frames) {
    if(frames.size() < 2) return 1.0 / 30.0;
#ifndef DEPTHAI_ENABLE_PROTOBUF
    (void)frames;
    throw std::runtime_error("ReplayVideo node requires protobuf support");
#else
    ImgFrame first;
    ImgFrame last;
    first.load(frames.front());
    last.load(frames.back());
    const auto delta = last.getTimestampDevice() - first.getTimestampDevice();
    const auto seconds = std::chrono::duration<double>(delta).count() / static_cast<double>(frames.size() - 1);
    return seconds > 0.0 ? seconds : 1.0 / 30.0;
#endif
}

std::filesystem::path packDirectory(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> onDisk;
    std::vector<std::string> inArchive;
    for(const auto& entry : std::filesystem::recursive_directory_iterator(directory)) {
        if(!entry.is_regular_file()) continue;
        const auto relative = std::filesystem::relative(entry.path(), directory);
        bool hidden = false;
        for(const auto& part : relative) {
            const auto text = part.string();
            if(!text.empty() && text.front() == '.') hidden = true;
        }
        if(hidden) continue;
        onDisk.push_back(entry.path());
        inArchive.push_back(relative.generic_string());
    }
    auto destination = platform::getTempPath() / "depthai_holistic_pack" / (directory.filename().string() + ".tar");
    std::filesystem::create_directories(destination.parent_path());
    archiveFiles(destination, onDisk, inArchive);
    return destination;
}

void applyMockIspSize(const std::shared_ptr<Node>& node, int width, int height, double fps) {
    if(auto camera = std::dynamic_pointer_cast<node::Camera>(node)) {
        camera->properties.mockIspWidth = width;
        camera->properties.mockIspHeight = height;
        camera->properties.mockIspFps = fps;
        return;
    }
    DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
    if(auto color = std::dynamic_pointer_cast<node::ColorCamera>(node)) color->setMockIspSize(width, height);
    if(auto mono = std::dynamic_pointer_cast<node::MonoCamera>(node)) mono->setMockIspSize(width, height);
    DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
}

bool setupHolisticReplayDevice(Pipeline& pipeline, const std::filesystem::path& replayPath, bool legacy) {
    const bool archive = !platform::checkPathExists(replayPath, true);
    const auto index = indexRecording(replayPath, archive);
    const auto recordConfig = readRecordConfig(replayPath, index);
    auto probeDir = platform::getTempPath() / "depthai_holistic_probe";
    std::filesystem::remove_all(probeDir);
    std::filesystem::create_directories(probeDir);

    struct Probe {
        int width = 0;
        int height = 0;
        double fps = 30.0;
    };
    std::unordered_map<std::string, Probe> probes;
    std::unordered_map<std::string, HolisticLayout> layouts;

    auto sources = pipeline.getSourceNodes();
    for(const auto& node : sources) {
        auto source = std::dynamic_pointer_cast<SourceNode>(node);
        if(source == nullptr) throw std::runtime_error("Node is listed as a source node but does not implement the SourceNode interface.");
        const auto name = source->getNodeRecordParams().name;
        const bool camera = std::dynamic_pointer_cast<node::Camera>(node) || std::dynamic_pointer_cast<node::ColorCamera>(node)
                            || std::dynamic_pointer_cast<node::MonoCamera>(node);
        if(!camera) {
            if(!hasExact(index, name + ".mcap")) throw std::runtime_error("Recording does not match the pipeline configuration.");
            layouts[name] = HolisticLayout::Mcap;
            continue;
        }
        std::string videoExt = ".mp4";
        const auto layout = cameraLayout(index, name, videoExt);
        if(layout == HolisticLayout::Missing) throw std::runtime_error("Recording does not match the pipeline configuration.");
        layouts[name] = layout;
        Probe probe;
        if(layout == HolisticLayout::Video) {
            const auto video = materialize(replayPath, index, name + videoExt, probeDir);
            const auto metadata = materialize(replayPath, index, name + ".mcap", probeDir);
            const auto [width, height, fps] = utility::getVideoSize(video.string());
            probe.width = static_cast<int>(width);
            probe.height = static_cast<int>(height);
            probe.fps = fps;
            const auto metadataSize = BytePlayer::getVideoSize(metadata.string());
            if(metadataSize.has_value()) {
                const auto [metaWidth, metaHeight] = metadataSize.value();
                if(static_cast<int>(metaWidth) != probe.width || static_cast<int>(metaHeight) != probe.height) {
                    throw std::runtime_error("Video size does not match metadata size for node " + name);
                }
            }
        } else if(layout == HolisticLayout::Frames) {
            std::vector<std::pair<std::int64_t, std::string>> frames;
            const std::string prefix = name + "/" + name + "_";
            for(const auto& entry : index.names) {
                if(entry.rfind(prefix, 0) != 0 || lowerExtOf(entry) != ".dai") continue;
                const auto seqText = std::filesystem::path(entry).stem().string().substr(name.size() + 1);
                frames.emplace_back(std::stoll(seqText), entry);
            }
            std::sort(frames.begin(), frames.end());
            if(frames.empty()) throw std::runtime_error("Recording does not match the pipeline configuration.");
            std::vector<std::filesystem::path> local;
            local.push_back(materialize(replayPath, index, frames.front().second, probeDir));
            if(frames.size() > 1) local.push_back(materialize(replayPath, index, frames.back().second, probeDir));
#ifndef DEPTHAI_ENABLE_PROTOBUF
            throw std::runtime_error("ReplayVideo node requires protobuf support");
#else
            ImgFrame first;
            first.load(local.front());
            probe.width = static_cast<int>(first.getWidth());
            probe.height = static_cast<int>(first.getHeight());
#endif
            const auto configured = configuredStreamFps(recordConfig, name);
            probe.fps = configured.has_value() ? configured.value() : std::round(1.0 / nominalFromDai(local));
        } else {
            const auto metadata = materialize(replayPath, index, name + ".mcap", probeDir);
            const auto metadataSize = BytePlayer::getVideoSize(metadata.string());
            if(!metadataSize.has_value()) throw std::runtime_error("ReplayVideo source size is not set");
            probe.width = static_cast<int>(std::get<0>(metadataSize.value()));
            probe.height = static_cast<int>(std::get<1>(metadataSize.value()));
            const auto configured = configuredStreamFps(recordConfig, name);
            probe.fps = configured.has_value() ? configured.value() : 30.0;
        }
        probes[name] = probe;
    }

    // The device cache stores one blob. A directory recording is packed into a tar for that upload.
    // The size probe above is a separate read and is not the cache upload.
    const auto sourcePath = archive ? replayPath : packDirectory(replayPath);
    std::filesystem::remove_all(probeDir);

    for(const auto& node : sources) {
        auto source = std::dynamic_pointer_cast<SourceNode>(node);
        const auto name = source->getNodeRecordParams().name;
        const auto layout = layouts[name];
        const bool camera = layout == HolisticLayout::Video || layout == HolisticLayout::Frames;
        if(camera) {
            auto replay = pipeline.create<dai::node::ReplayVideo>();
            replay->setSource(sourcePath);
            replay->setArchiveEntry(name);
            replay->setLoop(true);
            replay->setOutFrameType(legacy ? ImgFrame::Type::YUV420p : ImgFrame::Type::NV12);
            const auto& probe = probes[name];
            applyMockIspSize(node, probe.width, probe.height, probe.fps);
            replay->out.link(source->getReplayInput());
        } else {
            auto replay = pipeline.create<dai::node::ReplayMetadataOnly>();
            replay->setSource(sourcePath);
            replay->setArchiveEntry(name);
            replay->setLoop(true);
            if(probes.count(name) != 0) {
                const auto& probe = probes[name];
                applyMockIspSize(node, probe.width, probe.height, probe.fps);
            }
            replay->out.link(source->getReplayInput());
        }
    }
    return true;
}

}  // namespace

bool setupHolisticReplay(Pipeline pipeline,
                         std::filesystem::path replayPath,
                         const std::string& deviceId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::filesystem::path>& outFilenames,
                         bool legacy) {
    UNUSED(deviceId);
    if(!platform::checkPathExists(replayPath)) {
        spdlog::warn("Replay path does not exist: {}", replayPath.string());
        return false;
    }

    pipeline.getDefaultDevice()->mockCameraFeatures(replayPath);

    const bool onRvc4 = pipeline.getDefaultDevice()->getPlatform() == Platform::RVC4;
    if(onRvc4) return setupHolisticReplayDevice(pipeline, replayPath, legacy);

    auto sources = pipeline.getSourceNodes();
    try {
        bool useTar = !platform::checkPathExists(replayPath, true);
        std::vector<std::string> tarNodenames;
        std::string tarRoot = ".";
        std::string videoExt = ".mp4";
        std::filesystem::path rootPath = useTar ? platform::getTempPath() : replayPath;
        if(useTar)
            tarNodenames = filenamesInArchive(replayPath);
        else
            tarNodenames = platform::getFilenamesInDirectory(replayPath);
        bool hasMp4Files = std::any_of(tarNodenames.begin(), tarNodenames.end(), [](const std::string& path) {
            auto pathObj = std::filesystem::path(path);
            return pathObj.extension() == ".mp4";
        });
        bool hasAviFiles = std::any_of(tarNodenames.begin(), tarNodenames.end(), [](const std::string& path) {
            auto pathObj = std::filesystem::path(path);
            return pathObj.extension() == ".avi";
        });
        if(hasMp4Files && hasAviFiles) {
            throw std::runtime_error("Recording contains both .mp4 and .avi files.");
        } else if(hasMp4Files) {
            videoExt = ".mp4";
        } else if(hasAviFiles) {
            videoExt = ".avi";
        }
        tarNodenames.erase(std::remove_if(tarNodenames.begin(),
                                          tarNodenames.end(),
                                          [](const std::string& path) {
                                              auto pathObj = std::filesystem::path(path);
                                              return pathObj.extension() != ".mcap";
                                          }),
                           tarNodenames.end());
        if(useTar) tarRoot = tarNodenames.empty() ? "." : tarNodenames[0].substr(0, tarNodenames[0].find_last_of("/\\") + 1);
        for(auto& path : tarNodenames) {
            auto pathObj = std::filesystem::path(path);
            path = pathObj.stem().string();  // Remove extension
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
            // Needed for multi-device recordings, not yet supported
            // std::string nodeName = (deviceId + "_").append(nodeParams.name);
            nodeNames.push_back(nodeParams.name);
            pipelineFilenames.push_back(nodeParams.name);
        }
        std::vector<std::string> inFiles;
        std::vector<std::filesystem::path> outFiles;
        inFiles.reserve(sources.size() * 2);
        outFiles.reserve(sources.size() * 2);
        if(allMatch(pipelineFilenames, tarNodenames)) {
            for(auto& nodeName : nodeNames) {
                // auto filename = (deviceId + "_").append(nodeName);
                auto filename = nodeName;
                if(useTar) {
                    inFiles.push_back(tarRoot + filename + videoExt);
                    inFiles.push_back(tarRoot + filename + ".mcap");
                }
                std::filesystem::path filePath = platform::joinPaths(rootPath, filename);
                outFiles.push_back(std::filesystem::path(filePath).concat(videoExt));
                outFiles.push_back(std::filesystem::path(filePath).concat(".mcap"));
                outFilenames[nodeName] = filePath;
            }
            if(useTar) {
                extractFiles(replayPath, inFiles, outFiles);
            }
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
            // extractFiles(replayPath, inFiles, outFiles);
        }

        recordConfig.state = RecordConfig::RecordReplayState::REPLAY;

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
                replay->setReplayVideoFile(platform::joinPaths(rootPath, nodeName + videoExt));
                replay->setOutFrameType(legacy ? ImgFrame::Type::YUV420p : ImgFrame::Type::NV12);

                auto videoSize = BytePlayer::getVideoSize(replay->getReplayMetadataFile().string());
                auto [vidWidth, vidHeight, vidFps] = utility::getVideoSize(replay->getReplayVideoFile().string());
                if(cameraNode) {
                    cameraNode->properties.mockIspWidth = vidWidth;
                    cameraNode->properties.mockIspHeight = vidHeight;
                    cameraNode->properties.mockIspFps = vidFps;
                } else if(colorCameraNode) {
                    colorCameraNode->setMockIspSize(vidWidth, vidHeight);
                } else if(monoCameraNode) {
                    monoCameraNode->setMockIspSize(vidWidth, vidHeight);
                }
                if(videoSize.has_value()) {
                    auto [width, height] = videoSize.value();
                    if(width != vidWidth || height != vidHeight) {
                        throw std::runtime_error("Video size does not match metadata size for node " + nodeName);
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
