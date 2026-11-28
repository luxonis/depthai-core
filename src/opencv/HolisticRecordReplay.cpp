#include "../utility/HolisticRecordReplay.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
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
#include "utility/ReplayArchive.hpp"

#define UNUSED(x) (void)(x)

namespace dai {
namespace utility {

namespace {

std::string normalizeArchiveName(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    while(path.rfind("./", 0) == 0) path.erase(0, 2);
    return path;
}

bool hiddenRelative(const std::string& name) {
    std::size_t start = 0;
    while(start <= name.size()) {
        const auto slash = name.find('/', start);
        const auto part = name.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        if(!part.empty() && part.front() == '.') return true;
        if(slash == std::string::npos) break;
        start = slash + 1;
    }
    return false;
}

struct RecordingIndex {
    bool archive = false;
    std::vector<std::string> names;
    std::unordered_map<std::string, std::string> members;
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
    for(const auto& original : raw) {
        auto name = normalizeArchiveName(original);
        if(name.empty() || name.back() == '/') continue;
        if(hiddenRelative(name)) continue;
        if(!index.members.emplace(name, archive ? original : name).second) continue;
        index.names.push_back(std::move(name));
    }
    return index;
}

std::string archiveMember(const RecordingIndex& index, const std::string& name) {
    const auto found = index.members.find(name);
    if(found == index.members.end()) return name;
    return found->second;
}

struct RecordingFilePick {
    bool found = false;
    bool ambiguous = false;
    std::string name;
};

RecordingFilePick pickRecordingFile(const RecordingIndex& index, const std::string& filename) {
    RecordingFilePick pick;
    std::string root;
    std::size_t count = 0;
    for(const auto& name : index.names) {
        if(std::filesystem::path(name).filename() != filename) continue;
        ++count;
        if(name.find('/') == std::string::npos)
            root = name;
        else if(pick.name.empty())
            pick.name = name;
    }
    if(count == 0) return pick;
    if(!root.empty()) {
        pick.found = true;
        pick.name = root;
        return pick;
    }
    if(count == 1) {
        pick.found = true;
        return pick;
    }
    pick.ambiguous = true;
    pick.name.clear();
    spdlog::warn("More than one {} in replay path.", filename);
    return pick;
}

}  // namespace

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
    const auto index = indexRecording(replayPath, useTar);
    const auto info = pickRecordingFile(index, "camera_info.json");
    if(info.ambiguous) return false;
    const auto calibration = info.found ? RecordingFilePick{} : pickRecordingFile(index, "calibration.json");
    if(!info.found && calibration.ambiguous) return false;
    const bool hasCameraInfo = info.found;
    const auto chosen = hasCameraInfo ? info.name : calibration.name;
    if(!chosen.empty()) {
        std::vector<uint8_t> cameraInfoData;
        if(useTar) {
            try {
                cameraInfoData = readFileInArchive(replayPath, archiveMember(index, chosen));
            } catch(const std::exception& e) {
                spdlog::warn("Error while reading from tar file: {}", e.what());
                return false;
            }
        } else {
            std::filesystem::path cameraInfoPath = replayPath / chosen;
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

enum class HolisticLayout { Video, Frames, Mcap, Missing };

HolisticLayout layoutOf(const replay::NamedArchiveEntry& selected) {
    if(selected.kind == replay::NamedArchiveEntry::Kind::Video) return HolisticLayout::Video;
    if(selected.kind == replay::NamedArchiveEntry::Kind::Frames) return HolisticLayout::Frames;
    if(selected.kind == replay::NamedArchiveEntry::Kind::Mcap) return HolisticLayout::Mcap;
    return HolisticLayout::Missing;
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
        bytes = readFileInArchive(replayPath, archiveMember(index, configName));
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
        std::filesystem::create_directories(destination.parent_path());
        extractFiles(replayPath, {archiveMember(index, entry)}, {destination});
        return destination;
    }
    return replayPath / entry;
}

double nominalFromDai(const std::filesystem::path& first, const std::filesystem::path& last, std::size_t frameCount) {
    if(frameCount < 2) return 1.0 / 30.0;
#ifndef DEPTHAI_ENABLE_PROTOBUF
    (void)first;
    (void)last;
    throw std::runtime_error("ReplayVideo node requires protobuf support");
#else
    ImgFrame firstFrame;
    ImgFrame lastFrame;
    firstFrame.load(first);
    lastFrame.load(last);
    const auto delta = lastFrame.getTimestampDevice() - firstFrame.getTimestampDevice();
    const auto seconds = std::chrono::duration<double>(delta).count() / static_cast<double>(frameCount - 1);
    return seconds > 0.0 ? seconds : 1.0 / 30.0;
#endif
}

struct MediaProbe {
    int width = 0;
    int height = 0;
    double fps = 30.0;
};

MediaProbe probeFrameMedia(const std::vector<std::filesystem::path>& local,
                           const std::string& extension,
                           const std::optional<nlohmann::json>& config,
                           const std::string& name,
                           std::size_t frameCount) {
    MediaProbe probe;
    if(local.empty()) throw std::runtime_error("Recording does not match the pipeline configuration.");
    if(extension == ".dai") {
#ifndef DEPTHAI_ENABLE_PROTOBUF
        (void)config;
        (void)name;
        (void)frameCount;
        throw std::runtime_error("ReplayVideo node requires protobuf support");
#else
        ImgFrame first;
        first.load(local.front());
        probe.width = static_cast<int>(first.getWidth());
        probe.height = static_cast<int>(first.getHeight());
        const auto configured = configuredStreamFps(config, name);
        probe.fps =
            configured.has_value() ? static_cast<double>(configured.value()) : std::round(1.0 / nominalFromDai(local.front(), local.back(), frameCount));
#endif
    } else {
        const auto image = cv::imread(local.front().string(), cv::IMREAD_UNCHANGED | cv::IMREAD_IGNORE_ORIENTATION);
        if(image.empty()) throw std::runtime_error("ReplayVideo image cannot be read: " + local.front().string());
        probe.width = image.cols;
        probe.height = image.rows;
        const auto configured = configuredStreamFps(config, name);
        probe.fps = configured.has_value() ? static_cast<double>(configured.value()) : 30.0;
    }
    return probe;
}

std::filesystem::path packDirectory(const std::filesystem::path& directory, const std::filesystem::path& scratch) {
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
    auto destination = scratch / (directory.filename().string() + ".tar");
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
    const auto scratch = replay::uniqueReplayTempDirectory();
    const auto probeDir = scratch / "probe";
    std::filesystem::create_directories(probeDir);
    struct ProbeCleanup {
        std::filesystem::path path;
        ~ProbeCleanup() {
            std::error_code error;
            std::filesystem::remove_all(path, error);
        }
    } probeCleanup{probeDir};

    std::unordered_map<std::string, MediaProbe> probes;
    std::unordered_map<std::string, HolisticLayout> layouts;
    std::unordered_map<std::string, replay::NamedArchiveEntry> selectedByName;

    auto sources = pipeline.getSourceNodes();
    for(const auto& node : sources) {
        auto source = std::dynamic_pointer_cast<SourceNode>(node);
        if(source == nullptr) throw std::runtime_error("Node is listed as a source node but does not implement the SourceNode interface.");
        const auto name = source->getNodeRecordParams().name;
        const bool camera = std::dynamic_pointer_cast<node::Camera>(node) || std::dynamic_pointer_cast<node::ColorCamera>(node)
                            || std::dynamic_pointer_cast<node::MonoCamera>(node);
        const auto selected = replay::selectNamedArchiveEntry(index.names, name, !camera);
        const auto layout = layoutOf(selected);
        if(!camera) {
            if(layout != HolisticLayout::Mcap) throw std::runtime_error("Recording does not match the pipeline configuration.");
            layouts[name] = HolisticLayout::Mcap;
            selectedByName[name] = selected;
            continue;
        }
        if(layout == HolisticLayout::Missing) throw std::runtime_error("Recording does not match the pipeline configuration.");
        layouts[name] = layout;
        selectedByName[name] = selected;
        MediaProbe probe;
        if(layout == HolisticLayout::Video) {
            const auto video = materialize(replayPath, index, selected.path, probeDir);
            const auto [width, height, fps] = utility::getVideoSize(video.string());
            probe.width = static_cast<int>(width);
            probe.height = static_cast<int>(height);
            probe.fps = fps;
            if(selected.hasMetadata) {
                const auto metadata = materialize(replayPath, index, selected.metadataPath, probeDir);
                const auto metadataSize = BytePlayer::getVideoSize(metadata.string());
                if(metadataSize.has_value()) {
                    const auto [metaWidth, metaHeight] = metadataSize.value();
                    if(static_cast<int>(metaWidth) != probe.width || static_cast<int>(metaHeight) != probe.height) {
                        throw std::runtime_error("Video size does not match metadata size for node " + name);
                    }
                }
            }
        } else if(layout == HolisticLayout::Frames) {
            if(selected.frames.empty()) throw std::runtime_error("Recording does not match the pipeline configuration.");
            std::vector<std::filesystem::path> local;
            local.push_back(materialize(replayPath, index, selected.frames.front(), probeDir));
            if(selected.extension == ".dai" && selected.frames.size() > 1) {
                local.push_back(materialize(replayPath, index, selected.frames.back(), probeDir));
            }
            probe = probeFrameMedia(local, selected.extension, recordConfig, name, selected.frames.size());
        } else {
            const auto metadata = materialize(replayPath, index, selected.path, probeDir);
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
    const auto sourcePath = archive ? replayPath : packDirectory(replayPath, scratch);

    for(const auto& node : sources) {
        auto source = std::dynamic_pointer_cast<SourceNode>(node);
        const auto name = source->getNodeRecordParams().name;
        const auto layout = layouts[name];
        const auto archiveEntry = selectedByName[name].archiveEntry;
        const bool camera = layout == HolisticLayout::Video || layout == HolisticLayout::Frames;
        if(camera) {
            auto replay = pipeline.create<dai::node::ReplayVideo>();
            replay->setSource(sourcePath);
            replay->setArchiveEntry(archiveEntry);
            replay->setLoop(true);
            replay->setOutFrameType(legacy ? ImgFrame::Type::YUV420p : ImgFrame::Type::NV12);
            const auto& probe = probes[name];
            applyMockIspSize(node, probe.width, probe.height, probe.fps);
            replay->out.link(source->getReplayInput());
        } else {
            auto replay = pipeline.create<dai::node::ReplayMetadataOnly>();
            replay->setSource(sourcePath);
            replay->setArchiveEntry(archiveEntry);
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
        const bool useTar = !platform::checkPathExists(replayPath, true);
        const auto index = indexRecording(replayPath, useTar);
        const auto savedConfig = readRecordConfig(replayPath, index);
        const auto scratch = useTar ? replay::uniqueReplayTempDirectory() : replayPath;
        auto place = [&](const std::vector<std::string>& relatives) {
            std::vector<std::filesystem::path> placed;
            placed.reserve(relatives.size());
            if(!useTar) {
                for(const auto& relative : relatives) placed.push_back(replayPath / relative);
                return placed;
            }
            std::vector<std::string> members;
            members.reserve(relatives.size());
            for(const auto& relative : relatives) {
                auto destination = scratch / relative;
                std::filesystem::create_directories(destination.parent_path());
                members.push_back(archiveMember(index, relative));
                placed.push_back(destination);
            }
            extractFiles(replayPath, members, placed);
            for(const auto& path : placed) {
                if(!std::filesystem::exists(path)) throw std::runtime_error("Recording does not match the pipeline configuration.");
            }
            return placed;
        };

        struct PlannedSource {
            std::shared_ptr<Node> node;
            std::string name;
            bool camera = false;
            replay::NamedArchiveEntry selected;
            HolisticLayout layout = HolisticLayout::Missing;
        };
        std::vector<PlannedSource> planned;
        planned.reserve(sources.size());
        for(auto& node : sources) {
            auto nodeS = std::dynamic_pointer_cast<SourceNode>(node);
            if(nodeS == nullptr) throw std::runtime_error("Node is listed as a source node but does not implement the SourceNode interface.");
            PlannedSource item;
            item.node = node;
            item.name = nodeS->getNodeRecordParams().name;
            DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
            item.camera = std::dynamic_pointer_cast<node::Camera>(node) || std::dynamic_pointer_cast<node::ColorCamera>(node)
                          || std::dynamic_pointer_cast<node::MonoCamera>(node);
            DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING
            item.selected = replay::selectNamedArchiveEntry(index.names, item.name, !item.camera);
            item.layout = layoutOf(item.selected);
            if(item.layout == HolisticLayout::Missing) throw std::runtime_error("Recording does not match the pipeline configuration.");
            if(!item.camera && item.layout != HolisticLayout::Mcap) throw std::runtime_error("Recording does not match the pipeline configuration.");
            if(item.layout == HolisticLayout::Frames && item.selected.frames.empty()) {
                throw std::runtime_error("Recording does not match the pipeline configuration.");
            }
            planned.push_back(std::move(item));
        }

        std::unordered_map<std::string, std::filesystem::path> published;
        for(const auto& item : planned) {
            auto nodeS = std::dynamic_pointer_cast<SourceNode>(item.node);
            const auto& nodeName = item.name;
            const auto& selected = item.selected;
            const auto layout = item.layout;
            if(!item.camera || layout == HolisticLayout::Mcap) {
                const auto metadata = place({selected.path}).front();
                auto replay = pipeline.create<dai::node::ReplayMetadataOnly>();
                replay->setReplayFile(metadata);
                if(item.camera) {
                    const auto metadataSize = BytePlayer::getVideoSize(metadata.string());
                    if(!metadataSize.has_value()) throw std::runtime_error("ReplayVideo source size is not set");
                    const auto configured = configuredStreamFps(savedConfig, nodeName);
                    const double fps = configured.has_value() ? static_cast<double>(configured.value()) : 30.0;
                    applyMockIspSize(item.node, static_cast<int>(std::get<0>(metadataSize.value())), static_cast<int>(std::get<1>(metadataSize.value())), fps);
                }
                published.emplace(nodeName, metadata.parent_path() / metadata.stem());
                replay->out.link(nodeS->getReplayInput());
                continue;
            }
            if(layout == HolisticLayout::Video) {
                std::vector<std::string> relatives{selected.path};
                if(selected.hasMetadata) relatives.push_back(selected.metadataPath);
                const auto placed = place(relatives);
                const auto& video = placed.front();
                auto replay = pipeline.create<dai::node::ReplayVideo>();
                replay->setReplayVideoFile(video);
                if(selected.hasMetadata) replay->setReplayMetadataFile(placed[1]);
                replay->setOutFrameType(legacy ? ImgFrame::Type::YUV420p : ImgFrame::Type::NV12);
                const auto [vidWidth, vidHeight, vidFps] = utility::getVideoSize(video.string());
                if(selected.hasMetadata) {
                    const auto videoSize = BytePlayer::getVideoSize(placed[1].string());
                    if(videoSize.has_value()) {
                        const auto [width, height] = videoSize.value();
                        if(width != vidWidth || height != vidHeight) {
                            throw std::runtime_error("Video size does not match metadata size for node " + nodeName);
                        }
                    }
                }
                applyMockIspSize(item.node, static_cast<int>(vidWidth), static_cast<int>(vidHeight), vidFps);
                published.emplace(nodeName, video.parent_path() / video.stem());
                replay->out.link(nodeS->getReplayInput());
                continue;
            }
            const auto placed = place(selected.frames);
            if(placed.empty()) throw std::runtime_error("Recording does not match the pipeline configuration.");
            std::vector<std::filesystem::path> samples{placed.front()};
            if(selected.extension == ".dai" && placed.size() > 1) samples.push_back(placed.back());
            const auto probed = probeFrameMedia(samples, selected.extension, savedConfig, nodeName, selected.frames.size());
            auto replay = pipeline.create<dai::node::ReplayVideo>();
            replay->setReplayVideoFile(placed.front().parent_path());
            replay->setOutFrameType(legacy ? ImgFrame::Type::YUV420p : ImgFrame::Type::NV12);
            applyMockIspSize(item.node, probed.width, probed.height, probed.fps);
            published.emplace(nodeName, placed.front().parent_path());
            replay->out.link(nodeS->getReplayInput());
        }
        for(const auto& entry : published) outFilenames[entry.first] = entry.second;
        recordConfig.state = RecordConfig::RecordReplayState::REPLAY;
        return true;
    } catch(const std::exception& e) {
        spdlog::warn("Replay disabled: {}", e.what());
    }
    return false;
}

}  // namespace utility
}  // namespace dai
