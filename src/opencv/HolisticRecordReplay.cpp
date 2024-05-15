#include "../utility/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"
#include "depthai/utility/Compression.hpp"
#include "depthai/utility/RecordReplay.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"

namespace dai {
namespace utility {

bool setupHolisticRecord(Pipeline& pipeline, const std::string& mxId, RecordConfig& recordConfig, std::unordered_map<std::string, std::string>& outFilenames) {
    auto sources = pipeline.getSourceNodes();
    const auto recordPath = recordConfig.outputDir;
    try {
        for(auto& node : sources) {
            NodeRecordParams nodeParams = node->getNodeRecordParams();
            std::string nodeName = nodeParams.name;
            auto recordNode = pipeline.create<dai::node::Record>();
            std::string filePath = platform::joinPaths(recordPath, (mxId + "_").append(nodeName));
            recordNode->setRecordFile(filePath);
            recordNode->setCompressionLevel((dai::node::Record::CompressionLevel)recordConfig.compressionLevel);
            outFilenames[nodeName] = filePath;
            if(strcmp(node->getName(), "Camera") == 0 || strcmp(node->getName(), "ColorCamera") == 0 || strcmp(node->getName(), "MonoCamera") == 0) {
                if(recordConfig.videoEncoding.enabled) {
                    auto imageManip = pipeline.create<dai::node::ImageManip>();
                    imageManip->initialConfig.setFrameType(ImgFrame::Type::NV12);
                    imageManip->setMaxOutputFrameSize(3110400);  // TODO(asahtik): set size depending on isp size
                    auto videnc = pipeline.create<dai::node::VideoEncoder>();
                    videnc->setProfile(recordConfig.videoEncoding.profile);
                    videnc->setLossless(recordConfig.videoEncoding.lossless);
                    videnc->setBitrate(recordConfig.videoEncoding.bitrate);
                    videnc->setQuality(recordConfig.videoEncoding.quality);

                    node->getRecordOutput().link(imageManip->inputImage);
                    imageManip->out.link(videnc->input);
                    videnc->out.link(recordNode->input);
                } else {
                    node->getRecordOutput().link(recordNode->input);
                }
            } else {
                node->getRecordOutput().link(recordNode->input);
            }
        }
        outFilenames["record_config"] = platform::joinPaths(recordPath, mxId + "_record_config.json");
    } catch(const std::runtime_error& e) {
        recordConfig.state = RecordConfig::RecordReplayState::NONE;
        spdlog::warn("Record disabled: {}", e.what());
        return false;
    }
    // Write recordConfig to output dir
    try {
        std::ofstream file(Path(platform::joinPaths(recordPath, mxId + "_record_config.json")));
        json j = recordConfig;
        file << j.dump(4);
    } catch(const std::exception& e) {
        spdlog::warn("Error while writing DEPTHAI_RECORD json file: {}", e.what());
        return false;
    }
    return true;
}

bool setupHolisticReplay(Pipeline& pipeline,
                         std::string replayPath,
                         const std::string& mxId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::string>& outFilenames) {
    const std::string rootPath = platform::getDirFromPath(replayPath);
    auto sources = pipeline.getSourceNodes();
    try {
        auto tarNodenames = filenamesInTar(replayPath);
        tarNodenames.erase(std::remove_if(tarNodenames.begin(),
                                          tarNodenames.end(),
                                          [](const std::string& path) {
                                              auto pathDelim = path.find_last_of("/\\");
                                              auto filename = pathDelim == std::string::npos ? path : path.substr(path.find_last_of("/\\") + 1);
                                              return filename.size() < 4 || filename.substr(filename.size() - 4, filename.size()) == "mcap"
                                                     || filename == "record_config.json";
                                          }),
                           tarNodenames.end());

        std::string tarRoot = tarNodenames.empty() ? "" : tarNodenames[0].substr(0, tarNodenames[0].find_first_of("/\\") + 1);
        for(auto& path : tarNodenames) {
            auto pathDelim = path.find_last_of("/\\");
            path = pathDelim == std::string::npos ? path : path.substr(path.find_last_of("/\\") + 1);
            path = path.substr(0, path.find_last_of('.'));
        }

        std::vector<std::string> nodeNames;
        std::vector<std::string> pipelineFilenames;
        pipelineFilenames.reserve(sources.size());
        for(auto& node : sources) {
            NodeRecordParams nodeParams = node->getNodeRecordParams();
            // Needed for muti-device recordings, not yet supported
            // std::string nodeName = (mxId + "_").append(nodeParams.name);
            std::string nodeName = nodeParams.name;
            pipelineFilenames.push_back(nodeName);
            nodeNames.push_back(nodeParams.name);
        }
        std::string configPath;
        std::vector<std::string> inFiles;
        std::vector<std::string> outFiles;
        inFiles.reserve(sources.size() + 1);
        outFiles.reserve(sources.size() + 1);
        if(allMatch(tarNodenames, pipelineFilenames)) {
            for(auto& nodeName : nodeNames) {
                // auto filename = (mxId + "_").append(nodeName);
                auto filename = nodeName;
                inFiles.push_back(tarRoot + filename + ".mp4");
                inFiles.push_back(tarRoot + filename + ".mcap");
                std::string filePath = platform::joinPaths(rootPath, filename);
                outFiles.push_back(filePath + ".mp4");
                outFiles.push_back(filePath + ".mcap");
                outFilenames[nodeName] = filePath;
            }
            inFiles.emplace_back(tarRoot + "record_config.json");
            configPath = platform::joinPaths(rootPath, "record_config.json");
            outFiles.push_back(configPath);
            outFilenames["record_config"] = configPath;
            untarFiles(replayPath, inFiles, outFiles);
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

        // FIXME(asahtik): If this fails, extracted files do not get removed
        std::ifstream file(configPath);
        json j = json::parse(file);
        recordConfig = j.get<RecordConfig>();
        recordConfig.state = RecordConfig::RecordReplayState::REPLAY;

        for(auto& node : sources) {
            NodeRecordParams nodeParams = node->getNodeRecordParams();
            std::string nodeName = nodeParams.name;
            if(strcmp(node->getName(), "Camera") == 0 || strcmp(node->getName(), "ColorCamera") == 0 || strcmp(node->getName(), "MonoCamera") == 0) {
                auto replay = pipeline.create<dai::node::ReplayVideo>();
                // replay->setReplayFile(platform::joinPaths(rootPath, (mxId + "_").append(nodeName).append(".mcap")));
                replay->setReplayMetadataFile(platform::joinPaths(rootPath, nodeName + ".mcap"));
                // replay->setReplayVideo(platform::joinPaths(rootPath, (mxId + "_").append(nodeName).append(".mp4")));
                replay->setReplayVideo(platform::joinPaths(rootPath, nodeName + ".mp4"));
                replay->setOutFrameType(ImgFrame::Type::YUV420p);

                auto videoSize = BytePlayer::getVideoSize(replay->getReplayMetadataFile());
                if (videoSize.has_value()) {
                    auto [width, height] = videoSize.value();
                    if(strcmp(node->getName(), "Camera") == 0) {
                        auto cam = std::dynamic_pointer_cast<dai::node::Camera>(node);
                        cam->setMockIspSize(width, height);
                    } else if (strcmp(node->getName(), "ColorCamera") == 0) {
                        auto cam = std::dynamic_pointer_cast<dai::node::ColorCamera>(node);
                        cam->setMockIspSize(width, height);
                    } else if (strcmp(node->getName(), "MonoCamera") == 0) {
                        auto cam = std::dynamic_pointer_cast<dai::node::MonoCamera>(node);
                        cam->setMockIspSize(width, height);
                    }
                }
                replay->out.link(node->getReplayInput());
            } else {
                auto replay = pipeline.create<dai::node::ReplayMessage>();
                replay->setReplayFile(platform::joinPaths(rootPath, nodeName + ".mcap"));
                replay->out.link(node->getReplayInput());
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
