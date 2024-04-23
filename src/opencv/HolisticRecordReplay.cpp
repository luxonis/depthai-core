#include "../utility/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"
#include "depthai/utility/Compression.hpp"
#include "depthai/utility/RecordReplay.hpp"

namespace dai {
namespace utility {

bool setupHolisticRecord(Pipeline& pipeline,
                         std::string mxId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::string>& outFilenames) {
    auto sources = pipeline.getSourceNodes();
    auto recordPath = recordConfig.outputDir;
    try {
        for(auto& node : sources) {
            NodeRecordParams nodeParams = node->getNodeRecordParams();
            std::string nodeName = nodeParams.name;
            auto recordNode = pipeline.create<dai::node::Record>();
            std::string filePath = platform::joinPaths(recordPath, mxId.append("_").append(nodeName)).append(".mp4");
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
        outFilenames["record_recordConfig"] = platform::joinPaths(recordPath, mxId.append("_record_config.json"));
    } catch(const std::runtime_error& e) {
        recordConfig.state = RecordConfig::RecordReplayState::NONE;
        spdlog::warn("Record disabled: {}", e.what());
        return false;
    }
    // Write recordConfig to output dir
    try {
        std::ofstream file(Path(platform::joinPaths(recordPath, mxId.append("_record_config.json"))));
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
                         std::string mxId,
                         RecordConfig& recordConfig,
                         std::unordered_map<std::string, std::string>& outFilenames) {
    std::string rootPath = platform::getDirFromPath(replayPath);
    auto sources = pipeline.getSourceNodes();
    try {
        auto tarFilenames = filenamesInTar(replayPath);
        std::remove_if(tarFilenames.begin(), tarFilenames.end(), [](const std::string& filename) {
            return filename.size() < 4 || filename.substr(filename.size() - 4, filename.size()) == "meta";
        });
        std::vector<std::string> nodeNames;
        std::vector<std::string> pipelineFilenames;
        pipelineFilenames.reserve(sources.size());
        for(auto& node : sources) {
            NodeRecordParams nodeParams = node->getNodeRecordParams();
            std::string nodeName = mxId.append("_").append(nodeParams.name).append(".rec");
            pipelineFilenames.push_back(nodeName);
            nodeNames.push_back(nodeParams.name);
        }
        std::string configPath;
        std::vector<std::string> inFiles;
        std::vector<std::string> outFiles;
        inFiles.reserve(sources.size() + 1);
        outFiles.reserve(sources.size() + 1);
        if(allMatch(tarFilenames, pipelineFilenames)) {
            for(auto& nodeName : nodeNames) {
                auto filename = mxId.append("_").append(nodeName).append(".mp4");
                inFiles.push_back(filename);
                inFiles.push_back(filename + ".meta");
                std::string filePath = platform::joinPaths(rootPath, filename);
                outFiles.push_back(filePath);
                outFiles.push_back(filePath.append(".meta"));
                outFilenames[nodeName] = filePath;
            }
            inFiles.emplace_back("record_config.json");
            configPath = platform::joinPaths(rootPath, mxId.append("_record_config.json"));
            outFiles.push_back(configPath);
            outFilenames["record_config"] = configPath;
            untarFiles(replayPath, inFiles, outFiles);
        } else {
            std::vector<std::string> mxIds;
            mxIds.reserve(tarFilenames.size());
            for(auto& filename : tarFilenames) {
                mxIds.push_back(filename.substr(0, filename.find_first_of('_')));
            }
            // Get unique mxIds
            std::sort(mxIds.begin(), mxIds.end());
            mxIds.erase(std::unique(mxIds.begin(), mxIds.end()), mxIds.end());
            std::string mxIdRec = matchTo(mxIds, tarFilenames, nodeNames);
            if(mxIdRec.empty()) {
                throw std::runtime_error("No recordings match the pipeline configuration.");
            }
            for(auto& nodeName : nodeNames) {
                auto inFilename = mxIdRec.append("_").append(nodeName).append(".rec");
                auto outFilename = mxId.append("_").append(nodeName).append(".rec");
                inFiles.push_back(inFilename);
                inFiles.push_back(inFilename + ".meta");
                std::string filePath = platform::joinPaths(rootPath, outFilename);
                outFiles.push_back(filePath);
                outFiles.push_back(filePath.append(".meta"));
                outFilenames[nodeName] = filePath;
            }
            inFiles.emplace_back("record_config.json");
            configPath = platform::joinPaths(rootPath, mxId.append("_record_config.json"));
            outFiles.push_back(configPath);
            outFilenames["record_config"] = configPath;
            untarFiles(replayPath, inFiles, outFiles);
        }

        // FIXME(asahtik): If this fails, extracted files do not get removed
        std::ifstream file(configPath);
        json j = json::parse(file);
        recordConfig = j.get<RecordConfig>();
        recordConfig.state = RecordConfig::RecordReplayState::REPLAY;

        for(auto& node : sources) {
            NodeRecordParams nodeParams = node->getNodeRecordParams();
            std::string nodeName = nodeParams.name;
            auto replay = pipeline.create<dai::node::Replay>();
            replay->setReplayFile(platform::joinPaths(rootPath, mxId.append("_").append(nodeName).append(".mcap")));
            if(strcmp(node->getName(), "Camera") == 0 || strcmp(node->getName(), "ColorCamera") == 0 || strcmp(node->getName(), "MonoCamera") == 0) {
                replay->setReplayVideo(platform::joinPaths(rootPath, mxId.append("_").append(nodeName).append(".mp4")));
                replay->setOutFrameType(ImgFrame::Type::YUV420p);
            }
            replay->out.link(node->getReplayInput());
        }
        return true;
    } catch(const std::exception& e) {
        spdlog::warn("Replay disabled: {}", e.what());
    }
    return false;
}

}  // namespace utility
}  // namespace dai
