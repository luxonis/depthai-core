#include "depthai/utility/RecordReplay.hpp"

#include <google/protobuf/descriptor.pb.h>
#include <spdlog/spdlog.h>

#include <mcap/types.hpp>
#include <optional>
#include <stdexcept>

#include "Environment.hpp"
#include "RecordReplayImpl.hpp"
#include "build/version.hpp"
#include "depthai/schemas/ADatatype.pb.h"
#include "utility/Compression.hpp"
#include "utility/Platform.hpp"

namespace dai {
namespace utility {

// Recursively adds all `fd` dependencies to `fd_set`.
void fdSetInternal(google::protobuf::FileDescriptorSet& fd_set, std::unordered_set<std::string>& files, const google::protobuf::FileDescriptor* fd) {
    for(int i = 0; i < fd->dependency_count(); ++i) {
        const auto* dep = fd->dependency(i);
        auto [_, inserted] = files.insert(dep->name());
        if(!inserted) continue;
        fdSetInternal(fd_set, files, fd->dependency(i));
    }
    fd->CopyTo(fd_set.add_file());
}

// Returns a serialized google::protobuf::FileDescriptorSet containing
// the necessary google::protobuf::FileDescriptor's to describe d.
std::string fdSet(const google::protobuf::Descriptor* d) {
    std::string res;
    std::unordered_set<std::string> files;
    google::protobuf::FileDescriptorSet fd_set;
    fdSetInternal(fd_set, files, d->file());
    return fd_set.SerializeAsString();
}

mcap::Schema createSchema(const google::protobuf::Descriptor* d) {
    mcap::Schema schema(d->full_name(), "protobuf", fdSet(d));
    return schema;
}

ByteRecorder::~ByteRecorder() {
    close();
}

void ByteRecorder::init(const std::string& filePath, RecordConfig::CompressionLevel compressionLevel, const std::string& channelName) {
    if(initialized) {
        throw std::runtime_error("ByteRecorder already initialized");
    }
    if(filePath.empty()) {
        throw std::runtime_error("ByteRecorder file path is empty");
    }
    {
        auto options = mcap::McapWriterOptions("protobuf");
        options.library = "depthai" + std::string(build::VERSION);
        switch(compressionLevel) {
            case RecordConfig::CompressionLevel::NONE:
                options.compression = mcap::Compression::None;
                break;
            case RecordConfig::CompressionLevel::FASTEST:
                options.compressionLevel = mcap::CompressionLevel::Fastest;
                break;
            case RecordConfig::CompressionLevel::FAST:
                options.compressionLevel = mcap::CompressionLevel::Fast;
                break;
            case RecordConfig::CompressionLevel::DEFAULT:
                options.compressionLevel = mcap::CompressionLevel::Default;
                break;
            case RecordConfig::CompressionLevel::SLOW:
                options.compressionLevel = mcap::CompressionLevel::Slow;
                break;
            case RecordConfig::CompressionLevel::SLOWEST:
                options.compressionLevel = mcap::CompressionLevel::Slowest;
                break;
        }
        options.compression = mcap::Compression::Lz4;
        const auto res = writer.open(filePath, options);
        if(!res.ok()) {
            throw std::runtime_error("Failed to open file for writing: " + res.message);
        }
    }
    {
        mcap::Schema schema = createSchema(dai::proto::adatatype::ADatatype::descriptor());
        writer.addSchema(schema);
        mcap::Channel channel(channelName, "protobuf", schema.id);
        writer.addChannel(channel);
        channelId = channel.id;
    }

    initialized = true;
}

void ByteRecorder::close() {
    if(initialized) {
        writer.close();
        initialized = false;
    }
}

BytePlayer::~BytePlayer() {
    close();
}

void BytePlayer::init(const std::string& filePath) {
    if(initialized) {
        throw std::runtime_error("BytePlayer already initialized");
    }
    if(filePath.empty()) {
        throw std::runtime_error("BytePlayer file path is empty");
    }
    {
        const auto res = reader.open(filePath);
        if(!res.ok()) {
            throw std::runtime_error("Failed to open file for reading: " + res.message);
        }
    }
    messageView = std::make_unique<mcap::LinearMessageView>(reader.readMessages());
    it = std::make_unique<mcap::LinearMessageView::Iterator>(messageView->begin());
    initialized = true;
}

std::optional<proto::adatatype::ADatatype> BytePlayer::next() {
    if(!initialized) {
        throw std::runtime_error("BytePlayer not initialized");
    }
    if(*it == messageView->end()) return std::nullopt;
    if((*it)->channel->messageEncoding != "protobuf") {
        throw std::runtime_error("Unsupported message encoding: " + (*it)->channel->messageEncoding);
    }

    proto::adatatype::ADatatype adatatype;
    if(!adatatype.ParseFromArray(reinterpret_cast<const char*>((*it)->message.data), (*it)->message.dataSize)) {
        throw std::runtime_error("Failed to parse protobuf message");
    }

    ++(*it);

    return adatatype;
}

void BytePlayer::restart() {
    if(!initialized) {
        throw std::runtime_error("BytePlayer not initialized");
    }
    it = std::make_unique<mcap::LinearMessageView::Iterator>(messageView->begin());
}

void BytePlayer::close() {
    if(initialized) {
        reader.close();
        initialized = false;
    }
}

std::optional<std::tuple<uint32_t, uint32_t>> BytePlayer::getVideoSize(const std::string& filePath) {
    if(filePath.empty()) {
        throw std::runtime_error("File path is empty in BytePlayer::getVideoSize");
    }
    mcap::McapReader reader;
    {
        const auto res = reader.open(filePath);
        if(!res.ok()) {
            throw std::runtime_error("Failed to open file for reading: " + res.message);
        }
    }
    auto messageView = reader.readMessages();
    if(messageView.begin() == messageView.end()) {
        return std::nullopt;
    } else {
        auto msg = messageView.begin();
        if(msg->channel->messageEncoding != "protobuf") {
            throw std::runtime_error("Unsupported message encoding: " + msg->channel->messageEncoding);
        }
        proto::adatatype::ADatatype adatatype;
        if(!adatatype.ParseFromArray(reinterpret_cast<const char*>(msg->message.data), msg->message.dataSize)) {
            throw std::runtime_error("Failed to parse protobuf message");
        }

        switch(adatatype.data_case()) {
            case proto::adatatype::ADatatype::kEncodedFrame:
                return std::make_tuple(adatatype.encodedframe().width(), adatatype.encodedframe().height());
            case proto::adatatype::ADatatype::kImgFrame:
                return std::make_tuple(adatatype.imgframe().fb().width(), adatatype.imgframe().fb().height());
            case proto::adatatype::ADatatype::kImuData:
            case proto::adatatype::ADatatype::kImageAnnotations:
            case proto::adatatype::ADatatype::kImgDetections:
            case proto::adatatype::ADatatype::kPointCloudData:
            case proto::adatatype::ADatatype::kSpatialImgDetections:
            case proto::adatatype::ADatatype::DATA_NOT_SET:
                return std::nullopt;
                break;
        }
    }
    return std::nullopt;
}

bool checkRecordConfig(std::string& recordPath, RecordConfig& config) {
    if(!platform::checkPathExists(recordPath)) {
        spdlog::warn("DEPTHAI_RECORD path does not exist or is invalid. Record disabled.");
        return false;
    }
    if(platform::checkPathExists(recordPath, true)) {
        // Is a directory
        config.outputDir = recordPath;
    } else {
        // Is a file
        std::string ext = recordPath.substr(recordPath.find_last_of('.') + 1);
        if(ext != "json") {
            spdlog::warn("DEPTHAI_RECORD path is not a directory or a json file. Record disabled.");
            return false;
        }
        try {
            std::ifstream file(recordPath);
            json j = json::parse(file);
            config = j.get<RecordConfig>();

            if(platform::checkPathExists(config.outputDir, true)) {
                // Is a directory
                recordPath = config.outputDir;
            } else {
                spdlog::warn("DEPTHAI_RECORD outputDir is not a directory. Record disabled.");
                return false;
            }
        } catch(const std::exception& e) {
            spdlog::warn("Error while processing DEPTHAI_RECORD json file: {}. Record disabled.", e.what());
            return false;
        }
    }
    return true;
}

bool allMatch(const std::vector<std::string>& v1, const std::vector<std::string>& v2) {
    for(const auto& el : v1) {
        if(std::find(v2.begin(), v2.end(), el) == v2.end()) return false;
    }
    return true;
}
std::string matchTo(const std::vector<std::string>& mxIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames) {
    std::string mxId = "";
    for(const auto& id : mxIds) {
        std::vector<std::string> matches;
        for(const auto& filename : filenames) {
            if(filename.size() >= 4 && filename.substr(filename.size() - 4, filename.size()) != "meta" && filename.find(id) != std::string::npos) {
                matches.push_back(filename.substr(id.size() + 1, filename.find_last_of('.') - id.size() - 1));
            }
        }
        if(matches.size() == nodenames.size()) {
            if(allMatch(matches, nodenames)) {
                if(mxId.empty()) {
                    mxId = id;
                } else {
                    throw std::runtime_error("Multiple recordings match the pipeline configuration - unsupported.");
                }
            }
        }
    }
    return mxId;
}

}  // namespace utility
}  // namespace dai
