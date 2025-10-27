#include "depthai/utility/RecordReplay.hpp"

#include <spdlog/spdlog.h>

#include <mcap/types.hpp>
#include <optional>
#include <stdexcept>

#include "Environment.hpp"
#include "RecordReplayImpl.hpp"
#include "build/version.hpp"
#include "utility/Compression.hpp"
#include "utility/Platform.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include <google/protobuf/descriptor.pb.h>

    #include "depthai/schemas/ImgFrame.pb.h"
#endif

namespace dai {
namespace utility {

#ifdef DEPTHAI_ENABLE_PROTOBUF
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
#endif

void ByteRecorder::setWriter(const std::string& filePath, RecordConfig::CompressionLevel compressionLevel) {
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

ByteRecorder::~ByteRecorder() {
    close();
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

std::string BytePlayer::init(const std::string& filePath) {
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
    if(messageView->begin() == messageView->end()) {
        throw std::runtime_error("No messages in file");
    }
    it = std::make_unique<mcap::LinearMessageView::Iterator>(messageView->begin());
    initialized = true;
    return (*it)->schema->name;
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
#ifdef DEPTHAI_ENABLE_PROTOBUF
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
        proto::img_frame::ImgFrame adatatype;
        if(!adatatype.ParseFromArray(reinterpret_cast<const char*>(msg->message.data), msg->message.dataSize)) {
            throw std::runtime_error("Failed to parse protobuf message");
        }

        return std::make_tuple(adatatype.fb().width(), adatatype.fb().height());
    }
    return std::nullopt;
#else
    // Avoid warning for an unused parameter
    (void)filePath;
    throw std::runtime_error("BytePlayer::getVideoSize requires protobuf support");
#endif
}

bool checkRecordConfig(std::filesystem::path& recordPath, RecordConfig& config) {
    if(!platform::checkPathExists(recordPath)) {
        spdlog::warn("DEPTHAI_RECORD path does not exist or is invalid. Record disabled.");
        return false;
    }
    if(platform::checkPathExists(recordPath, true)) {
        // Is a directory
        config.outputDir = recordPath;
    } else {
        // Is a file
        if(recordPath.extension() != ".json") {
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
std::string matchTo(const std::vector<std::string>& deviceIds, const std::vector<std::string>& filenames, const std::vector<std::string>& nodenames) {
    std::string deviceId = "";
    for(const auto& id : deviceIds) {
        std::vector<std::string> matches;
        for(const auto& filename : filenames) {
            if(filename.size() >= 4 && filename.substr(filename.size() - 4, filename.size()) != "meta" && filename.find(id) != std::string::npos) {
                matches.push_back(filename.substr(id.size() + 1, filename.find_last_of('.') - id.size() - 1));
            }
        }
        if(matches.size() == nodenames.size()) {
            if(allMatch(matches, nodenames)) {
                if(deviceId.empty()) {
                    deviceId = id;
                } else {
                    throw std::runtime_error("Multiple recordings match the pipeline configuration - unsupported.");
                }
            }
        }
    }
    return deviceId;
}

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
std::tuple<size_t, size_t> getVideoSize(const std::string& filePath) {
    (void)filePath;
    throw std::runtime_error("OpenCV is required to get video size");
}
#endif

}  // namespace utility
}  // namespace dai
