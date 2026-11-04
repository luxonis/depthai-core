#include "ReplayArchive.hpp"

#include <archive.h>
#include <archive_entry.h>

#include <array>
#include <cctype>
#include <algorithm>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace dai {
namespace replay {
namespace {

bool escapesArchive(const std::filesystem::path& path) {
    if(path.empty() || path.is_absolute() || path.has_root_name()) return true;
    for(const auto& part : path) {
        if(part == "..") return true;
    }
    return false;
}

std::filesystem::path memberPath(const char* name) {
    if(name == nullptr || escapesArchive(name)) throw std::runtime_error("ReplayVideo archive entry escapes: " + std::string(name == nullptr ? "" : name));
    return std::filesystem::path(name).lexically_normal();
}

void checkOutputPath(const std::filesystem::path& destination, const std::filesystem::path& relative) {
    auto path = destination;
    if(std::filesystem::is_symlink(std::filesystem::symlink_status(path))) throw std::runtime_error("ReplayVideo archive destination is a symlink");
    for(const auto& part : relative) {
        path /= part;
        if(std::filesystem::is_symlink(std::filesystem::symlink_status(path))) throw std::runtime_error("ReplayVideo archive entry is a symlink: " + relative.string());
    }
}

std::uint64_t extractFile(struct archive* reader, const std::filesystem::path& output) {
    std::filesystem::create_directories(output.parent_path());
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(output, std::ios::binary | std::ios::trunc);
    std::array<char, 8192> buffer{};
    std::uint64_t total = 0;
    la_ssize_t bytes = 0;
    while((bytes = archive_read_data(reader, buffer.data(), buffer.size())) > 0) {
        out.write(buffer.data(), static_cast<std::streamsize>(bytes));
        total += static_cast<std::uint64_t>(bytes);
    }
    if(bytes < 0) throw std::runtime_error("ReplayVideo archive entry cannot be extracted: " + output.string());
    out.close();
    return total;
}

}  // namespace

ArchiveListing extractReplayArchive(const std::filesystem::path& archivePath, const std::filesystem::path& destination) {
    const std::unique_ptr<struct archive, decltype(&archive_read_free)> reader(archive_read_new(), archive_read_free);
    if(!reader) throw std::runtime_error("ReplayVideo archive cannot be opened: " + archivePath.string());
    archive_read_support_filter_all(reader.get());
    archive_read_support_format_all(reader.get());
#ifdef _WIN32
    const auto opened = archive_read_open_filename_w(reader.get(), archivePath.c_str(), 10240);
#else
    const auto opened = archive_read_open_filename(reader.get(), archivePath.c_str(), 10240);
#endif
    if(opened != ARCHIVE_OK) throw std::runtime_error("ReplayVideo archive cannot be opened: " + archivePath.string());
    ArchiveListing listing;
    listing.extractedRoot = destination;
    struct archive_entry* entry = nullptr;
    int result = ARCHIVE_OK;
    while((result = archive_read_next_header(reader.get(), &entry)) == ARCHIVE_OK) {
        const auto name = memberPath(archive_entry_pathname(entry));
        const auto type = archive_entry_filetype(entry);
        const auto* link = archive_entry_symlink(entry);
        if(link == nullptr) link = archive_entry_hardlink(entry);
        if(link != nullptr) {
            memberPath(link);
            archive_read_data_skip(reader.get());
            continue;
        }
        checkOutputPath(destination, name);
        const auto output = destination / name;
        if(type == AE_IFDIR) std::filesystem::create_directories(output);
        else if(type == AE_IFREG) listing.entryBytes += extractFile(reader.get(), output);
        archive_read_data_skip(reader.get());
    }
    if(result != ARCHIVE_EOF) throw std::runtime_error("ReplayVideo archive cannot be read: " + archivePath.string());
    return listing;
}

std::filesystem::path selectReplayArchiveSource(const std::filesystem::path& root) {
    auto directory = root;
    while(std::filesystem::is_directory(directory)) {
        std::vector<std::filesystem::path> directories;
        std::vector<std::filesystem::path> videos;
        std::vector<std::filesystem::path> metadata;
        bool frames = false;
        for(const auto& entry : std::filesystem::directory_iterator(directory)) {
            const auto name = entry.path().filename().string();
            if(name.empty() || name.front() == '.' || entry.is_symlink()) continue;
            if(entry.is_directory()) directories.push_back(entry.path());
            if(!entry.is_regular_file()) continue;
            auto ext = entry.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
            if(ext == ".dai" || ext == ".jpg" || ext == ".jpeg" || ext == ".png") frames = true;
            if(ext == ".mp4" || ext == ".avi") videos.push_back(entry.path());
            if(ext == ".mcap") metadata.push_back(entry.path());
        }
        if(frames && videos.empty()) return directory;
        if(!frames && videos.size() == 1) return videos.front();
        if(!frames && videos.empty() && directories.empty() && metadata.size() == 1) return metadata.front();
        if(!frames && videos.empty() && directories.size() == 1) {
            directory = directories.front();
            continue;
        }
        throw std::runtime_error("ReplayVideo archive needs one source or setArchiveEntry: " + directory.string());
    }
    return directory;
}

}  // namespace replay
}  // namespace dai
