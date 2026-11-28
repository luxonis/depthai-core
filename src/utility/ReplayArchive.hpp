#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace dai {
namespace replay {

struct ArchiveListing {
    std::uint64_t entryBytes = 0;
    std::filesystem::path extractedRoot;
};

// Shared by host replay and firmware; has no image-decoder or protobuf dependency.
// Extracts regular files and directories, rejecting paths and link targets that escape.
ArchiveListing extractReplayArchive(const std::filesystem::path& archive, const std::filesystem::path& destination);

// Select a frame directory or a single media file, unwrapping enclosing directories.
std::filesystem::path selectReplayArchiveSource(const std::filesystem::path& root);

struct NamedArchiveEntry {
    enum class Kind { Missing, Video, Frames, Mcap };
    Kind kind = Kind::Missing;
    std::string archiveEntry;
    std::string path;
    std::string extension;
    bool hasMetadata = false;
    std::string metadataPath;
    std::vector<std::string> frames;
};

NamedArchiveEntry selectNamedArchiveEntry(const std::vector<std::string>& entries, const std::string& name, bool metadataOnly);

std::filesystem::path uniqueReplayTempDirectory();

}  // namespace replay
}  // namespace dai
