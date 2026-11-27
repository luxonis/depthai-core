#pragma once

#include <cstdint>
#include <filesystem>

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

}  // namespace replay
}  // namespace dai
