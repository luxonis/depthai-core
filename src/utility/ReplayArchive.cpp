#include "ReplayArchive.hpp"

#include <archive.h>
#include <archive_entry.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef _WIN32
    #include <unistd.h>
#else
    #include <stdio.h>
#endif

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
        if(std::filesystem::is_symlink(std::filesystem::symlink_status(path)))
            throw std::runtime_error("ReplayVideo archive entry is a symlink: " + relative.string());
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
        if(type == AE_IFDIR)
            std::filesystem::create_directories(output);
        else if(type == AE_IFREG)
            listing.entryBytes += extractFile(reader.get(), output);
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

namespace {

std::string normalizeEntryName(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    while(path.rfind("./", 0) == 0) path.erase(0, 2);
    return path;
}

std::string lowerExtension(const std::string& name) {
    const auto slash = name.find_last_of('/');
    const auto base = slash == std::string::npos ? name : name.substr(slash + 1);
    const auto dot = base.find_last_of('.');
    if(dot == std::string::npos) return {};
    auto ext = base.substr(dot);
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return ext;
}

bool hasFile(const std::vector<std::string>& names, const std::string& path) {
    return std::find(names.begin(), names.end(), path) != names.end();
}

bool directoryPresent(const std::vector<std::string>& names, const std::string& entry) {
    const auto prefix = entry + "/";
    for(const auto& name : names) {
        if(name.rfind(prefix, 0) == 0) return true;
    }
    return false;
}

struct FramePick {
    bool valid = false;
    bool blocked = false;
    std::string extension;
    std::vector<std::string> frames;
};

FramePick pickFrames(const std::vector<std::string>& names, const std::string& entry) {
    FramePick pick;
    const auto prefix = entry + "/";
    std::set<std::string> prefixes;
    std::set<std::string> extensions;
    std::set<std::int64_t> seen;
    std::vector<std::pair<std::int64_t, std::string>> frames;
    for(const auto& name : names) {
        if(name.rfind(prefix, 0) != 0) continue;
        const auto file = name.substr(prefix.size());
        if(file.empty() || file.find('/') != std::string::npos || file.front() == '.') continue;
        auto ext = lowerExtension(file);
        if(ext == ".jpeg") ext = ".jpg";
        if(ext != ".dai" && ext != ".jpg" && ext != ".png") continue;
        const auto stem = std::filesystem::path(file).stem().string();
        const auto split = stem.rfind('_');
        if(split == std::string::npos || split == 0 || split + 1 >= stem.size()) continue;
        const auto seqText = stem.substr(split + 1);
        if(!std::all_of(seqText.begin(), seqText.end(), [](unsigned char character) { return std::isdigit(static_cast<unsigned char>(character)) != 0; }))
            continue;
        std::int64_t sequence = 0;
        try {
            sequence = std::stoll(seqText);
        } catch(const std::exception&) {
            pick.blocked = true;
            return pick;
        }
        if(!seen.insert(sequence).second) {
            pick.blocked = true;
            return pick;
        }
        prefixes.insert(stem.substr(0, split));
        extensions.insert(ext);
        frames.emplace_back(sequence, name);
    }
    if(frames.empty()) return pick;
    if(prefixes.size() != 1 || extensions.size() != 1) {
        pick.blocked = true;
        return pick;
    }
    std::sort(frames.begin(), frames.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    pick.valid = true;
    pick.extension = *extensions.begin();
    pick.frames.reserve(frames.size());
    for(const auto& frame : frames) pick.frames.push_back(frame.second);
    return pick;
}

enum class AttemptKind { Miss, Blocked, Hit };

struct Attempt {
    AttemptKind kind = AttemptKind::Miss;
    NamedArchiveEntry entry;
};

Attempt selectAt(const std::vector<std::string>& names, const std::string& entry, bool metadataOnly) {
    Attempt result;
    if(metadataOnly) {
        const auto mcap = entry + ".mcap";
        if(!hasFile(names, mcap)) return result;
        result.kind = AttemptKind::Hit;
        result.entry.kind = NamedArchiveEntry::Kind::Mcap;
        result.entry.archiveEntry = entry;
        result.entry.path = mcap;
        result.entry.extension = ".mcap";
        return result;
    }
    if(directoryPresent(names, entry)) {
        const auto frames = pickFrames(names, entry);
        if(!frames.valid) {
            result.kind = AttemptKind::Blocked;
            return result;
        }
        result.kind = AttemptKind::Hit;
        result.entry.kind = NamedArchiveEntry::Kind::Frames;
        result.entry.archiveEntry = entry;
        result.entry.path = entry;
        result.entry.extension = frames.extension;
        result.entry.frames = frames.frames;
        const auto mcap = entry + ".mcap";
        if(hasFile(names, mcap)) {
            result.entry.hasMetadata = true;
            result.entry.metadataPath = mcap;
        }
        return result;
    }
    for(const char* ext : {".mp4", ".avi", ".mcap"}) {
        const auto path = entry + ext;
        if(!hasFile(names, path)) continue;
        result.kind = AttemptKind::Hit;
        result.entry.archiveEntry = entry;
        result.entry.path = path;
        result.entry.extension = ext;
        if(std::string(ext) == ".mcap") {
            result.entry.kind = NamedArchiveEntry::Kind::Mcap;
        } else {
            result.entry.kind = NamedArchiveEntry::Kind::Video;
            const auto mcap = entry + ".mcap";
            if(hasFile(names, mcap)) {
                result.entry.hasMetadata = true;
                result.entry.metadataPath = mcap;
            }
        }
        return result;
    }
    return result;
}

std::optional<std::string> commonOuterUnder(const std::vector<std::string>& names, const std::string& prefix) {
    const auto lead = prefix.empty() ? std::string() : prefix + "/";
    std::string top;
    bool any = false;
    for(const auto& name : names) {
        if(!lead.empty() && name.rfind(lead, 0) != 0) continue;
        const auto rest = lead.empty() ? name : name.substr(lead.size());
        if(rest.empty()) continue;
        const auto slash = rest.find('/');
        if(slash == std::string::npos) return std::nullopt;
        const auto piece = rest.substr(0, slash);
        if(!any) {
            top = piece;
            any = true;
        } else if(top != piece) {
            return std::nullopt;
        }
    }
    if(!any) return std::nullopt;
    return top;
}

}  // namespace

NamedArchiveEntry selectNamedArchiveEntry(const std::vector<std::string>& entries, const std::string& name, bool metadataOnly) {
    std::vector<std::string> names;
    names.reserve(entries.size());
    for(const auto& entry : entries) {
        auto normalized = normalizeEntryName(entry);
        if(normalized.empty() || normalized.back() == '/') continue;
        names.push_back(std::move(normalized));
    }
    std::string prefix;
    auto attempt = selectAt(names, name, metadataOnly);
    while(attempt.kind == AttemptKind::Miss) {
        const auto outer = commonOuterUnder(names, prefix);
        if(!outer.has_value()) break;
        prefix = prefix.empty() ? *outer : prefix + "/" + *outer;
        attempt = selectAt(names, prefix + "/" + name, metadataOnly);
    }
    if(attempt.kind != AttemptKind::Hit) return NamedArchiveEntry{};
    return attempt.entry;
}

std::filesystem::path uniqueReplayTempDirectory() {
#if defined(_WIN32)
    auto basePath = std::filesystem::temp_directory_path() / L"depthai_XXXXXX";
    auto baseStr = basePath.wstring();
    if(_wmktemp_s(baseStr.data(), baseStr.size() + 1) != 0 || !std::filesystem::create_directories(baseStr)) {
        throw std::runtime_error("Failed to create a unique temporary directory");
    }
    return std::filesystem::path(baseStr);
#else
    auto pattern = (std::filesystem::temp_directory_path() / "depthai_XXXXXX").string();
    std::vector<char> storage(pattern.begin(), pattern.end());
    storage.push_back('\0');
    if(mkdtemp(storage.data()) == nullptr) throw std::runtime_error("Failed to create a unique temporary directory");
    return std::filesystem::path(storage.data());
#endif
}

}  // namespace replay
}  // namespace dai
