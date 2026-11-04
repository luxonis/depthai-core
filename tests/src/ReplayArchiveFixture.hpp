#pragma once

#include <archive.h>
#include <archive_entry.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace replaytest {

struct Member {
    std::string name;
    std::vector<std::uint8_t> bytes;
    bool directory = false;
    std::string symlink;
};

class Directory {
   public:
    Directory() {
        static std::atomic<unsigned int> next{0};
        path = std::filesystem::temp_directory_path()
               / ("dai-archive-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" + std::to_string(next++));
        std::filesystem::create_directories(path);
    }
    ~Directory() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    Directory(const Directory&) = delete;
    Directory& operator=(const Directory&) = delete;
    std::filesystem::path path;
};

inline void writeArchive(const std::filesystem::path& path, const std::vector<Member>& members) {
    const std::unique_ptr<struct archive, decltype(&archive_write_free)> writer(archive_write_new(), archive_write_free);
    if(!writer) throw std::runtime_error("Could not create test archive writer");
    auto check = [&](int result) {
        if(result != ARCHIVE_OK) {
            const auto* error = archive_error_string(writer.get());
            throw std::runtime_error(error == nullptr ? "Could not write test archive" : error);
        }
    };
    if(path.extension() == ".xz") check(archive_write_add_filter_xz(writer.get()));
    else if(path.extension() == ".gz" || path.extension() == ".tgz") check(archive_write_add_filter_gzip(writer.get()));
    check(archive_write_set_format_pax_restricted(writer.get()));
#ifdef _WIN32
    check(archive_write_open_filename_w(writer.get(), path.c_str()));
#else
    check(archive_write_open_filename(writer.get(), path.c_str()));
#endif
    for(const auto& member : members) {
        const std::unique_ptr<struct archive_entry, decltype(&archive_entry_free)> entry(archive_entry_new(), archive_entry_free);
        if(!entry) throw std::runtime_error("Could not create test archive entry");
        archive_entry_set_pathname(entry.get(), member.name.c_str());
        archive_entry_set_filetype(entry.get(), member.directory ? AE_IFDIR : member.symlink.empty() ? AE_IFREG : AE_IFLNK);
        archive_entry_set_perm(entry.get(), member.directory ? 0755 : 0644);
        archive_entry_set_size(entry.get(), static_cast<la_int64_t>(member.bytes.size()));
        if(!member.symlink.empty()) archive_entry_set_symlink(entry.get(), member.symlink.c_str());
        check(archive_write_header(writer.get(), entry.get()));
        if(!member.bytes.empty() && archive_write_data(writer.get(), member.bytes.data(), member.bytes.size()) != static_cast<la_ssize_t>(member.bytes.size())) {
            throw std::runtime_error("Could not write test archive payload");
        }
    }
    check(archive_write_close(writer.get()));
}

}  // namespace replaytest
