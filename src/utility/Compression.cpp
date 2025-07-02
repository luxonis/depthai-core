#include "depthai/utility/Compression.hpp"

#include <fmt/format.h>
#include <fmt/std.h>

#include <cassert>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "archive.h"
#include "archive_entry.h"
#include "utility/span.hpp"
#include "zlib.h"

namespace dai {
namespace utility {

std::vector<uint8_t> deflate(span<uint8_t>& data, int compressionLevel) {
    z_stream stream;
    stream.zalloc = Z_NULL;
    stream.zfree = Z_NULL;
    stream.opaque = Z_NULL;
    int ret = deflateInit(&stream, compressionLevel);
    if(ret != Z_OK) {
        throw std::runtime_error("deflateInit failed with error code " + std::to_string(ret) + ".");
    }
    std::vector<uint8_t> result(deflateBound(&stream, data.size()));
    stream.next_out = result.data();
    stream.avail_out = result.size();
    stream.next_in = data.data();
    stream.avail_in = data.size();
    ret = deflate(&stream, Z_NO_FLUSH);
    if(ret != Z_OK) {
        throw std::runtime_error("deflate failed with error code " + std::to_string(ret) + ".");
    }
    ret = deflate(&stream, Z_FINISH);
    if(ret != Z_STREAM_END) {
        throw std::runtime_error("Could not finish deflation.");
    }
    deflateEnd(&stream);
    result.resize(stream.total_out);
    return result;
}

std::vector<uint8_t> inflate(span<uint8_t>& data) {
    z_stream stream;
    stream.zalloc = Z_NULL;
    stream.zfree = Z_NULL;
    stream.opaque = Z_NULL;
    int ret = inflateInit(&stream);
    if(ret != Z_OK) {
        throw std::runtime_error("inflateInit failed with error code " + std::to_string(ret) + ".");
    }
    std::vector<uint8_t> result;
    stream.next_out = result.data();
    stream.avail_out = result.size();
    stream.next_in = data.data();
    stream.avail_in = data.size();
    ret = inflate(&stream, Z_NO_FLUSH);
    if(ret != Z_OK) {
        throw std::runtime_error("inflate failed with error code " + std::to_string(ret) + ".");
    }
    ret = inflate(&stream, Z_FINISH);
    if(ret != Z_STREAM_END) {
        throw std::runtime_error("Could not finish inflation.");
    }
    inflateEnd(&stream);
    result.resize(stream.total_out);
    return result;
}

void tarFiles(const std::filesystem::path& tarPath, const std::vector<std::filesystem::path>& filesOnDisk, const std::vector<std::string>& filesInTar) {
    assert(filesOnDisk.size() == filesInTar.size());

    struct archive* a;
    struct archive_entry* entry;
    char buff[8192];
    std::ifstream fileStream;

    a = archive_write_new();
    archive_write_set_format_pax_restricted(a);
#ifdef _WIN32
    archive_write_open_filename_w(a, tarPath.c_str());
#else
    archive_write_open_filename(a, tarPath.c_str());
#endif
    for(size_t i = 0; i < filesOnDisk.size(); i++) {
        const auto& file = filesOnDisk[i];
        const std::string& outFile = filesInTar[i];
        entry = archive_entry_new();
        archive_entry_set_pathname(entry, outFile.c_str());
        archive_entry_set_filetype(entry, AE_IFREG);
        archive_entry_set_perm(entry, 0644);

        std::filesystem::path filePath(file);
        archive_entry_set_size(entry, std::filesystem::file_size(filePath));

        archive_write_header(a, entry);
        fileStream.open(filePath, std::ios::binary);
        while(fileStream.read(buff, sizeof(buff))) {
            archive_write_data(a, buff, fileStream.gcount());
        }
        if(fileStream.gcount() > 0) {
            archive_write_data(a, buff, fileStream.gcount());
        }
        fileStream.close();
        archive_entry_free(entry);
    }
    archive_write_close(a);
    archive_write_free(a);
}

std::vector<std::string> filenamesInTar(const std::filesystem::path& tarPath) {
    std::vector<std::string> result;

    struct archive* a;
    struct archive_entry* entry;

    a = archive_read_new();
    archive_read_support_filter_all(a);
    archive_read_support_format_all(a);
#if defined(_WIN32)
    int r = archive_read_open_filename_w(a, tarPath.c_str(), 10240);
#else
    int r = archive_read_open_filename(a, tarPath.c_str(), 10240);
#endif
    if(r != ARCHIVE_OK) {
        throw std::runtime_error("Could not open archive.");
    }
    while(archive_read_next_header(a, &entry) == ARCHIVE_OK) {
        result.emplace_back(archive_entry_pathname(entry));
        archive_read_data_skip(a);
    }
    r = archive_read_free(a);
    if(r != ARCHIVE_OK) {
        throw std::runtime_error("Could not free archive.");
    }

    return result;
}

void untarFiles(const std::filesystem::path& tarPath, const std::vector<std::string>& filesInTar, const std::vector<std::filesystem::path>& filesOnDisk) {
    struct archive* a;
    struct archive_entry* entry;
    std::ofstream outFileStream;

    a = archive_read_new();
    archive_read_support_filter_all(a);
    archive_read_support_format_all(a);
#if defined(_WIN32) && defined(_MSC_VER)
    int r = archive_read_open_filename_w(a, tarPath.c_str(), 10240);
#else
    int r = archive_read_open_filename(a, tarPath.c_str(), 10240);
#endif
    if(r != ARCHIVE_OK) {
        throw std::runtime_error("Could not open archive.");
    }
    assert(filesInTar.size() == filesOnDisk.size());
    while(archive_read_next_header(a, &entry) == ARCHIVE_OK) {
        for(size_t i = 0; i < filesInTar.size(); i++) {
            const auto& file = filesInTar[i];
            if(file == archive_entry_pathname(entry)) {
                const auto& outFile = filesOnDisk[i];
                std::filesystem::path outFilePath(outFile);
                outFileStream.open(outFilePath, std::ios::binary);
                if(!outFileStream) {
                    throw std::runtime_error(fmt::format("Could not open file {} for writing.", outFile));
                }
                size_t size = archive_entry_size(entry);
                std::vector<uint8_t> buff(size);
                archive_read_data(a, buff.data(), size);
                outFileStream.write(reinterpret_cast<char*>(buff.data()), size);
                outFileStream.close();
                break;
            }
        }
        archive_read_data_skip(a);
    }

    r = archive_read_free(a);
    if(r != ARCHIVE_OK) {
        throw std::runtime_error("Could not free archive.");
    }
}

}  // namespace utility
}  // namespace dai