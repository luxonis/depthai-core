#include "Compression.hpp"

#include <fcntl.h>

#include <cassert>
#include <stdexcept>

#include "archive.h"
#include "archive_entry.h"
#include "zlib.h"

namespace dai {
namespace utility {

std::vector<uint8_t> deflate(uint8_t* data, size_t size, int compressionLevel) {
    z_stream stream;
    stream.zalloc = Z_NULL;
    stream.zfree = Z_NULL;
    stream.opaque = Z_NULL;
    int ret = deflateInit(&stream, compressionLevel);
    if(ret != Z_OK) {
        throw std::runtime_error("deflateInit failed with error code " + std::to_string(ret) + ".");
    }
    std::vector<uint8_t> result(deflateBound(&stream, size));
    stream.next_out = result.data();
    stream.avail_out = result.size();
    stream.next_in = data;
    stream.avail_in = size;
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
std::vector<uint8_t> inflate(uint8_t* data, size_t size) {
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
    stream.next_in = data;
    stream.avail_in = size;
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

void tarFiles(const std::string& path, const std::vector<std::string>& files, const std::vector<std::string>& outFiles) {
    assert(files.size() == outFiles.size());

    struct archive* a;
    struct archive_entry* entry;
    struct stat st;
    char buff[8192];
    int len;
    int fd;

    a = archive_write_new();
    // archive_write_add_filter_gzip(a);
    archive_write_set_format_pax_restricted(a);
    archive_write_open_filename(a, path.c_str());
    for(size_t i = 0; i < files.size(); i++) {
        const auto& file = files[i];
        const auto& outFile = outFiles[i];
        stat(file.c_str(), &st);
        entry = archive_entry_new();
        archive_entry_set_pathname(entry, outFile.c_str());
        archive_entry_set_size(entry, st.st_size);
        archive_entry_set_filetype(entry, AE_IFREG);
        archive_entry_set_perm(entry, 0644);
        archive_write_header(a, entry);
        fd = open(file.c_str(), O_RDONLY);
        len = read(fd, buff, sizeof(buff));
        while(len > 0) {
            archive_write_data(a, buff, len);
            len = read(fd, buff, sizeof(buff));
        }
        close(fd);
        archive_entry_free(entry);
    }
    archive_write_close(a);
    archive_write_free(a);
}

std::vector<std::string> filenamesInTar(const std::string& path) {
    std::vector<std::string> result;

    struct archive* a;
    struct archive_entry* entry;
    int r;

    a = archive_read_new();
    archive_read_support_filter_all(a);
    archive_read_support_format_all(a);
    r = archive_read_open_filename(a, path.c_str(), 10240);
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

void untarFiles(const std::string& path, const std::vector<std::string>& files, const std::vector<std::string>& outFiles) {
    struct archive* a;
    struct archive_entry* entry;
    int r;

    a = archive_read_new();
    archive_read_support_filter_all(a);
    archive_read_support_format_all(a);
    r = archive_read_open_filename(a, path.c_str(), 10240);
    if(r != ARCHIVE_OK) {
        throw std::runtime_error("Could not open archive.");
    }
    for(size_t i = 0; i < files.size(); i++) {
        const auto& file = files[i];
        const auto& outFile = outFiles[i];
        while(archive_read_next_header(a, &entry) == ARCHIVE_OK) {
            if(file == archive_entry_pathname(entry)) {
                int fd = open(outFile.c_str(), O_WRONLY | O_CREAT, archive_entry_perm(entry));
                if(fd < 0) {
                    throw std::runtime_error("Could not open file.");
                }
                size_t size = archive_entry_size(entry);
                std::vector<uint8_t> buff(size);
                archive_read_data(a, buff.data(), size);
                write(fd, buff.data(), size);
                close(fd);
                break;
            }
            archive_read_data_skip(a);
        }
    }

    r = archive_read_free(a);
    if(r != ARCHIVE_OK) {
        throw std::runtime_error("Could not free archive.");
    }
}

}  // namespace utility
}  // namespace dai
