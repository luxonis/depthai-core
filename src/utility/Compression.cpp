#include "Compression.hpp"

#include <stdexcept>
#include <fcntl.h>

#include "archive.h"
#include "archive_entry.h"
#include "zlib.h"

namespace dai {
namespace utility {

std::vector<uint8_t> deflate(std::vector<uint8_t>& data, int compressionLevel) {
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
std::vector<uint8_t> inflate(std::vector<uint8_t>& data) {
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

void tarFiles(const std::string& path, const std::vector<std::string>& files) {
    struct archive* a;
    struct archive_entry* entry;
    struct stat st;
    char buff[8192];
    int len;
    int fd;

    a = archive_write_new();
    archive_write_add_filter_gzip(a);
    archive_write_set_format_pax_restricted(a);
    archive_write_open_filename(a, path.c_str());
    for(const auto& file : files) {
        stat(file.c_str(), &st);
        entry = archive_entry_new();
        archive_entry_set_pathname(entry, file.c_str());
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

}  // namespace utility
}  // namespace dai
