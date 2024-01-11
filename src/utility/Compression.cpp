#include "Compression.hpp"

#include <stdexcept>

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

std::string tarFiles(const std::string& path, const std::vector<std::string>& files) {
    return "";
}

}  // namespace utility
}  // namespace dai
