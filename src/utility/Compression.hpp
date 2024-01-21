#include <cstdint>
#include <string>
#include <vector>

namespace dai {
namespace utility {

std::vector<uint8_t> deflate(uint8_t* data, size_t size, int compressionLevel = 6);
std::vector<uint8_t> inflate(uint8_t* data, size_t size);

void tarFiles(const std::string& path, const std::vector<std::string>& files, const std::vector<std::string>& outFiles);

}  // namespace utility
}  // namespace dai
