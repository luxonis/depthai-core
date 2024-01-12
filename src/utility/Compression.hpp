#include <cstdint>
#include <string>
#include <vector>

namespace dai {
namespace utility {

std::vector<uint8_t> deflate(std::vector<uint8_t>& data, int compressionLevel = 6);
std::vector<uint8_t> inflate(std::vector<uint8_t>& data);

void tarFiles(const std::string& path, const std::vector<std::string>& files);

}  // namespace utility
}  // namespace dai
