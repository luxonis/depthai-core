#include <cstdint>
#include <string>
#include <vector>

#include <filesystem>

namespace dai {
namespace utility {

std::vector<uint8_t> deflate(uint8_t* data, size_t size, int compressionLevel = 6);
std::vector<uint8_t> inflate(uint8_t* data, size_t size);

void tarFiles(const std::filesystem::path& path, const std::vector<std::string>& files, const std::vector<std::string>& outFiles);

std::vector<std::string> filenamesInTar(const std::filesystem::path& path);

void untarFiles(const std::filesystem::path& path, const std::vector<std::string>& files, const std::vector<std::string>& outFiles);

}  // namespace utility
}  // namespace dai
