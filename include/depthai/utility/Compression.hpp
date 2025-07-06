#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace dai {
namespace utility {

std::vector<uint8_t> deflate(uint8_t* data, size_t size, int compressionLevel = 6);
std::vector<uint8_t> inflate(uint8_t* data, size_t size);

/**
 * Gets a list of filenames contained within a tar archive.
 * @param tarPath Path to the tar file to read
 * @return Vector of paths for the files within the tar archive
 */
std::vector<std::string> filenamesInTar(const std::filesystem::path& tarPath);

/**
 * Creates a tar archive containing the specified files.
 * @param tarPath Path where the tar file will be created
 * @param filesOnDisk Vector of paths to file on the host filesystem to include in the archive
 * @param filesInTar Vector of paths for the files within the tar archive
 */
void tarFiles(const std::filesystem::path& tarPath, const std::vector<std::filesystem::path>& filesOnDisk, const std::vector<std::string>& filesInTar);

/**
 * Extracts files from a tar archive.
 * @param tarPath Path to the tar file to extract from
 * @param filesInTar Vector of paths for the files within the tar to extract
 * @param filesOnDisk Vector of paths where the extracted files should be written
 */
void untarFiles(const std::filesystem::path& tarPath, const std::vector<std::string>& filesInTar, const std::vector<std::filesystem::path>& filesOnDisk);

}  // namespace utility
}  // namespace dai
