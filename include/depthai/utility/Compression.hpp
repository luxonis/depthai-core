#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace dai {
namespace utility {

/**
 * Gets a list of filenames contained within an archive.
 * @param archivePath Path to the archive file to read
 * @return Vector of paths for the files within the archive
 */
std::vector<std::string> filenamesInArchive(const std::filesystem::path& archivePath);

/**
 * Reads a single file from an archive into memory.
 * @param archivePath Path to the archive file to read
 * @param fileInArchive Path of the file inside the archive
 * @return File contents as a byte vector
 */
std::vector<uint8_t> readFileInArchive(const std::filesystem::path& archivePath, const std::string& fileInArchive);

/**
 * Creates a tar archive containing the specified files.
 * @param archivePath Path where the archive file will be created
 * @param filesOnDisk Vector of paths to file on the host filesystem to include in the archive
 * @param filesInArchive Vector of paths for the files within the archive
 */
void archiveFiles(const std::filesystem::path& archivePath,
                  const std::vector<std::filesystem::path>& filesOnDisk,
                  const std::vector<std::string>& filesInArchive);

/**
 * Creates a gzip-compressed tar archive containing the specified files.
 * @param archivePath Path where the archive file will be created
 * @param filesOnDisk Vector of paths to file on the host filesystem to include in the archive
 * @param filesInArchive Vector of paths for the files within the archive
 */
void archiveFilesCompressed(const std::filesystem::path& archivePath,
                            const std::vector<std::filesystem::path>& filesOnDisk,
                            const std::vector<std::string>& filesInArchive);

/**
 * Extracts files from an archive, supports both compressed (e.g. tar.gz) and uncompressed (e.g. tar) archives.
 * @param archivePath Path to the archive file to extract from
 * @param filesInArchive Vector of paths for the files within the archive to extract
 * @param filesOnDisk Vector of paths where the extracted files should be written
 */
void extractFiles(const std::filesystem::path& archivePath,
                  const std::vector<std::string>& filesInArchive,
                  const std::vector<std::filesystem::path>& filesOnDisk);

}  // namespace utility
}  // namespace dai
