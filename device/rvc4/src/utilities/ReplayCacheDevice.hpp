#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace dai {
namespace gate {

/// Rows are (key, kind, sourceHash, sizeBytes, lastUseMs, inUse).
std::tuple<std::uint64_t, std::uint64_t, std::vector<std::tuple<std::string, std::int32_t, std::string, std::uint64_t, std::int64_t, bool>>> replayCacheInfo();

void replayCacheSetLimit(std::uint64_t bytes);
void replayCacheClear();
std::tuple<bool, std::string, std::uint64_t> replayCacheQuery(const std::string& hash);
void replayCacheBegin(const std::string& hash, std::uint64_t totalBytes);
std::tuple<bool, std::string, std::uint64_t> replayCachePrepareFile(const std::string& hash, std::uint64_t totalBytes);
std::tuple<bool, std::string, std::uint64_t> replayCacheBeginDirectory(const std::string& hash,
                                                                   const std::map<std::string, std::uint64_t>& files);
void replayCacheWrite(const std::string& hash, std::uint64_t offset, const std::vector<std::uint8_t>& bytes);
void replayCacheWriteFile(const std::string& hash, const std::string& filename, std::uint64_t offset, const std::vector<std::uint8_t>& bytes);
void replayCacheCommit(const std::string& hash);
void replayCacheAbort(const std::string& hash);
void replayCacheRelease(const std::vector<std::string>& keys);

/// (width, height, fps) of an absolute path already on the device. fps is 0 when the container does not say.
std::tuple<int, int, double> replayCacheSourceSize(const std::string& absolutePath);

/// Remember that a committed upload should survive pipeline stop. Does not take a use count.
void replayCacheMarkPersistent(const std::string& hash, const std::string& metadataPath = {});

/**
 * Called when a replay node actually starts sending.
 * replayCacheRelease deletes a non-persistent entry only after it was armed,
 * so a check that fails before the run keeps the completed upload.
 */
void replayCacheArm(const std::string& hash, const std::string& metadataPath = {});

}  // namespace gate
}  // namespace dai
