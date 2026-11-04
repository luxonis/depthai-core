#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace dai {

/**
 * Where a replay source is stored.
 * HOST paths are relative to the process working directory. DEVICE paths must be absolute.
 */
enum class ReplaySourceLocation : std::int32_t { HOST = 0, DEVICE = 1 };

/**
 * STREAM decodes during the run. PRE_DECODE decodes every frame before the pipeline starts.
 */
enum class ReplayDecodeMode : std::int32_t { STREAM = 0, PRE_DECODE = 1 };

/// A cache entry is either the source or a pre-decoded frame set.
enum class ReplayCacheKind : std::int32_t { SOURCE = 0, DECODED = 1 };

/**
 * One replay-cache entry.
 * `key` is internal. `sourceHash` is lowercase hexadecimal SHA-256.
 */
struct ReplayCacheEntry {
    std::string key;
    ReplayCacheKind kind = ReplayCacheKind::SOURCE;
    std::string sourceHash;
    std::uint64_t sizeBytes = 0;
    std::int64_t lastUseMs = 0;
    bool inUse = false;
};

/// Limit, bytes in use, and the entries of one replay cache.
struct ReplayCacheInfo {
    std::uint64_t limitBytes = 0;
    std::uint64_t usedBytes = 0;
    std::vector<ReplayCacheEntry> entries;
};

}  // namespace dai

/*
 * Device cache RPC contract (implemented by RVC4 firmware, called by the host):
 *   replayCacheInfo -> (limitBytes, usedBytes, rows of (key, kind, sourceHash, sizeBytes, lastUseMs, inUse))
 *   replayCacheSetLimit(uint64_t)
 *   replayCacheClear()
 *   replayCacheQuery(hash) -> (present, devicePath, sizeBytes)
 *   replayCacheBegin(hash, totalBytes)
 *   replayCacheWrite(hash, offset, bytes)
 *   replayCacheCommit(hash)
 *   replayCacheAbort(hash)
 *   replayCacheRelease(keys)
 *   replayCacheSourceSize(absolutePath) -> (width, height, fps)
 * A hash mismatch or an interrupted upload leaves no entry. Chunk size is 1 MiB.
 * Progress line: "Replay upload <hash> <done>/<total> bytes".
 */
