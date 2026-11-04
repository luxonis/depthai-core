#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ReplayCache.hpp"

namespace dai {
namespace replay {

constexpr std::uint64_t kDefaultCacheLimit = 32ull << 30;
constexpr std::uint64_t kUploadChunk = 1ull << 20;

struct CacheSlotRequest {
    std::string key;
    ReplayCacheKind kind = ReplayCacheKind::SOURCE;
    std::string sourceHash;
    std::uint64_t sizeBytes = 0;
    std::uint64_t diskBytes = 0;
    bool ownsFiles = false;
    std::filesystem::path externalPath;
};

struct CacheSlot {
    bool hit = false;
    std::filesystem::path path;
    std::vector<std::string> logs;
};

std::filesystem::path hostCacheDirectory(bool standalone);
void setReplayCacheStandalone(bool standalone);

std::string sha256OfBytes(const std::uint8_t* data, std::size_t size);
std::string sha256OfFile(const std::filesystem::path& path);
std::string sha256OfDirectory(const std::filesystem::path& directory);
std::string sha256OfFileAndMetadata(const std::filesystem::path& file, const std::filesystem::path& metadata);

/// Evicts if needed and either returns the existing entry or a path the caller must fill.
CacheSlot beginCacheEntry(std::uint64_t nodeId, std::uint64_t pipelineToken, bool persistent, const CacheSlotRequest& request);
void commitCacheEntry(std::uint64_t nodeId, std::uint64_t pipelineToken, bool persistent, const CacheSlotRequest& request);
void abortCacheEntry(const std::string& key);
void releaseReplayNode(std::uint64_t nodeId, std::uint64_t pipelineToken);
void releaseReplayToken(std::uint64_t pipelineToken);

ReplayCacheInfo readHostCacheInfo();
void writeHostCacheLimit(std::uint64_t bytes);
void clearHostCache();

struct ListedFrame {
    std::filesystem::path path;
    std::int64_t sequence = 0;
};

struct ListedSource {
    enum class Kind { Video, Frames, Mcap };
    Kind kind = Kind::Video;
    std::filesystem::path path;
    std::string stream;
    std::vector<ListedFrame> frames;
    std::filesystem::path autoMetadata;
    bool dai = false;
    bool mjpegDai = false;
    bool jpegOrPng = false;
    int width = 0;
    int height = 0;
    ImgFrame::Type sourceType = ImgFrame::Type::YUV420p;
    /// .dai type outside the supported set, sent unchanged when type and size are unset.
    bool passthrough = false;
    double fps = 0;
    int frameCount = 0;
    std::string codec;
    std::string profile;
    std::uint64_t bytes = 0;
};

struct ArchiveListing {
    std::uint64_t entryBytes = 0;
    std::filesystem::path extractedRoot;
};

/// Rejects absolute paths, ".." components, and symlink targets that leave the archive.
ArchiveListing extractReplayArchive(const std::filesystem::path& archive, const std::filesystem::path& destination);

ListedSource inspectReplaySource(const std::filesystem::path& path);
/// Selects `<entry>` inside a holistic directory: a frame directory, a video, or an mcap.
std::filesystem::path resolveArchiveEntry(const std::filesystem::path& root, const std::string& entry, bool metadataOnly);

struct DecodedPixels {
    int width = 0;
    int height = 0;
    ImgFrame::Type type = ImgFrame::Type::YUV420p;
    /// Tightly packed payload for `type`, or the passthrough frame's data.
    std::vector<std::uint8_t> data;
    bool passthrough = false;
    bool fullRange = false;
    ImgFrame::Specs fb{};
    ImgFrame::Specs sourceFb{};
    ImgFrame::CameraSettings cam{};
    ImgTransformation transformation{};
    std::uint32_t instanceNum = 0;
    std::uint32_t category = 0;
    /// Recorded `.dai` timing. Video frames leave this unset and use metadata or the schedule.
    bool hasTiming = false;
    std::int64_t sequenceNum = 0;
    std::chrono::steady_clock::time_point timestamp{};
    std::chrono::steady_clock::time_point timestampDevice{};
    std::optional<std::chrono::system_clock::time_point> timestampSystem;
};

class FrameReader {
   public:
    FrameReader();
    ~FrameReader();
    FrameReader(FrameReader&&) noexcept;
    FrameReader& operator=(FrameReader&&) noexcept;
    void open(const ListedSource& source);
    int count() const;
    DecodedPixels read(int index);

   private:
    struct State;
    std::unique_ptr<State> state;
};

/**
 * Converts `pixels` to `outType` and `outW` x `outH`.
 * YUV and RGB use BT.601 limited range, except a full-range JPEG source.
 * A size equal to the source is not resized.
 */
DecodedPixels convertPixels(const DecodedPixels& pixels, ImgFrame::Type outType, int outW, int outH);

std::shared_ptr<ImgFrame> pixelsToFrame(const DecodedPixels& pixels);

bool isReplayOutType(ImgFrame::Type type);
std::string frameTypeName(ImgFrame::Type type);

std::tuple<int, int, double> probeMediaSize(const std::filesystem::path& path);

}  // namespace replay
}  // namespace dai
