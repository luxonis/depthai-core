#pragma once

#include <cstdint>
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/properties/ReplayMetadataOnlyProperties.hpp>
#include <depthai/properties/ReplayVideoProperties.hpp>
#include <depthai/utility/ReplayCache.hpp>
#include <memory>

namespace dai {
namespace node {

struct ReplayRuntime;

/** Drops replay-cache holds when pipeline build fails before the nodes start. */
void releaseEveryReplayPrepare(Pipeline& pipeline);

/**
 * @brief Replays a video, a frame directory, or an archive as ImgFrame messages.
 *
 * On RVC4 the node runs on the device unless setRunOnHost(true) is set. On RVC2 and on a
 * pipeline with no device it runs on the host. Setters do not validate; the pipeline checks
 * the configuration at start. The existing setters keep their meaning.
 */
class ReplayVideo : public DeviceNodeCRTP<DeviceNode, ReplayVideo, ReplayVideoProperties>, public HostRunnable {
    friend class dai::PipelineImpl;
    friend void releaseEveryReplayPrepare(Pipeline& pipeline);

   public:
    constexpr static const char* NAME = "ReplayVideo";

    /**
     * ImgFrame output.
     * Default queue is blocking with size 8.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    ReplayVideo();
    explicit ReplayVideo(const std::shared_ptr<Device>& device);
    explicit ReplayVideo(std::unique_ptr<Properties> props);
    ~ReplayVideo() override;

    /** Validate the source, fill the device properties, and pre-decode when that mode is set. */
    void buildStage1() override;
    /** Send frames on the host. Device execution uses the same engine when the source is local. */
    void run() override;
    /** Report where this node runs. The value follows the device and setRunOnHost. */
    bool runOnHost() const override;

    std::filesystem::path getReplayMetadataFile() const;
    std::filesystem::path getReplayVideoFile() const;
    ImgFrame::Type getOutFrameType() const;
    std::tuple<int, int> getSize() const;
    float getFps() const;
    bool getLoop() const;
    /** True when setOutFrameType was called. The frames of an unset .dai stream keep their stored type. */
    bool getOutFrameTypeSet() const;
    ReplaySourceLocation getSourceLocation() const;
    ReplayDecodeMode getDecodeMode() const;
    bool getPersistentUpload() const;
    std::string getArchiveEntry() const;
    bool getStopPipelineAtEnd() const;

    /** Host path of a video file, a frame directory, or an archive. */
    ReplayVideo& setReplayVideoFile(const std::filesystem::path& replayVideo);
    /** Host path of an ImgFrame .mcap sidecar. */
    ReplayVideo& setReplayMetadataFile(const std::filesystem::path& replayFile);
    /** Output type. Checked at start against the supported set, except an unset .dai stream. */
    ReplayVideo& setOutFrameType(ImgFrame::Type outFrameType);
    ReplayVideo& setSize(std::tuple<int, int> size);
    ReplayVideo& setSize(int width, int height);
    /** Replay rate. 0 is the same as not set. 1000 schedules one frame per millisecond. */
    ReplayVideo& setFps(float fps);
    ReplayVideo& setLoop(bool loop);
    /**
     * Source path. HOST is the default. A DEVICE path must be absolute.
     * setReplayVideoFile is setSource(path, HOST).
     */
    ReplayVideo& setSource(const std::filesystem::path& path, ReplaySourceLocation location = ReplaySourceLocation::HOST);
    /** Metadata .mcap. setReplayMetadataFile is this setter. */
    ReplayVideo& setMetadataFile(const std::filesystem::path& path);
    ReplayVideo& setDecodeMode(ReplayDecodeMode mode);
    /** Force host (true) or device (false). The default follows the device. */
    ReplayVideo& setRunOnHost(bool runOnHost);
    /** Keep the cache entries of this start after the pipeline stops. The default removes them. */
    ReplayVideo& setPersistentUpload(bool persistent);
    /**
     * Select one stream inside an archive or a holistic directory.
     * The entry is a record name such as CameraCAM_A.
     */
    ReplayVideo& setArchiveEntry(const std::string& entry);
    /** With loop off, stop the pipeline after the last frame. Ignored when loop is on. */
    ReplayVideo& setStopPipelineAtEnd(bool stopPipeline);

    /** True for NV12, YUV420p, GRAY8, RGB888i, and BGR888i. */
    static bool isSupportedOutFrameType(ImgFrame::Type type);

    /** Host-cache limit in bytes. The default is 32 GiB. A new limit removes nothing by itself. */
    static void setReplayCacheLimit(std::uint64_t bytes);
    /** Host-cache limit, used bytes, and entries. */
    static ReplayCacheInfo getReplayCacheInfo();
    /** Remove host-cache entries that are not in use. */
    static void clearReplayCache();

   private:
    void releaseAfterStop();
    void invalidatePrepare();

    std::unique_ptr<ReplayRuntime> runtime;
    bool prepared = false;
};

/**
 * @brief Replays the messages of one .mcap file.
 *
 * On RVC4 the node runs on the device unless setRunOnHost(true) is set. The messages keep
 * their stored payload. ReplayMessage is another name for this class.
 */
class ReplayMetadataOnly : public DeviceNodeCRTP<DeviceNode, ReplayMetadataOnly, ReplayMetadataOnlyProperties>, public HostRunnable {
    friend class dai::PipelineImpl;
    friend void releaseEveryReplayPrepare(Pipeline& pipeline);

   public:
    constexpr static const char* NAME = "ReplayMetadataOnly";

    /**
     * Output for the replayed messages.
     * Default queue is blocking with size 8.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::Buffer, true}}}}};

    ReplayMetadataOnly();
    explicit ReplayMetadataOnly(const std::shared_ptr<Device>& device);
    explicit ReplayMetadataOnly(std::unique_ptr<Properties> props);
    ~ReplayMetadataOnly() override;

    /** Validate the file and fill the device properties. */
    void buildStage1() override;
    /** Send messages on the host. */
    void run() override;
    /** Report where this node runs. The value follows the device and setRunOnHost. */
    bool runOnHost() const override;

    std::filesystem::path getReplayFile() const;
    float getFps() const;
    bool getLoop() const;
    ReplaySourceLocation getSourceLocation() const;
    bool getPersistentUpload() const;
    std::string getArchiveEntry() const;
    bool getStopPipelineAtEnd() const;

    /** Host path of the .mcap file. */
    ReplayMetadataOnly& setReplayFile(const std::filesystem::path& replayFile);
    /** Replay rate. 0 is the same as not set. */
    ReplayMetadataOnly& setFps(float fps);
    ReplayMetadataOnly& setLoop(bool loop);
    /** Source path. HOST is the default. A DEVICE path must be absolute. */
    ReplayMetadataOnly& setSource(const std::filesystem::path& path, ReplaySourceLocation location = ReplaySourceLocation::HOST);
    /** Force host (true) or device (false). The default follows the device. */
    ReplayMetadataOnly& setRunOnHost(bool runOnHost);
    /** Keep the cache entries of this start after the pipeline stops. */
    ReplayMetadataOnly& setPersistentUpload(bool persistent);
    /** Inside an archive or a holistic directory, select `<entry>.mcap`. */
    ReplayMetadataOnly& setArchiveEntry(const std::string& entry);
    /** With loop off, stop the pipeline after the last message. Ignored when loop is on. */
    ReplayMetadataOnly& setStopPipelineAtEnd(bool stopPipeline);

   private:
    void releaseAfterStop();
    void invalidatePrepare();

    std::unique_ptr<ReplayRuntime> runtime;
    bool prepared = false;
};

using ReplayMessage = ReplayMetadataOnly;

}  // namespace node
}  // namespace dai
