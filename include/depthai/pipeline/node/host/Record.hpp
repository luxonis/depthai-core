#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// shared
#include <depthai/properties/internal/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/RecordControl.hpp>
#include <depthai/pipeline/datatype/RecordStatus.hpp>
#include <depthai/properties/RecordVideoProperties.hpp>
#include <depthai/utility/RecordReplay.hpp>
#include <mutex>
#include <optional>

#include "depthai/pipeline/ThreadedHostNode.hpp"

namespace dai {
namespace node {

using XLinkOutProperties = ::dai::internal::XLinkOutProperties;

struct RecordHostState;

/**
 * @brief RecordVideo node. Records ImgFrame or EncodedFrame messages to disk.
 *
 * The node runs on an RVC4 device unless setRunOnHost(true) is set, and on the host
 * for RVC2 and for a pipeline with no device. VIDEO writes a video file plus .mcap
 * metadata. RAW_FRAMES writes one .dai file per frame. Setters do not validate;
 * the pipeline checks the configuration at start.
 */
class RecordVideo : public DeviceNodeCRTP<DeviceNode, RecordVideo, RecordVideoProperties>, public HostRunnable {
    friend struct RecordHostState;

   public:
    using CompressionLevel = dai::RecordConfig::CompressionLevel;

    constexpr static const char* NAME = "RecordVideo";

    /**
     * Input for ImgFrame or EncodedFrame messages to be recorded.
     *
     * Default queue does not block and keeps the 15 newest messages.
     */
    Input input{*this, {"input", DEFAULT_GROUP, false, 15, {{{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::EncodedFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Optional RecordControl input. Does not block. Queue of 4.
     */
    Input inputControl{*this, {"inputControl", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::RecordControl, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * RecordStatus output. One message for each state change. Sending it does not drop a message.
     */
    Output status{*this, {"status", DEFAULT_GROUP, {{{DatatypeEnum::RecordStatus, false}}}}};

    using DeviceNodeCRTP::DeviceNodeCRTP;
    RecordVideo() = default;
    explicit RecordVideo(std::unique_ptr<Properties> props);
    ~RecordVideo() override;

    /** Validate configuration and create a host recording directory. */
    void buildStage1() override;
    /** Record on the host. Device execution is performed by firmware. */
    void run() override;
    /** Report where this node runs. The value follows the device and setRunOnHost. */
    bool runOnHost() const override;

    std::filesystem::path getRecordMetadataFile() const;
    std::filesystem::path getRecordVideoFile() const;
    CompressionLevel getCompressionLevel() const;

    RecordVideo& setRecordMetadataFile(const std::filesystem::path& recordFile);
    RecordVideo& setRecordVideoFile(const std::filesystem::path& recordFile);
    RecordVideo& setCompressionLevel(CompressionLevel compressionLevel);
    /** Frames per second written into the video file. The value is not measured from the stream. */
    RecordVideo& setFps(unsigned int fps);
    /**
     * Fractional frames per second. Holistic record uses this when a camera output is not a whole number.
     * setFps(unsigned int) remains the setter for a whole number of frames per second.
     */
    RecordVideo& setStreamFps(float fps);

    /** Select VIDEO or RAW_FRAMES. */
    RecordVideo& setFormat(RecordFormat format);
    /** Directory name of the recording. The default is recording_<local time with milliseconds>. */
    RecordVideo& setRecordingName(const std::string& name);
    /** Host directory that holds the recording, or that receives it at pipeline stop. */
    RecordVideo& setOutputDirectory(const std::filesystem::path& path);
    /** Stream name. The default is frames. */
    RecordVideo& setStreamName(const std::string& name);
    /** Start idle until a RecordControl START arrives. */
    RecordVideo& setWaitForTrigger(bool waitForTrigger);
    /** Number of newest idle frames written at the next START. 0 keeps none. */
    RecordVideo& setPreTriggerFrames(std::int32_t frames);
    /** Byte limit of the pre-trigger buffer. */
    RecordVideo& setMaxPreTriggerBytes(std::uint64_t bytes);
    /** On device execution, copy the recording to the host at pipeline stop. */
    RecordVideo& setTransferToHost(bool transfer);
    /** On device execution, keep the device copy after the transfer. */
    RecordVideo& setKeepOnDevice(bool keep);
    /** Force host (true) or device (false). The default follows the device. */
    RecordVideo& setRunOnHost(bool runOnHost);

    RecordFormat getFormat() const;
    std::string getRecordingName() const;
    std::filesystem::path getOutputDirectory() const;
    std::string getStreamName() const;
    float getFps() const;
    bool getWaitForTrigger() const;
    std::int32_t getPreTriggerFrames() const;
    std::uint64_t getMaxPreTriggerBytes() const;
    bool getTransferToHost() const;
    bool getKeepOnDevice() const;
    /** True when setRecordVideoFile or setRecordMetadataFile was called. */
    bool isLegacyLayout() const;

    /**
     * Last status message, or no value before the first one.
     * On the host this is set even when status is not linked.
     */
    std::optional<RecordStatus> getStatus() const;

   private:
    struct ResolvedConfig {
        bool ready = false;
        bool onHost = true;
        bool legacy = false;
        RecordFormat format = RecordFormat::VIDEO;
        std::string recordingName;
        std::filesystem::path outputDirectory;
        std::string streamName;
        float fps = 30.f;
        CompressionLevel compression = CompressionLevel::DEFAULT;
        bool waitForTrigger = false;
        std::int32_t preTriggerFrames = 60;
        std::uint64_t maxPreTriggerBytes = 268435456ull;
        bool transferToHost = true;
        bool keepOnDevice = false;
        std::filesystem::path recordVideoFile;
        std::filesystem::path recordMetadataFile;
    };

    ResolvedConfig resolved;
    mutable std::mutex statusMutex;
    std::optional<RecordStatus> lastStatus;
    bool rawProducer = false;

    bool deviceIsRvc4() const;
    void publishStatus(const std::shared_ptr<RecordStatus>& message);
};

/**
 * @brief RecordMetadataOnly node, used to record a source stream to a file
 */
class RecordMetadataOnly : public NodeCRTP<ThreadedHostNode, RecordMetadataOnly> {
   public:
    using CompressionLevel = dai::RecordConfig::CompressionLevel;

    constexpr static const char* NAME = "RecordMetadataOnly";

    /**
     * Input IMU messages to be recorded (will support other types in the future)
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, 15, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    std::filesystem::path getRecordFile() const;
    CompressionLevel getCompressionLevel() const;

    RecordMetadataOnly& setRecordFile(const std::filesystem::path& recordFile);
    RecordMetadataOnly& setCompressionLevel(CompressionLevel compressionLevel);

   private:
    std::filesystem::path recordFile;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
};

using RecordMessage = RecordMetadataOnly;

}  // namespace node
}  // namespace dai
