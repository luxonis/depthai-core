#include "depthai/pipeline/node/host/Record.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "depthai/config/config.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/utility/RecordingStore.hpp"
#include "depthai/utility/span.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/RecordH264Cut.hpp"
#include "utility/RecordReplayImpl.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/imgproc.hpp>
#endif
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/EncodedFrame.pb.h"
    #include "depthai/schemas/IMUData.pb.h"
    #include "depthai/schemas/ImgFrame.pb.h"
    #include "depthai/schemas/PointCloudData.pb.h"
    #include "depthai/utility/ProtoSerializable.hpp"
#endif

namespace dai {
namespace node {

namespace {

enum class FrameKind { None, Image, Encoded };

struct HeldFrame {
    std::shared_ptr<Buffer> message;
    std::chrono::steady_clock::time_point received;
    std::uint64_t bytes = 0;
};

bool isEnospace(const std::error_code& error) {
    return error == std::errc::no_space_on_device;
}

bool diskFull(const std::filesystem::path& directory, std::uint64_t bytes) {
    std::error_code error;
    const auto space = std::filesystem::space(directory.empty() ? std::filesystem::path(".") : directory, error);
    if(error) return false;
    return space.available < bytes;
}

std::string sequenceFileName(std::int64_t sequence) {
    if(sequence >= 0 && sequence <= 99999999) {
        std::ostringstream text;
        text << std::setw(8) << std::setfill('0') << sequence;
        return text.str();
    }
    return std::to_string(sequence);
}

RecordConfig::CompressionLevel compressionFrom(std::int32_t value) {
    if(value < 0 || value > static_cast<std::int32_t>(RecordConfig::CompressionLevel::SLOWEST)) {
        return RecordConfig::CompressionLevel::DEFAULT;
    }
    return static_cast<RecordConfig::CompressionLevel>(value);
}

}  // namespace

struct RecordHostState {
    RecordVideo& node;
    std::shared_ptr<spdlog::async_logger> logger;
    enum class Phase { Idle, Recording } phase = Phase::Idle;
    bool fatal = false;
    bool stopRequested = false;
    bool pt4Checked = false;
    bool preFullWarned = false;
    bool warnedH264 = false;
    bool warnedH265 = false;
    bool filesOpen = false;
    bool videoOpened = false;
    FrameKind kind = FrameKind::None;
    std::int32_t segmentIndex = -1;
    std::int32_t lastClosedSegment = -1;
    std::int32_t statusSequence = 0;
    std::int32_t maxFramesLimit = -1;
    std::int32_t maxDurationLimit = -1;
    std::int32_t framesInSegment = 0;
    std::int32_t skipped = 0;
    std::int64_t firstSeq = -1;
    std::int64_t lastSeq = -1;
    bool haveLast = false;
    bool rateWarned = false;
    std::uint32_t videoWidth = 0;
    std::uint32_t videoHeight = 0;
    int videoProfile = -1;
    bool videoShapeSet = false;
    std::size_t pretriggerRemaining = 0;
    std::uint64_t preBytes = 0;
    std::deque<HeldFrame> pretrigger;
    std::deque<HeldFrame> pending;
    std::deque<HeldFrame> incoming;
    std::chrono::steady_clock::time_point segmentStart{};
    std::unique_ptr<utility::VideoRecorder> video;
    std::unique_ptr<utility::ByteRecorder> metadata;
    // H.264 files start on an IDR. Frames before that IDR are not written.
    // maxFrames and maxDurationMs open the next segment on a later IDR instead
    // of stopping. The host cannot ask the device encoder for that IDR.
    bool waitingForIdr = false;
    bool splitArmed = false;
    bool avcMode = false;
    bool finishing = false;
    std::vector<std::uint8_t> h264Sps;
    std::vector<std::uint8_t> h264Pps;

    explicit RecordHostState(RecordVideo& self) : node(self), logger(self.pimpl->logger) {}

    std::filesystem::path recordingDirectory() const {
        return node.resolved.outputDirectory / node.resolved.recordingName;
    }

    void publish(RecordStatus::State state, const std::string& error, bool idleValues) {
        auto message = std::make_shared<RecordStatus>();
        message->state = state;
        message->path = recordingDirectory().string();
        message->error = error;
        if(idleValues) {
            message->segment = lastClosedSegment;
            message->frameCount = 0;
            message->firstSequenceNum = -1;
            message->lastSequenceNum = -1;
            message->skippedFrames = 0;
        } else {
            message->segment = segmentIndex;
            message->frameCount = framesInSegment;
            message->firstSequenceNum = framesInSegment == 0 ? -1 : firstSeq;
            message->lastSequenceNum = framesInSegment == 0 ? -1 : lastSeq;
            message->skippedFrames = skipped;
        }
        const auto now = std::chrono::steady_clock::now();
        message->setTimestamp(now);
        message->setTimestampDevice(now);
        message->setSequenceNum(statusSequence++);
        node.publishStatus(message);
    }

    void fail(const std::string& text) {
        if(!fatal) {
            fatal = true;
            const bool mp4Ok = closeFilesQuiet();
            std::string error = text;
            if(videoOpened && text.find("storage is full") != std::string::npos && text.find("mp4 close ") == std::string::npos) {
                error += mp4Ok ? "; mp4 close succeeded" : "; mp4 close failed";
            }
            const bool idle = phase != Phase::Recording;
            publish(RecordStatus::State::ERROR, error, idle);
            logger->error("{}", error);
            throw std::runtime_error(error);
        }
        throw std::runtime_error(text);
    }

    void writeMarker(const std::string& contents) {
        if(!node.resolved.onHost || node.resolved.legacy) return;
        const auto path = recordingDirectory() / (".dai_closed_" + node.resolved.streamName);
        std::error_code error;
        std::filesystem::create_directories(path.parent_path(), error);
        std::ofstream output(path, std::ios::trunc);
        if(output) output << contents;
    }

    bool closeFilesQuiet(std::string* failure = nullptr) {
        bool mp4Ok = true;
        const auto noteFailure = [&](const char* kind, const std::exception& error) {
            if(failure == nullptr) return;
            if(!failure->empty()) *failure += "; ";
            if(errno == ENOSPC) *failure += "Record storage is full; ";
            *failure += std::string("Record ") + kind + " close failed: " + error.what();
        };
        if(metadata) {
            try {
                metadata->close();
            } catch(const std::exception& error) {
                noteFailure("metadata", error);
            }
            metadata.reset();
        }
        if(video) {
            try {
                video->close();
            } catch(const std::exception& error) {
                mp4Ok = false;
                noteFailure("video", error);
            }
            video.reset();
        }
        filesOpen = false;
        return mp4Ok;
    }

    void ensureSpace(const std::filesystem::path& directory, std::uint64_t bytes) {
        if(diskFull(directory, bytes)) {
            fail("Record storage is full");
        }
    }

    void noteSkip(std::int64_t sequence) {
        if(!haveLast) return;
        if(sequence > lastSeq + 1) {
            const auto gap = sequence - lastSeq - 1;
            const auto room = static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) - skipped;
            skipped += static_cast<std::int32_t>(std::min(gap, std::max<std::int64_t>(room, 0)));
            logger->warn("Record skipped {} frames ({} -> {})", gap, lastSeq, sequence);
        } else if(sequence <= lastSeq) {
            logger->warn("Record sequence went back ({} -> {})", lastSeq, sequence);
        }
    }

    void noteRate(std::uint64_t bytes) {
        if(rateWarned) return;
        rateWarned = true;
        const double rate = static_cast<double>(bytes) * static_cast<double>(node.resolved.fps);
        if(rate > 100000000.0) {
            logger->warn("Record expected write rate {} MB/s, frames can be skipped", std::llround(rate / 1000000.0));
        }
    }

    void accept(const std::shared_ptr<Buffer>& message) {
        const auto image = std::dynamic_pointer_cast<ImgFrame>(message);
        const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(message);
        if(!image && !encoded) {
            fail("Record input accepts only ImgFrame and EncodedFrame");
        }
        if(image && image->getType() == ImgFrame::Type::BITSTREAM) {
            fail("Record BITSTREAM ImgFrame is not supported");
        }
        const FrameKind messageKind = image ? FrameKind::Image : FrameKind::Encoded;
        if(kind == FrameKind::None) {
            kind = messageKind;
        } else if(kind != messageKind) {
            fail("Record message type changed");
        }
        if(node.resolved.format == RecordFormat::VIDEO && encoded && encoded->getProfile() == EncodedFrame::Profile::HEVC) {
            fail("Record H265 is not supported in VIDEO format");
        }
        if(node.resolved.format == RecordFormat::RAW_FRAMES && encoded) {
            if(encoded->getProfile() == EncodedFrame::Profile::HEVC && !warnedH265) {
                warnedH265 = true;
                logger->warn("Record H265 frames cannot be replayed");
            }
            if(encoded->getProfile() == EncodedFrame::Profile::AVC && !warnedH264) {
                warnedH264 = true;
                logger->warn("Record H264 frames cannot be replayed");
            }
        }
    }

    void checkVideoShape(const std::shared_ptr<Buffer>& message) {
        if(node.resolved.format != RecordFormat::VIDEO) return;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        int profile = -1;
        if(const auto image = std::dynamic_pointer_cast<ImgFrame>(message)) {
            width = image->getWidth();
            height = image->getHeight();
            profile = static_cast<int>(image->getType());
        } else if(const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(message)) {
            width = encoded->getWidth();
            height = encoded->getHeight();
            profile = static_cast<int>(encoded->getProfile());
        }
        if(!videoShapeSet) {
            videoWidth = width;
            videoHeight = height;
            videoProfile = profile;
            videoShapeSet = true;
            return;
        }
        if(width != videoWidth || height != videoHeight || profile != videoProfile) {
            fail("Record video profile or size changed");
        }
    }

    void openWriters(const std::shared_ptr<Buffer>& message) {
        if(filesOpen || node.resolved.format == RecordFormat::RAW_FRAMES) return;
        std::filesystem::path videoPath;
        std::filesystem::path metadataPath;
        if(node.resolved.legacy) {
            videoPath = pathWithSegment(node.resolved.recordVideoFile, segmentIndex);
            metadataPath = pathWithSegment(node.resolved.recordMetadataFile, segmentIndex);
        } else {
            const std::string suffix = segmentIndex > 0 ? "_s" + std::to_string(segmentIndex) : "";
            const auto directory = recordingDirectory();
            const bool image = kind == FrameKind::Image;
            videoPath = directory / (node.resolved.streamName + suffix + (image ? ".avi" : ".mp4"));
            metadataPath = directory / (node.resolved.streamName + suffix + ".mcap");
        }
        if(!videoPath.empty()) {
            std::filesystem::create_directories(videoPath.parent_path());
            ensureSpace(videoPath.parent_path(), message->getData().size());
            video = std::make_unique<utility::VideoRecorder>();
            const auto fps = static_cast<unsigned int>(std::max<long>(1, std::lround(node.resolved.fps)));
            if(kind == FrameKind::Encoded) {
                const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(message);
                const auto codec =
                    encoded->getProfile() == EncodedFrame::Profile::JPEG ? utility::VideoRecorder::VideoCodec::MJPEG : utility::VideoRecorder::VideoCodec::H264;
                video->init(videoPath.string(), encoded->getWidth(), encoded->getHeight(), fps, codec);
                if(codec == utility::VideoRecorder::VideoCodec::H264 && h264Sps.size() >= 4 && !h264Pps.empty()) {
                    video->setH264ParameterSets(h264Sps, h264Pps);
                }
            } else {
                const auto image = std::dynamic_pointer_cast<ImgFrame>(message);
                if(!image) fail("Record video frame is not an ImgFrame");
                video->init(videoPath.string(), image->getWidth(), image->getHeight(), fps, utility::VideoRecorder::VideoCodec::RAW);
            }
            videoOpened = true;
        }
#ifdef DEPTHAI_ENABLE_PROTOBUF
        if(!metadataPath.empty()) {
            std::filesystem::create_directories(metadataPath.parent_path());
            metadata = std::make_unique<utility::ByteRecorder>();
            metadata->init<dai::proto::img_frame::ImgFrame>(metadataPath.string(), node.resolved.compression, "video");
        }
#else
        if(!metadataPath.empty()) {
            fail("RecordVideo requires protobuf support");
        }
#endif
        filesOpen = true;
    }

    void writeVideo(const std::shared_ptr<Buffer>& message) {
        openWriters(message);
        if(!video) return;
        ensureSpace(recordingDirectory(), message->getData().size());
        if(kind == FrameKind::Image) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
            const auto image = std::dynamic_pointer_cast<ImgFrame>(message);
            auto frame = image->getCvFrame();
            const bool grayscale = image->getType() == ImgFrame::Type::GRAY8 || image->getType() == ImgFrame::Type::GRAYF16
                                   || (ImgFrame::Type::RAW16 <= image->getType() && image->getType() <= ImgFrame::Type::RAW8);
            if(grayscale) cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
            span<uint8_t> view(frame.data, frame.total() * frame.elemSize());
            video->write(view, static_cast<std::uint32_t>(frame.step));
    #ifdef DEPTHAI_ENABLE_PROTOBUF
            if(metadata) metadata->write(image->serializeProto(true));
    #endif
#else
            fail("RecordVideo raw video requires OpenCV support");
#endif
        } else {
            auto view = message->getData();
            video->write(view);
#ifdef DEPTHAI_ENABLE_PROTOBUF
            if(metadata) {
                const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(message);
                auto imageMeta = encoded->getImgFrameMeta();
                metadata->write(imageMeta.serializeProto(true));
            }
#endif
        }
    }

    void writeRaw(const std::shared_ptr<Buffer>& message) {
#ifndef DEPTHAI_ENABLE_PROTOBUF
        (void)message;
        fail("RecordVideo RAW_FRAMES requires protobuf support");
#else
        const auto sequence = message->getSequenceNum();
        if(sequence < 0) {
            fail("Record sequenceNum " + std::to_string(sequence) + " is below 0");
        }
        const auto directory = recordingDirectory() / node.resolved.streamName;
        std::filesystem::create_directories(directory);
        const auto finalPath = directory / (node.resolved.streamName + "_" + sequenceFileName(sequence) + ".dai");
        if(std::filesystem::exists(finalPath)) {
            fail("Record file already exists: " + finalPath.filename().string());
        }
        ensureSpace(directory, message->getData().size() + 4096);
        const auto temporary = std::filesystem::path(finalPath.string() + ".tmp");
        std::filesystem::remove(temporary);
        const auto serializable = std::dynamic_pointer_cast<ProtoSerializable>(message);
        if(!serializable) fail("Record frame cannot be saved as .dai");
        try {
            serializable->save(temporary);
        } catch(const std::exception&) {
            std::filesystem::remove(temporary);
            if(diskFull(directory, 1)) fail("Record storage is full");
            throw;
        }
        std::error_code error;
        std::filesystem::rename(temporary, finalPath, error);
        if(error) {
            std::filesystem::remove(temporary);
            if(isEnospace(error)) fail("Record storage is full");
            fail("Record failed to name " + finalPath.filename().string() + ": " + error.message());
        }
#endif
    }

    void writeFrame(const HeldFrame& held) {
        try {
            checkVideoShape(held.message);
            noteRate(held.bytes);
            noteSkip(held.message->getSequenceNum());
            if(node.resolved.format == RecordFormat::RAW_FRAMES) {
                writeRaw(held.message);
            } else {
                writeVideo(held.message);
            }
        } catch(const std::runtime_error& error) {
            if(fatal) throw;
            const std::string text = error.what();
            if(text.find("ENOSPC") != std::string::npos || text.find("No space") != std::string::npos || diskFull(recordingDirectory(), 1)) {
                fail("Record storage is full");
            }
            fail(text);
        }
        if(framesInSegment == 0) firstSeq = held.message->getSequenceNum();
        lastSeq = held.message->getSequenceNum();
        haveLast = true;
        if(framesInSegment < std::numeric_limits<std::int32_t>::max()) ++framesInSegment;
        logger->debug("Record wrote frame {}", held.message->getSequenceNum());
    }

    void dropOldestIdle() {
        if(pretrigger.empty()) return;
        preBytes -= pretrigger.front().bytes;
        pretrigger.pop_front();
    }

    void pushIdle(const HeldFrame& held) {
        rememberH264(held.message);
        if(node.resolved.preTriggerFrames <= 0) return;
        if(!pt4Checked) {
            pt4Checked = true;
            const auto need = static_cast<std::uint64_t>(node.resolved.preTriggerFrames) * held.bytes;
            if(need > node.resolved.maxPreTriggerBytes) {
                fail("Record pre-trigger buffer " + std::to_string(need) + " bytes exceeds " + std::to_string(node.resolved.maxPreTriggerBytes));
            }
        }
        while(static_cast<std::int32_t>(pretrigger.size()) >= node.resolved.preTriggerFrames) dropOldestIdle();
        bool droppedForBytes = false;
        while(!pretrigger.empty() && preBytes + held.bytes > node.resolved.maxPreTriggerBytes) {
            dropOldestIdle();
            droppedForBytes = true;
        }
        if(preBytes + held.bytes > node.resolved.maxPreTriggerBytes) {
            droppedForBytes = true;
        } else {
            pretrigger.push_back(held);
            preBytes += held.bytes;
        }
        if(droppedForBytes && !preFullWarned) {
            preFullWarned = true;
            logger->warn("Record pre-trigger buffer full");
        }
    }

    bool isAvc(const std::shared_ptr<Buffer>& message) const {
        if(node.resolved.format != RecordFormat::VIDEO) return false;
        const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(message);
        return encoded && encoded->getProfile() == EncodedFrame::Profile::AVC;
    }

    bool isIdrFrame(const std::shared_ptr<Buffer>& message) const {
        return isAvc(message) && utility::EncodedVideoRecorder::containsIdr(message->getData());
    }

    void rememberH264(const std::shared_ptr<Buffer>& message) {
        if(!isAvc(message)) return;
        const auto sets = utility::EncodedVideoRecorder::peekH264ParameterSets(message->getData());
        if(!sets.sps.empty()) h264Sps = sets.sps;
        if(!sets.pps.empty()) h264Pps = sets.pps;
    }

    void dropUntilKeyframe(const HeldFrame& held) {
        rememberH264(held.message);
        logger->debug("Record dropped frame {} until the segment keyframe", held.message->getSequenceNum());
    }

    void handoffIncoming() {
        if(pretriggerRemaining != 0) return;
        for(auto& frame : incoming) pending.push_back(std::move(frame));
        incoming.clear();
    }

    void armSplit() {
        if(splitArmed || !avcMode || framesInSegment < 1) return;
        const bool framesDue = utility::h264FrameCutDue(framesInSegment, maxFramesLimit);
        const bool timeDue = utility::h264DurationCutDue(segmentStart, std::chrono::steady_clock::now(), maxDurationLimit, static_cast<double>(node.resolved.fps));
        if(!framesDue && !timeDue) return;
        splitArmed = true;
    }

    void rollSegment() {
        const auto frames = maxFramesLimit;
        const auto duration = maxDurationLimit;
        std::string closeFailure;
        const bool mp4Ok = closeFilesQuiet(&closeFailure);
        if(!closeFailure.empty()) {
            if(videoOpened) closeFailure += mp4Ok ? "; mp4 close succeeded" : "; mp4 close failed";
            fail(closeFailure);
        }
        writeMarker("closed");
        publish(RecordStatus::State::STOPPED, "", false);
        ++segmentIndex;
        phase = Phase::Recording;
        segmentStart = std::chrono::steady_clock::now();
        maxFramesLimit = frames;
        maxDurationLimit = duration;
        framesInSegment = 0;
        skipped = 0;
        firstSeq = -1;
        lastSeq = -1;
        haveLast = false;
        rateWarned = false;
        videoShapeSet = false;
        videoOpened = false;
        filesOpen = false;
        waitingForIdr = false;
        splitArmed = false;
        writeMarker("open");
        publish(RecordStatus::State::STARTED, "", false);
    }

    void dropUndecodablePretrigger() {
        if(node.resolved.format != RecordFormat::VIDEO) {
            waitingForIdr = false;
            return;
        }
        if(pending.empty() || !isAvc(pending.front().message)) {
            waitingForIdr = pending.empty();
            return;
        }
        avcMode = true;
        std::deque<HeldFrame> kept;
        bool seenIdr = false;
        for(auto& held : pending) {
            if(!seenIdr && !isIdrFrame(held.message)) {
                dropUntilKeyframe(held);
                continue;
            }
            seenIdr = true;
            kept.push_back(std::move(held));
        }
        pending = std::move(kept);
        pretriggerRemaining = pending.size();
        waitingForIdr = !seenIdr;
    }

    bool pastDuration(std::chrono::steady_clock::time_point received) const {
        if(maxDurationLimit < 0 || phase != Phase::Recording) return false;
        return received >= segmentStart + std::chrono::milliseconds(maxDurationLimit);
    }

    bool durationReached() const {
        if(maxDurationLimit < 0 || phase != Phase::Recording) return false;
        return std::chrono::steady_clock::now() >= segmentStart + std::chrono::milliseconds(maxDurationLimit);
    }

    void beginSegment(std::int32_t maxFrames, std::int32_t maxDurationMs) {
        ++segmentIndex;
        phase = Phase::Recording;
        segmentStart = std::chrono::steady_clock::now();
        maxFramesLimit = maxFrames;
        maxDurationLimit = maxDurationMs;
        framesInSegment = 0;
        skipped = 0;
        firstSeq = -1;
        lastSeq = -1;
        haveLast = false;
        rateWarned = false;
        videoShapeSet = false;
        videoOpened = false;
        filesOpen = false;
        stopRequested = false;
        if(maxFrames >= 0) {
            while(static_cast<std::int32_t>(pretrigger.size()) > maxFrames) dropOldestIdle();
        }
        pending = std::move(pretrigger);
        pretrigger.clear();
        preBytes = 0;
        pretriggerRemaining = pending.size();
        incoming.clear();
        splitArmed = false;
        dropUndecodablePretrigger();
        writeMarker("open");
        publish(RecordStatus::State::STARTED, "", false);
    }

    void closeSegment() {
        if(phase != Phase::Recording) {
            stopRequested = false;
            return;
        }
        std::string closeFailure;
        const bool mp4Ok = closeFilesQuiet(&closeFailure);
        if(!closeFailure.empty()) {
            if(videoOpened) closeFailure += mp4Ok ? "; mp4 close succeeded" : "; mp4 close failed";
            fail(closeFailure);
        }
        writeMarker("closed");
        publish(RecordStatus::State::STOPPED, "", false);
        phase = Phase::Idle;
        lastClosedSegment = segmentIndex;
        stopRequested = false;
        maxFramesLimit = -1;
        maxDurationLimit = -1;
        pretriggerRemaining = 0;
        videoShapeSet = false;
    }

    void moveRestToIdle() {
        for(const auto& frame : pending) pushIdle(frame);
        pending.clear();
        for(const auto& frame : incoming) pushIdle(frame);
        incoming.clear();
        pretriggerRemaining = 0;
    }

    void writeOne() {
        HeldFrame held = std::move(pending.front());
        pending.pop_front();
        if(pretriggerRemaining > 0) --pretriggerRemaining;
        const bool avc = isAvc(held.message);
        if(avc) avcMode = true;
        const bool idr = avc && utility::EncodedVideoRecorder::containsIdr(held.message->getData());
        if(waitingForIdr) {
            if(!avc) {
                waitingForIdr = false;
            } else if(!idr) {
                dropUntilKeyframe(held);
                handoffIncoming();
                return;
            } else {
                waitingForIdr = false;
            }
        }
        const bool hardFrames = maxFramesLimit >= 0 && framesInSegment >= maxFramesLimit;
        const bool hardTime = pastDuration(held.received);
        // A limit rolls H.264 onto the next IDR. Frames before that IDR stay in
        // this segment, even when that carries it past the limit.
        if(avc && idr && framesInSegment > 0 && (splitArmed || hardFrames || hardTime) && !stopRequested && !finishing) {
            rollSegment();
        } else if(hardTime && !avc) {
            closeSegment();
            logger->debug("Record idle frame {}", held.message->getSequenceNum());
            pushIdle(held);
            moveRestToIdle();
            return;
        }
        if(avc && framesInSegment > 0 && (hardFrames || hardTime)) {
            logger->debug("Record kept frame {} past the segment limit", held.message->getSequenceNum());
        }
        writeFrame(held);
        if(avc)
            armSplit();
        else if(maxFramesLimit >= 0 && framesInSegment >= maxFramesLimit) {
            closeSegment();
            moveRestToIdle();
            return;
        }
        handoffIncoming();
    }

    void pullIncoming() {
        while(auto message = node.input.tryGet<Buffer>()) {
            accept(message);
            incoming.push_back(HeldFrame{message, std::chrono::steady_clock::now(), message->getData().size()});
        }
    }

    void apply(const std::shared_ptr<RecordControl>& control) {
        if(!control) return;
        if(control->command == RecordControl::Command::STOP) {
            if(phase == Phase::Idle) {
                logger->debug("Record ignored stop");
            } else {
                stopRequested = true;
            }
            return;
        }
        if(phase == Phase::Recording || stopRequested) {
            logger->warn("Record start ignored");
            return;
        }
        if(control->maxFrames == 0 || control->maxDurationMs == 0) {
            logger->warn("Record limit of 0 ignored");
            return;
        }
        beginSegment(control->maxFrames, control->maxDurationMs);
    }

    void flushPending() {
        if(pending.empty() && !incoming.empty()) pending.swap(incoming);
        while(phase == Phase::Recording && !pending.empty()) writeOne();
    }

    void drainControls() {
        while(auto control = node.inputControl.tryGet<RecordControl>()) {
            if(stopRequested && phase == Phase::Recording) {
                flushPending();
                closeSegment();
            }
            apply(control);
        }
    }

    void noteFrame(const std::shared_ptr<Buffer>& message, std::chrono::steady_clock::time_point received) {
        accept(message);
        HeldFrame held{message, received, message->getData().size()};
        if(phase == Phase::Idle) {
            logger->debug("Record idle frame {}", message->getSequenceNum());
            pushIdle(held);
            return;
        }
        // Video frames are queued even after the duration limit. H.264 may still
        // be waiting for the IDR that opens the next segment, and MJPEG closes
        // from writeOne so this frame is not written into the closed segment.
        if(pastDuration(received) && node.resolved.format != RecordFormat::VIDEO) {
            closeSegment();
            logger->debug("Record idle frame {}", message->getSequenceNum());
            pushIdle(held);
            return;
        }
        pending.push_back(std::move(held));
    }

    void onPipelineStop() {
        if(fatal) return;
        if(phase == Phase::Recording) {
            finishing = true;
            flushPending();
            closeSegment();
        } else {
            // An idle pipeline stop discards the pre-trigger buffer and leaves the closed-marker open.
            pretrigger.clear();
            preBytes = 0;
        }
    }

    void run() {
#ifndef DEPTHAI_ENABLE_PROTOBUF
        fail("RecordVideo requires protobuf support");
#endif
        if(!node.resolved.waitForTrigger) beginSegment(-1, -1);
        while(node.mainLoop() && !fatal) {
            try {
                drainControls();
                if(phase == Phase::Recording && pretriggerRemaining > 0 && !stopRequested) pullIncoming();
                if(!pending.empty()) {
                    writeOne();
                    continue;
                }
                if(stopRequested && phase == Phase::Recording) {
                    closeSegment();
                    continue;
                }
                if(durationReached()) {
                    // Armed H.264 keeps the segment open until the split IDR, or until
                    // stop. Closing here would discard the frames still to come.
                    if(!(avcMode && splitArmed && framesInSegment > 0)) closeSegment();
                    continue;
                }
                // Take a frame only after the controls already queued have been applied.
                // Blocking on the frame queue would handle a frame that arrived after a
                // start or stop still sitting in inputControl.
                if(auto message = node.input.tryGet<Buffer>()) {
                    noteFrame(message, std::chrono::steady_clock::now());
                    continue;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            } catch(const MessageQueue::QueueException&) {
                break;
            }
        }
        onPipelineStop();
    }
};

RecordVideo::RecordVideo(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, RecordVideo, RecordVideoProperties>(std::move(props)) {}

RecordVideo::~RecordVideo() = default;

bool RecordVideo::deviceIsRvc4() const {
    const auto device = getDevice();
    if(!device) return false;
    try {
        return device->getPlatform() == Platform::RVC4;
    } catch(const std::exception&) {
        return false;
    }
}

bool RecordVideo::runOnHost() const {
    if(properties.runOnHostMode == 1) return true;
    if(properties.runOnHostMode == 2) return false;
    if(!deviceIsRvc4()) return true;
    const bool legacy = properties.recordVideoFileSet || properties.recordMetadataFileSet;
    if(!legacy) return false;
    bool linked = false;
    bool rawOnly = true;
    try {
        // Connections are stored on the producing node, so scan the whole map for this input.
        const auto connectionMap = getParentPipeline().getConnectionMap();
        for(const auto& entry : connectionMap) {
            for(const auto& connection : entry.second) {
                if(connection.in != &input) continue;
                linked = true;
                const auto types = connection.out->getPossibleDatatypes();
                if(types.empty()) rawOnly = false;
                for(const auto& type : types) {
                    if(type.datatype != DatatypeEnum::ImgFrame || type.descendants) rawOnly = false;
                }
            }
        }
    } catch(const std::exception&) {
        return false;
    }
    return linked && rawOnly;
}

void RecordVideo::buildStage1() {
    auto& logger = pimpl->logger;
    const bool legacy = properties.recordVideoFileSet || properties.recordMetadataFileSet;
    const bool onHost = runOnHost();
    if(properties.runOnHostMode == 2 && !deviceIsRvc4()) {
        throw std::runtime_error("Record setRunOnHost(false) is not supported without an RVC4 device");
    }
    bool rawOnly = false;
    bool linked = false;
    // Connections are stored on the producing node, so scan the whole map for this input.
    const auto connectionMap = getParentPipeline().getConnectionMap();
    for(const auto& entry : connectionMap) {
        for(const auto& connection : entry.second) {
            if(connection.in != &input) continue;
            linked = true;
            rawOnly = true;
            const auto types = connection.out->getPossibleDatatypes();
            if(types.empty()) rawOnly = false;
            for(const auto& type : types) {
                if(type.datatype != DatatypeEnum::ImgFrame || type.descendants) rawOnly = false;
            }
            if(!rawOnly) break;
        }
        if(linked && !rawOnly) break;
    }
    rawOnly = linked && rawOnly;
    rawProducer = rawOnly;
    if(properties.runOnHostMode == 2 && legacy && rawOnly) {
        throw std::runtime_error("Record setRunOnHost(false) cannot record raw frames with the legacy layout");
    }
    if(properties.format != 0 && properties.format != 1) {
        throw std::runtime_error("Record invalid format");
    }
    const auto format = properties.format == static_cast<std::int32_t>(RecordFormat::RAW_FRAMES) ? RecordFormat::RAW_FRAMES : RecordFormat::VIDEO;
    if(legacy && format == RecordFormat::RAW_FRAMES) {
        throw std::runtime_error("Record RAW_FRAMES cannot be used with setRecordVideoFile or setRecordMetadataFile");
    }
    if(!(properties.fps > 0.f) || !std::isfinite(properties.fps)) {
        throw std::runtime_error("Record invalid fps");
    }
    if(properties.preTriggerFrames < 0) {
        throw std::runtime_error("Record invalid preTriggerFrames");
    }
    if(properties.maxPreTriggerBytes == 0) {
        throw std::runtime_error("Record invalid maxPreTriggerBytes");
    }
    if(!properties.outputDirectorySet) {
        properties.outputDirectory = std::filesystem::current_path().string();
    }
    if(properties.outputDirectory.empty()) {
        throw std::runtime_error("Record invalid outputDirectory");
    }
    if(!properties.recordingNameSet || properties.recordingName.empty()) {
        properties.recordingName = makeDefaultRecordingName();
    }
    validateRecordingName(properties.recordingName);
    std::string streamName = properties.streamName;
    if(legacy) {
        streamName = std::filesystem::path(properties.recordVideoFile).stem().string();
        if(streamName.empty()) streamName = std::filesystem::path(properties.recordMetadataFile).stem().string();
        if(streamName.empty()) streamName = "video";
    } else {
        validateStreamName(streamName);
    }

    resolved = ResolvedConfig{};
    resolved.ready = true;
    resolved.onHost = onHost;
    resolved.legacy = legacy;
    resolved.format = format;
    resolved.recordingName = properties.recordingName;
    resolved.outputDirectory = properties.outputDirectory;
    resolved.streamName = streamName;
    resolved.fps = properties.fps;
    resolved.compression = compressionFrom(properties.compressionLevel);
    resolved.waitForTrigger = properties.waitForTrigger;
    resolved.preTriggerFrames = properties.preTriggerFrames;
    resolved.maxPreTriggerBytes = properties.maxPreTriggerBytes;
    resolved.transferToHost = properties.transferToHost;
    resolved.keepOnDevice = properties.keepOnDevice;
    resolved.recordVideoFile = properties.recordVideoFile;
    resolved.recordMetadataFile = properties.recordMetadataFile;

    for(const auto& candidate : getParentPipeline().getAllNodes()) {
        const auto other = std::dynamic_pointer_cast<RecordVideo>(candidate);
        if(!other || other.get() == this) continue;
        std::string otherName;
        std::string otherStream;
        bool otherLegacy = false;
        if(other->resolved.ready) {
            otherName = other->resolved.recordingName;
            otherStream = other->resolved.streamName;
            otherLegacy = other->resolved.legacy;
        } else if(other->properties.recordingNameSet) {
            otherName = other->properties.recordingName;
            otherLegacy = other->properties.recordVideoFileSet || other->properties.recordMetadataFileSet;
            otherStream = other->properties.streamName;
            if(otherLegacy) {
                otherStream = std::filesystem::path(other->properties.recordVideoFile).stem().string();
                if(otherStream.empty()) otherStream = std::filesystem::path(other->properties.recordMetadataFile).stem().string();
            }
        } else {
            continue;
        }
        if(otherName == resolved.recordingName && otherStream == resolved.streamName) {
            throw std::runtime_error("Record stream '" + resolved.streamName + "' is already used in recording '" + resolved.recordingName + "'");
        }
    }

    if(onHost && !legacy) {
        bool creator = true;
        for(const auto& candidate : getParentPipeline().getAllNodes()) {
            const auto other = std::dynamic_pointer_cast<RecordVideo>(candidate);
            if(!other || other.get() == this || !other->resolved.ready) continue;
            if(other->id < id && other->resolved.onHost && !other->resolved.legacy && other->resolved.recordingName == resolved.recordingName) {
                creator = false;
            }
        }
        const auto recordingDir = resolved.outputDirectory / resolved.recordingName;
        if(creator) {
            if(std::filesystem::exists(recordingDir)) {
                throw std::runtime_error("Record recording already exists: " + resolved.recordingName);
            }
            std::filesystem::create_directories(resolved.outputDirectory);
            std::filesystem::create_directory(recordingDir);
        }
        std::ofstream marker(recordingDir / (".dai_closed_" + resolved.streamName), std::ios::trunc);
        if(marker) marker << "open";
    } else if(!onHost) {
        if(!deviceStatusQueue) {
            deviceStatusQueue = status.createOutputQueue(1, false);
            const std::weak_ptr<RecordVideo> weakSelf = std::static_pointer_cast<RecordVideo>(shared_from_this());
            deviceStatusQueue->addCallback([weakSelf](std::shared_ptr<ADatatype> data) {
                const auto message = std::dynamic_pointer_cast<RecordStatus>(data);
                if(auto self = weakSelf.lock(); self && message) {
                    std::lock_guard<std::mutex> lock(self->statusMutex);
                    if(!self->lastStatus || message->getSequenceNum() > self->lastStatus->getSequenceNum()) self->lastStatus = *message;
                }
            });
        }
        try {
            const auto recordings = getDevice()->listRecordings();
            for(const auto& recording : recordings) {
                if(recording.name == resolved.recordingName) {
                    throw std::runtime_error("Record recording already exists: " + resolved.recordingName);
                }
            }
        } catch(const std::runtime_error& error) {
            if(std::string(error.what()).find("already exists") != std::string::npos) throw;
            logger->warn("Record could not check the device store: {}", error.what());
        }
    }

    if(onHost && properties.transferToHostSet) logger->warn("Record setTransferToHost ignored on the host");
    if(onHost && properties.keepOnDeviceSet) logger->warn("Record setKeepOnDevice ignored on the host");
    if(legacy && properties.outputDirectorySet) logger->warn("Record legacy layout: setOutputDirectory ignored");
    if(legacy && properties.streamNameSet) logger->warn("Record legacy layout: setStreamName ignored");
    if(properties.waitForTrigger && !inputControl.isConnected()) logger->warn("Record no trigger source");
}

void RecordVideo::run() {
    if(!resolved.ready) {
        throw std::runtime_error("RecordVideo started before the pipeline build");
    }
    if(!resolved.onHost) {
        throw std::runtime_error("RecordVideo device execution is implemented by device firmware");
    }
    RecordHostState state(*this);
    try {
        state.run();
    } catch(const MessageQueue::QueueException&) {
        state.onPipelineStop();
    }
}

void RecordVideo::publishStatus(const std::shared_ptr<RecordStatus>& message) {
    {
        std::lock_guard<std::mutex> lock(statusMutex);
        lastStatus = *message;
    }
    try {
        auto block = outputBlockEvent();
        status.send(message);
    } catch(const std::exception&) {
        // The status is kept for getStatus even when the output queue is already closed.
    }
}

std::filesystem::path RecordVideo::getRecordMetadataFile() const {
    if(resolved.ready) return resolved.recordMetadataFile;
    return properties.recordMetadataFile;
}
std::filesystem::path RecordVideo::getRecordVideoFile() const {
    if(resolved.ready) return resolved.recordVideoFile;
    return properties.recordVideoFile;
}
RecordVideo::CompressionLevel RecordVideo::getCompressionLevel() const {
    if(resolved.ready) return resolved.compression;
    return compressionFrom(properties.compressionLevel);
}
RecordVideo& RecordVideo::setRecordMetadataFile(const std::filesystem::path& recordFile) {
    properties.recordMetadataFile = recordFile.string();
    properties.recordMetadataFileSet = true;
    return *this;
}
RecordVideo& RecordVideo::setRecordVideoFile(const std::filesystem::path& recordFile) {
    properties.recordVideoFile = recordFile.string();
    properties.recordVideoFileSet = true;
    return *this;
}
RecordVideo& RecordVideo::setCompressionLevel(CompressionLevel compressionLevel) {
    properties.compressionLevel = static_cast<std::int32_t>(compressionLevel);
    return *this;
}
RecordVideo& RecordVideo::setFps(unsigned int fps) {
    properties.fps = static_cast<float>(fps);
    properties.fpsSet = true;
    return *this;
}
RecordVideo& RecordVideo::setStreamFps(float fps) {
    properties.fps = fps;
    properties.fpsSet = true;
    return *this;
}
RecordVideo& RecordVideo::setFormat(RecordFormat format) {
    properties.format = static_cast<std::int32_t>(format);
    return *this;
}
RecordVideo& RecordVideo::setRecordingName(const std::string& name) {
    properties.recordingName = name;
    properties.recordingNameSet = true;
    return *this;
}
RecordVideo& RecordVideo::setOutputDirectory(const std::filesystem::path& path) {
    properties.outputDirectory = path.string();
    properties.outputDirectorySet = true;
    return *this;
}
RecordVideo& RecordVideo::setStreamName(const std::string& name) {
    properties.streamName = name;
    properties.streamNameSet = true;
    return *this;
}
RecordVideo& RecordVideo::setWaitForTrigger(bool wait) {
    properties.waitForTrigger = wait;
    return *this;
}
RecordVideo& RecordVideo::setPreTriggerFrames(std::int32_t frames) {
    properties.preTriggerFrames = frames;
    return *this;
}
RecordVideo& RecordVideo::setMaxPreTriggerBytes(std::uint64_t bytes) {
    properties.maxPreTriggerBytes = bytes;
    return *this;
}
RecordVideo& RecordVideo::setTransferToHost(bool transfer) {
    properties.transferToHost = transfer;
    properties.transferToHostSet = true;
    return *this;
}
RecordVideo& RecordVideo::setKeepOnDevice(bool keep) {
    properties.keepOnDevice = keep;
    properties.keepOnDeviceSet = true;
    return *this;
}
RecordVideo& RecordVideo::setRunOnHost(bool runOnHost) {
    properties.runOnHostMode = runOnHost ? 1 : 2;
    return *this;
}
RecordFormat RecordVideo::getFormat() const {
    if(resolved.ready) return resolved.format;
    return properties.format == static_cast<std::int32_t>(RecordFormat::RAW_FRAMES) ? RecordFormat::RAW_FRAMES : RecordFormat::VIDEO;
}
std::string RecordVideo::getRecordingName() const {
    if(resolved.ready) return resolved.recordingName;
    return properties.recordingName;
}
std::filesystem::path RecordVideo::getOutputDirectory() const {
    if(resolved.ready) return resolved.outputDirectory;
    return properties.outputDirectory;
}
std::string RecordVideo::getStreamName() const {
    if(resolved.ready) return resolved.streamName;
    return properties.streamName;
}
float RecordVideo::getFps() const {
    if(resolved.ready) return resolved.fps;
    return properties.fps;
}
bool RecordVideo::getWaitForTrigger() const {
    if(resolved.ready) return resolved.waitForTrigger;
    return properties.waitForTrigger;
}
std::int32_t RecordVideo::getPreTriggerFrames() const {
    if(resolved.ready) return resolved.preTriggerFrames;
    return properties.preTriggerFrames;
}
std::uint64_t RecordVideo::getMaxPreTriggerBytes() const {
    if(resolved.ready) return resolved.maxPreTriggerBytes;
    return properties.maxPreTriggerBytes;
}
bool RecordVideo::getTransferToHost() const {
    if(resolved.ready) return resolved.transferToHost;
    return properties.transferToHost;
}
bool RecordVideo::getKeepOnDevice() const {
    if(resolved.ready) return resolved.keepOnDevice;
    return properties.keepOnDevice;
}
bool RecordVideo::isLegacyLayout() const {
    if(resolved.ready) return resolved.legacy;
    return properties.recordVideoFileSet || properties.recordMetadataFileSet;
}
std::optional<RecordStatus> RecordVideo::getStatus() const {
    std::lock_guard<std::mutex> lock(statusMutex);
    return lastStatus;
}

void RecordMetadataOnly::run() {
#ifdef DEPTHAI_ENABLE_PROTOBUF
    auto& logger = pimpl->logger;
    utility::ByteRecorder byteRecorder;

    DatatypeEnum streamType = DatatypeEnum::ADatatype;
    while(mainLoop()) {
        std::shared_ptr<dai::Buffer> msg = nullptr;
        {
            auto blockEvent = this->inputBlockEvent();
            msg = input.get<dai::Buffer>();
        }
        if(msg == nullptr) continue;
        if(streamType == DatatypeEnum::ADatatype) {
            if(std::dynamic_pointer_cast<ImgFrame>(msg) != nullptr) {
                streamType = DatatypeEnum::ImgFrame;
                byteRecorder.init<dai::proto::img_frame::ImgFrame>(recordFile.string(), compressionLevel, "img_frame");
            } else if(std::dynamic_pointer_cast<IMUData>(msg) != nullptr) {
                streamType = DatatypeEnum::IMUData;
                byteRecorder.init<dai::proto::imu_data::IMUData>(recordFile.string(), compressionLevel, "imu");
            } else if(std::dynamic_pointer_cast<EncodedFrame>(msg) != nullptr) {
                streamType = DatatypeEnum::EncodedFrame;
                byteRecorder.init<dai::proto::encoded_frame::EncodedFrame>(recordFile.string(), compressionLevel, "encoded_frame");
            } else if(std::dynamic_pointer_cast<PointCloudData>(msg) != nullptr) {
                streamType = DatatypeEnum::PointCloudData;
                byteRecorder.init<dai::proto::point_cloud_data::PointCloudData>(recordFile.string(), compressionLevel, "point_cloud");
            } else {
                throw std::runtime_error("RecordMetadataOnly node does not support this type of message");
            }
            if(logger) logger->trace("RecordMetadataOnly node detected stream type {}", (int)streamType);
        }
        auto serializable = std::dynamic_pointer_cast<ProtoSerializable>(msg);
        if(serializable == nullptr) {
            throw std::runtime_error("RecordMetadataOnly unsupported message type");
        }
        byteRecorder.write(serializable->serializeProto());
    }
#else
    throw std::runtime_error("RecordMetadataOnly node requires protobuf support");
#endif
}

std::filesystem::path RecordMetadataOnly::getRecordFile() const {
    return recordFile;
}
RecordMetadataOnly::CompressionLevel RecordMetadataOnly::getCompressionLevel() const {
    return compressionLevel;
}
RecordMetadataOnly& RecordMetadataOnly::setRecordFile(const std::filesystem::path& recordFile) {
    this->recordFile = recordFile;
    return *this;
}
RecordMetadataOnly& RecordMetadataOnly::setCompressionLevel(CompressionLevel compressionLevel) {
    this->compressionLevel = compressionLevel;
    return *this;
}

}  // namespace node
}  // namespace dai
