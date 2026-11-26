#include "Record.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/schemas/ImgFrame.pb.h"
#include "depthai/utility/ProtoSerializable.hpp"
#include "depthai/utility/RecordingStore.hpp"
#include "depthai/utility/VectorMemory.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/VideoEncoder.hpp"
#include "utilities/DmaHeapMemory.hpp"
#include "utilities/RecordingStoreDevice.hpp"
#include "utility/EncodedVideoRecorder.hpp"
#include "utility/RecordReplayImpl.hpp"

namespace dai {
namespace gate {
namespace {

using SteadyTime = std::chrono::steady_clock::time_point;

struct HeldFrame {
    std::shared_ptr<Buffer> message;
    SteadyTime received;
    std::uint64_t bytes = 0;
};

std::string resolveStream(const RecordVideoProperties& properties) {
    if(!properties.recordVideoFileSet && !properties.recordMetadataFileSet) {
        validateStreamName(properties.streamName);
        return properties.streamName;
    }
    auto stream = std::filesystem::path(properties.recordVideoFile).stem().string();
    if(stream.empty()) stream = std::filesystem::path(properties.recordMetadataFile).stem().string();
    if(stream.empty()) stream = "video";
    validateRecordingName(stream);
    return stream;
}

}  // namespace

struct RecordVideo::State {
    std::shared_ptr<RecordingStoreDevice> store = recordingStore();
    RecordVideo& node;
    std::shared_ptr<spdlog::async_logger> logger;
    const RecordVideoProperties config;
    const std::string stream;
    std::filesystem::path directory;
    bool reserved = false;
    bool recording = false;
    bool memoryChecked = false;
    bool bufferWarned = false;
    bool profileWarned = false;
    bool videoOpened = false;
    bool mp4CloseOk = true;
    DatatypeEnum kind = DatatypeEnum::ADatatype;
    RecordStatus latest;
    std::int64_t statusSequence = 0;
    std::int32_t maxFrames = -1;
    std::int32_t maxDurationMs = -1;
    std::int32_t maxSizeKb = -1;
    std::uint64_t segmentBytes = 0;
    SteadyTime segmentStart{};
    std::uint64_t bufferedBytes = 0;
    std::deque<HeldFrame> buffered;
    std::deque<HeldFrame> pending;
    std::unique_ptr<utility::EncodedVideoRecorder> video;
    std::unique_ptr<utility::ByteRecorder> metadata;
    unsigned int width = 0;
    unsigned int height = 0;
    EncodedFrame::Profile profile = EncodedFrame::Profile::AVC;
    std::vector<std::uint8_t> h264Sps;
    std::vector<std::uint8_t> h264Pps;
    // H.264 stays closed until an IDR. A frame, duration, or size limit stops
    // the segment and leaves the node idle until the next start.
    bool waitingForIdr = false;

    State(RecordVideo& self, const RecordVideoProperties& properties)
        : node(self), logger(self.pimpl->logger), config(properties), stream(resolveStream(properties)) {}

    ~State() {
        if(reserved) store->release(config.recordingName, stream);
    }

    void publish(RecordStatus::State value, const std::string& error = {}) {
        latest.state = value;
        latest.error = error;
        latest.path = directory.string();
        const auto now = std::chrono::steady_clock::now();
        latest.setTimestamp(now);
        latest.setTimestampDevice(now);
        latest.setSequenceNum(statusSequence++);
        auto event = node.outputBlockEvent();
        node.status.send(std::make_shared<RecordStatus>(latest));
    }

    void resetCounts() {
        latest.frameCount = 0;
        latest.firstSequenceNum = -1;
        latest.lastSequenceNum = -1;
        latest.skippedFrames = 0;
    }

    void closeFiles() {
        // Finish metadata first, including on a full disk.
        std::exception_ptr failure;
        if(metadata) {
            try {
                metadata->close();
            } catch(...) {
                failure = std::current_exception();
            }
            metadata.reset();
        }
        if(video) {
            if(profile == EncodedFrame::Profile::AVC) rememberH264ParameterSets();
            auto closing = std::move(video);
            try {
                closing->close();
            } catch(...) {
                mp4CloseOk = false;
                failure = std::current_exception();
            }
        }
        if(failure) std::rethrow_exception(failure);
    }

    void rememberH264ParameterSets() {
        if(!video) return;
        const auto sets = video->copyH264ParameterSets();
        if(!sets.sps.empty()) h264Sps = sets.sps;
        if(!sets.pps.empty()) h264Pps = sets.pps;
    }

    void requestUpstreamKeyframe() {
        bool found = false;
        const auto connectionMap = node.getParentPipeline().getConnectionMap();
        for(const auto& entry : connectionMap) {
            for(const auto& connection : entry.second) {
                if(connection.in != &node.input) continue;
                const auto producer = connection.outputNode.lock();
                const auto encoder = std::dynamic_pointer_cast<VideoEncoder>(producer);
                if(!encoder) continue;
                found = true;
                encoder->requestKeyframe();
            }
        }
        if(!found) logger->debug("Record has no VideoEncoder to request a segment keyframe from");
    }

    bool isAvc(const std::shared_ptr<Buffer>& frame) const {
        if(config.format != static_cast<std::int32_t>(RecordFormat::VIDEO)) return false;
        const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(frame);
        return encoded && encoded->getProfile() == EncodedFrame::Profile::AVC;
    }

    bool isIdr(const std::shared_ptr<Buffer>& frame) const {
        return isAvc(frame) && utility::EncodedVideoRecorder::containsIdr(frame->getData());
    }

    void rememberH264Bytes(span<const std::uint8_t> data) {
        const auto sets = utility::EncodedVideoRecorder::peekH264ParameterSets(data);
        if(!sets.sps.empty()) h264Sps = std::move(sets.sps);
        if(!sets.pps.empty()) h264Pps = std::move(sets.pps);
    }

    void dropUntilKeyframe(const HeldFrame& frame) {
        rememberH264Bytes(frame.message->getData());
        logger->debug("Record dropped frame {} until the segment keyframe", frame.message->getSequenceNum());
    }

    void dropUndecodablePretrigger() {
        if(config.format != static_cast<std::int32_t>(RecordFormat::VIDEO)) {
            waitingForIdr = false;
            return;
        }
        if(pending.empty() || !isAvc(pending.front().message)) {
            waitingForIdr = pending.empty();
            return;
        }
        std::deque<HeldFrame> kept;
        bool seenIdr = false;
        for(auto& held : pending) {
            if(!seenIdr && !isIdr(held.message)) {
                dropUntilKeyframe(held);
                continue;
            }
            seenIdr = true;
            kept.push_back(std::move(held));
        }
        pending = std::move(kept);
        waitingForIdr = !seenIdr;
    }

    bool sizeReached() const {
        if(maxSizeKb <= 0) return false;
        return segmentBytes >= static_cast<std::uint64_t>(maxSizeKb) * 1024ULL;
    }

    void begin(std::int32_t frames, std::int32_t duration, std::int32_t sizeKb, bool manualStart) {
        ++latest.segment;
        resetCounts();
        recording = true;
        maxFrames = frames;
        maxDurationMs = duration;
        maxSizeKb = sizeKb;
        segmentBytes = 0;
        segmentStart = std::chrono::steady_clock::now();
        videoOpened = false;
        mp4CloseOk = true;
        waitingForIdr = false;
        if(frames > 0) {
            while(buffered.size() > static_cast<std::size_t>(frames)) dropOldest();
        }
        if(sizeKb > 0) {
            const auto budget = static_cast<std::uint64_t>(sizeKb) * 1024ULL;
            while(buffered.size() > 1 && bufferedBytes > budget) dropOldest();
        }
        pending = std::move(buffered);
        buffered.clear();
        bufferedBytes = 0;
        dropUndecodablePretrigger();
        // A manual start can land mid-GOP. Ask for an IDR now. Pipeline start does not.
        if(manualStart && config.format == static_cast<std::int32_t>(RecordFormat::VIDEO)) requestUpstreamKeyframe();
        publish(RecordStatus::State::STARTED);
    }

    void end() {
        if(!recording) return;
        closeFiles();
        publish(RecordStatus::State::STOPPED);
        recording = false;
        while(!pending.empty()) {
            auto held = std::move(pending.front());
            pending.pop_front();
            idle(std::move(held));
        }
    }

    bool durationReached(SteadyTime time) const {
        return recording && maxDurationMs > 0 && time >= segmentStart + std::chrono::milliseconds(maxDurationMs);
    }

    void controls() {
        while(auto control = node.inputControl.tryGet<RecordControl>()) {
            if(control->command == RecordControl::Command::STOP) {
                if(recording) {
                    flushPending();
                    end();
                } else
                    logger->debug("Record ignored stop");
            } else if(control->command == RecordControl::Command::START) {
                if(recording)
                    logger->warn("Record start ignored");
                else if(control->maxFrames == 0 || control->maxDurationMs == 0 || control->maxSizeKb == 0)
                    logger->warn("Record limit of 0 ignored");
                else
                    begin(control->maxFrames, control->maxDurationMs, control->maxSizeKb, true);
            } else {
                throw std::runtime_error("Record invalid control command");
            }
        }
    }

    void accept(const std::shared_ptr<Buffer>& frame) {
        const auto image = std::dynamic_pointer_cast<ImgFrame>(frame);
        const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(frame);
        if(!image && !encoded) throw std::runtime_error("Record input accepts only ImgFrame and EncodedFrame");
        if(image && image->getType() == ImgFrame::Type::BITSTREAM) throw std::runtime_error("Record BITSTREAM ImgFrame is not supported");
        if(kind == DatatypeEnum::ADatatype) kind = frame->getDatatype();
        if(kind != frame->getDatatype()) throw std::runtime_error("Record message type changed");
        if(config.format == static_cast<std::int32_t>(RecordFormat::VIDEO)) {
            if(image) throw std::runtime_error("Record VIDEO on the device requires EncodedFrame: use RAW_FRAMES or put a VideoEncoder before the node");
            if(encoded->getProfile() == EncodedFrame::Profile::HEVC) throw std::runtime_error("Record H265 is not supported in VIDEO format");
            if(encoded->getProfile() != EncodedFrame::Profile::AVC && encoded->getProfile() != EncodedFrame::Profile::JPEG) {
                throw std::runtime_error("Record unsupported VIDEO profile");
            }
        } else if(encoded && !profileWarned) {
            if(encoded->getProfile() == EncodedFrame::Profile::AVC) logger->warn("Record H264 frames cannot be replayed");
            if(encoded->getProfile() == EncodedFrame::Profile::HEVC) logger->warn("Record H265 frames cannot be replayed");
            profileWarned = true;
        }
    }

    void noteH264ParameterSets(const std::shared_ptr<Buffer>& frame) {
        if(config.format != static_cast<std::int32_t>(RecordFormat::VIDEO)) return;
        const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(frame);
        if(!encoded || encoded->getProfile() != EncodedFrame::Profile::AVC) return;
        // The first IDR, which carries SPS and PPS, often arrives before the trigger.
        // Keep those sets so the next file's avcC is complete even if that IDR is not recorded.
        std::optional<DmaHeapMemory::CpuAccessGuard> access;
        if(auto memory = std::dynamic_pointer_cast<DmaHeapMemory>(frame->data)) access.emplace(memory->scopedCpuReadAccess());
        const auto sets = utility::EncodedVideoRecorder::peekH264ParameterSets(encoded->getData());
        if(!sets.sps.empty()) h264Sps = sets.sps;
        if(!sets.pps.empty()) h264Pps = sets.pps;
    }

    void idle(HeldFrame held) {
        noteH264ParameterSets(held.message);
        logger->debug("Record idle frame {}", held.message->getSequenceNum());
        if(config.preTriggerFrames == 0) return;
        if(!memoryChecked) {
            memoryChecked = true;
            const auto frames = static_cast<std::uint64_t>(config.preTriggerFrames);
            if(held.bytes > config.maxPreTriggerBytes / frames) {
                const auto need =
                    held.bytes > std::numeric_limits<std::uint64_t>::max() / frames ? "more than uint64 bytes" : std::to_string(frames * held.bytes);
                throw std::runtime_error("Record pre-trigger buffer " + need + " bytes exceeds " + std::to_string(config.maxPreTriggerBytes));
            }
        }
        while(buffered.size() >= static_cast<std::size_t>(config.preTriggerFrames)) dropOldest();
        const bool overBudget = held.bytes > config.maxPreTriggerBytes || bufferedBytes > config.maxPreTriggerBytes - held.bytes;
        if(overBudget && !bufferWarned) {
            bufferWarned = true;
            logger->warn("Record pre-trigger buffer full");
        }
        while(!buffered.empty() && (held.bytes > config.maxPreTriggerBytes || bufferedBytes > config.maxPreTriggerBytes - held.bytes)) dropOldest();
        if(held.bytes > config.maxPreTriggerBytes) return;
        // Return the producer's pool buffer immediately. Retaining camera/DMA buffers
        // for an idle pre-trigger history can exhaust the pool and stop the camera.
        held.message = copyForHistory(held.message);
        bufferedBytes += held.bytes;
        buffered.push_back(std::move(held));
    }

    std::shared_ptr<Buffer> copyForHistory(const std::shared_ptr<Buffer>& source) const {
        std::optional<DmaHeapMemory::CpuAccessGuard> access;
        if(auto memory = std::dynamic_pointer_cast<DmaHeapMemory>(source->data)) access.emplace(memory->scopedCpuReadAccess());
        std::shared_ptr<Buffer> copy;
        if(auto image = std::dynamic_pointer_cast<ImgFrame>(source))
            copy = std::make_shared<ImgFrame>(*image);
        else
            copy = std::make_shared<EncodedFrame>(*std::dynamic_pointer_cast<EncodedFrame>(source));
        const auto bytes = source->getData();
        copy->data = std::make_shared<VectorMemory>(std::vector<std::uint8_t>(bytes.begin(), bytes.end()));
        return copy;
    }

    void dropOldest() {
        bufferedBytes -= buffered.front().bytes;
        buffered.pop_front();
    }

    void writeRaw(const std::shared_ptr<Buffer>& frame) {
        const auto sequence = frame->getSequenceNum();
        if(sequence < 0) throw std::runtime_error("Record sequenceNum " + std::to_string(sequence) + " is below 0");
        std::ostringstream filename;
        filename << stream << '_' << std::setw(8) << std::setfill('0') << sequence << ".dai";
        const auto destination = directory / stream / filename.str();
        if(std::filesystem::exists(destination)) throw std::runtime_error("Record file already exists: " + filename.str());
        const auto temporary = std::filesystem::path(destination.string() + ".tmp");
        const auto serializable = std::dynamic_pointer_cast<ProtoSerializable>(frame);
        if(!serializable) throw std::runtime_error("Record frame cannot be saved as .dai");
        try {
            serializable->save(temporary);
            std::filesystem::rename(temporary, destination);
        } catch(...) {
            std::error_code error;
            std::filesystem::remove(temporary, error);
            throw;
        }
    }

    void openVideo(const std::shared_ptr<EncodedFrame>& frame) {
        width = frame->getWidth();
        height = frame->getHeight();
        profile = frame->getProfile();
        const auto suffix = latest.segment == 0 ? "" : "_s" + std::to_string(latest.segment);
        const auto videoPath = directory / (stream + suffix + ".mp4");
        const auto metadataPath = directory / (stream + suffix + ".mcap");
        store->claimFiles(config.recordingName, {videoPath.filename().string(), metadataPath.filename().string()});
        video = std::make_unique<utility::EncodedVideoRecorder>();
        const bool mjpeg = profile == EncodedFrame::Profile::JPEG;
        video->init(videoPath.string(), width, height, config.fps, mjpeg);
        // The node drops only the undecodable prefix. Later frames are written
        // until a limit or STOP closes the segment.
        if(!mjpeg && !h264Sps.empty() && !h264Pps.empty()) video->setH264ParameterSets(h264Sps, h264Pps);
        videoOpened = true;
        metadata = std::make_unique<utility::ByteRecorder>();
        const auto compression = static_cast<RecordConfig::CompressionLevel>(config.compressionLevel);
        metadata->init<proto::img_frame::ImgFrame>(metadataPath.string(), compression, "video");
    }

    bool writeVideo(const std::shared_ptr<Buffer>& frame) {
        const auto encoded = std::dynamic_pointer_cast<EncodedFrame>(frame);
        if(!encoded) throw std::runtime_error("Record VIDEO requires EncodedFrame");
        if(!video) openVideo(encoded);
        if(width != encoded->getWidth() || height != encoded->getHeight() || profile != encoded->getProfile()) {
            throw std::runtime_error("Record video profile or size changed");
        }
        const bool stored = video->write(encoded->getData());
        if(profile == EncodedFrame::Profile::AVC) rememberH264ParameterSets();
        if(!stored) return false;
        metadata->write(encoded->getImgFrameMeta().serializeProto(true));
        return true;
    }

    void countFrame(std::int64_t sequence) {
        if(latest.frameCount == 0) {
            latest.firstSequenceNum = sequence;
        } else if(sequence > latest.lastSequenceNum && static_cast<std::uint64_t>(sequence) - static_cast<std::uint64_t>(latest.lastSequenceNum) > 1) {
            // Unsigned subtraction also handles the full signed sequence-number range.
            const auto gap = static_cast<std::uint64_t>(sequence) - static_cast<std::uint64_t>(latest.lastSequenceNum) - 1;
            const auto room = static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max() - latest.skippedFrames);
            latest.skippedFrames += static_cast<std::int32_t>(std::min(gap, room));
            logger->warn("Record skipped {} frames ({} -> {})", gap, latest.lastSequenceNum, sequence);
        } else if(sequence <= latest.lastSequenceNum) {
            logger->warn("Record sequence went back ({} -> {})", latest.lastSequenceNum, sequence);
        }
        latest.lastSequenceNum = sequence;
        if(latest.frameCount < std::numeric_limits<std::int32_t>::max()) ++latest.frameCount;
    }

    void write(const HeldFrame& frame) {
        std::optional<DmaHeapMemory::CpuAccessGuard> access;
        if(auto memory = std::dynamic_pointer_cast<DmaHeapMemory>(frame.message->data)) access.emplace(memory->scopedCpuReadAccess());
        const bool avc = isAvc(frame.message);
        const bool idr = avc && utility::EncodedVideoRecorder::containsIdr(frame.message->getData());
        if(durationReached(frame.received)) {
            access.reset();
            auto remaining = std::move(pending);
            end();
            idle(frame);
            for(auto& held : remaining) idle(std::move(held));
            return;
        }
        if(waitingForIdr) {
            if(!avc) {
                waitingForIdr = false;
            } else if(!idr) {
                dropUntilKeyframe(frame);
                return;
            } else {
                waitingForIdr = false;
            }
        }
        if(latest.frameCount == 0) {
            const double rate = static_cast<double>(frame.bytes) * config.fps;
            if(rate > 100000000) logger->warn("Record expected write rate {:.0f} MB/s, frames can be skipped", rate / 1000000);
        }
        if(config.format == static_cast<std::int32_t>(RecordFormat::RAW_FRAMES))
            writeRaw(frame.message);
        else if(!writeVideo(frame.message)) {
            logger->debug("Record dropped frame {} until the segment keyframe", frame.message->getSequenceNum());
            return;
        }
        countFrame(frame.message->getSequenceNum());
        segmentBytes += frame.bytes;
        logger->debug("Record wrote frame {}", frame.message->getSequenceNum());
        if((maxFrames > 0 && latest.frameCount >= maxFrames) || sizeReached()) end();
    }

    void loop() {
        if(!config.waitForTrigger) begin(-1, -1, -1, false);
        while(node.mainLoop()) {
            try {
                controls();
                // Read one live frame for each write, including while flushing.
                // This keeps pending bounded by the pre-trigger backlog plus one frame.
                if(recording) {
                    if(auto frame = node.input.tryGet<Buffer>()) {
                        accept(frame);
                        pending.push_back(HeldFrame{frame, std::chrono::steady_clock::now(), frame->getData().size()});
                    }
                }
                if(!pending.empty()) {
                    auto frame = std::move(pending.front());
                    pending.pop_front();
                    write(frame);
                } else if(durationReached(std::chrono::steady_clock::now())) {
                    end();
                } else if(auto frame = node.input.tryGet<Buffer>()) {
                    accept(frame);
                    idle(HeldFrame{frame, std::chrono::steady_clock::now(), frame->getData().size()});
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            } catch(const MessageQueue::QueueException&) {
                // stop() closes the input queues; final status still goes out on status.
                break;
            }
        }
    }

    void flushPending() {
        while(recording && !pending.empty()) {
            auto frame = std::move(pending.front());
            pending.pop_front();
            write(frame);
        }
    }

    void fail(const std::string& reason) {
        const auto savedErrno = errno;
        std::error_code spaceError;
        const auto space = std::filesystem::space(directory, spaceError);
        const bool full = savedErrno == ENOSPC || (!spaceError && space.available == 0) || reason.find("ENOSPC") != std::string::npos;
        std::string error = full ? "Record storage is full: " + reason : reason;
        try {
            closeFiles();
        } catch(const std::exception& closeError) {
            error += std::string("; close failed: ") + closeError.what();
        }
        if(videoOpened) error += mp4CloseOk ? "; mp4 close succeeded" : "; mp4 close failed";
        if(!recording) resetCounts();
        publish(RecordStatus::State::ERROR, error);
        // The error path has closed every file; an abrupt process exit never reaches finish().
        logger->error("{}", error);
        try {
            store->finish(config.recordingName, stream);
        } catch(const std::exception& metadataError) {
            logger->error("Record could not mark {} complete: {}", config.recordingName, metadataError.what());
        }
        throw std::runtime_error(error);
    }
};

RecordVideo::RecordVideo() : RecordVideo(std::make_unique<Properties>()) {}
RecordVideo::RecordVideo(std::unique_ptr<Properties> props) : dai::node::RecordVideo(std::move(props)) {}
RecordVideo::~RecordVideo() = default;

void RecordVideo::buildStage1() {
    if(state) return;
    if(properties.format != 0 && properties.format != 1) throw std::runtime_error("Record invalid format");
    if(properties.format == 1 && (properties.recordVideoFileSet || properties.recordMetadataFileSet))
        throw std::runtime_error("Record RAW_FRAMES cannot use legacy setters");
    if(!std::isfinite(properties.fps) || properties.fps <= 0) throw std::runtime_error("Record invalid fps");
    if(properties.preTriggerFrames < 0) throw std::runtime_error("Record invalid preTriggerFrames");
    if(properties.maxPreTriggerBytes == 0) throw std::runtime_error("Record invalid maxPreTriggerBytes");
    validateRecordingName(properties.recordingName);
    auto prepared = std::make_unique<State>(*this, properties);
    const auto nodes = getParentPipeline().getAllNodes();
    if(nodes.empty()) throw std::runtime_error("Record has no parent pipeline");
    prepared->directory = prepared->store->reserve(properties.recordingName, prepared->stream, nodes.front().get());
    prepared->reserved = true;
    if(properties.format == 1) std::filesystem::create_directory(prepared->directory / prepared->stream);
    state = std::move(prepared);
}

void RecordVideo::run() {
    if(!state) throw std::runtime_error("RecordVideo started before the pipeline build");
    try {
        state->loop();
        state->flushPending();
        state->end();
        state->store->finish(state->config.recordingName, state->stream);
    } catch(const MessageQueue::QueueException&) {
        state->closeFiles();
        state->store->finish(state->config.recordingName, state->stream);
    } catch(const std::exception& error) {
        state->fail(error.what());
    }
}

void RecordVideo::releaseRecording() {
    if(state && state->reserved) {
        state->store->release(state->config.recordingName, state->stream);
        state->reserved = false;
    }
}

std::vector<std::uint8_t> RecordVideo::statusMetadata() const {
    if(!state || state->statusSequence == 0) return {};
    return utility::serialize(state->latest);
}

}  // namespace gate
}  // namespace dai
