#include "Replay.hpp"

#include <gst/video/gstvideometa.h>
#include <gst/video/video-info.h>
#include <gstreamer-1.0/gst/app/gstappsink.h>
#include <gstreamer-1.0/gst/gst.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/RGBDData.hpp"
#include "depthai/utility/Clock.hpp"
#include "depthai/utility/Compression.hpp"
#include "depthai/utility/ReplayPlayback.hpp"
#include "utilities/AcceleratorMemory.hpp"
#include "utilities/GstBufferMemory.hpp"
#include "utilities/RecordingStoreDevice.hpp"
#include "utilities/ReplayCacheDevice.hpp"
#include "utility/RecordReplayImpl.hpp"
#include "utility/ReplayArchive.hpp"
#include "utility/ReplayPacing.hpp"

namespace dai {
namespace gate {
namespace {

using SteadyTime = dai::Clock::time_point;

class RecordingReadLease {
   public:
    RecordingReadLease() = default;
    RecordingReadLease(const RecordingReadLease&) = delete;
    RecordingReadLease& operator=(const RecordingReadLease&) = delete;
    void acquire(const std::filesystem::path& path) {
        name = store->acquireReader(path);
    }
    ~RecordingReadLease() {
        store->releaseReader(name);
    }

   private:
    std::shared_ptr<RecordingStoreDevice> store = recordingStore();
    std::string name;
};

class ScratchDirLease {
   public:
    ScratchDirLease() = default;
    ScratchDirLease(const ScratchDirLease&) = delete;
    ScratchDirLease& operator=(const ScratchDirLease&) = delete;
    void hold(const std::string& directoryName) {
        replayCacheAcquireScratch(directoryName);
        held = directoryName;
    }
    ~ScratchDirLease() {
        if(!held.empty()) replayCacheReleaseScratch(held);
    }

   private:
    std::string held;
};

void holdScratch(std::vector<std::unique_ptr<ScratchDirLease>>& leases, const std::string& name) {
    auto lease = std::make_unique<ScratchDirLease>();
    lease->hold(name);
    leases.push_back(std::move(lease));
}

std::string unpackScratchName(const std::filesystem::path& archive, const std::string& hash) {
    return "unpack_" + (hash.empty() ? archive.filename().string() : hash);
}

std::string preScratchName(const std::string& hash) {
    return "pre_" + (hash.empty() ? std::string("local") : hash);
}

struct Chain {
    const char* demux;
    const char* parser;
    const char* decoder;
    bool fullRange;
    bool mmapOutput = false;
    const char* linearizer = nullptr;
};

std::string lowerCopy(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    return text;
}

std::string fileNameOf(const std::filesystem::path& path) {
    return lowerCopy(path.filename().string());
}

bool isArchiveName(const std::string& name) {
    return name.find(".tar") != std::string::npos || name.find(".tgz") != std::string::npos;
}

bool fpsIsSentinel(bool fpsSet, float fps) {
    return !fpsSet || fps <= 0.f || fps == 1000.f;
}

SteadyTime::duration nominalOf(bool fpsSet, float fps, SteadyTime first, SteadyTime last, int count) {
    if(!fpsIsSentinel(fpsSet, fps)) return replay::playbackPeriod(fps);
    if(count >= 2) {
        const auto mean = (last - first) / (count - 1);
        if(mean.count() > 0) return mean;
    }
    return replay::playbackPeriod(30.0);
}

void offsetRGBDChildren(const std::shared_ptr<Buffer>& buffer,
                        std::int64_t sequenceOffset,
                        SteadyTime::duration timestampOffset,
                        SteadyTime::duration deviceOffset) {
    auto rgbdData = std::dynamic_pointer_cast<RGBDData>(buffer);
    if(rgbdData == nullptr) return;
    const auto shift = [&](const std::optional<RGBDData::FrameVariant>& frame) {
        if(!frame.has_value()) return;
        std::visit(
            [&](const auto& typedFrame) {
                if(typedFrame == nullptr) return;
                typedFrame->setSequenceNum(typedFrame->getSequenceNum() + sequenceOffset);
                typedFrame->setTimestamp(typedFrame->getTimestamp() + timestampOffset);
                typedFrame->setTimestampDevice(typedFrame->getTimestampDevice() + deviceOffset);
                if(typedFrame->getTimestampSystem().has_value()) {
                    typedFrame->setTimestampSystem(*typedFrame->getTimestampSystem()
                                                   + std::chrono::duration_cast<std::chrono::system_clock::duration>(deviceOffset));
                }
            },
            frame.value());
    };
    shift(rgbdData->getRGBFrame());
    shift(rgbdData->getDepthFrame());
}

struct RecordedTiming {
    bool present = false;
    std::int64_t sequence = 0;
    SteadyTime timestamp{};
    SteadyTime timestampDevice{};
    std::optional<std::chrono::time_point<std::chrono::system_clock>> timestampSystem;
};

// One clock per pipeline, shared by ReplayVideo and ReplayMetadataOnly.
// The offset is elapsed / max span from the first run() entry, and it does not pass the pass being sent.
struct ReplayLoopSlot {
    std::mutex mutex;
    SteadyTime::duration maxSpan{};
    std::int64_t maxSeqStride = 0;
    int runners = 0;
    bool started = false;
    SteadyTime startedAt{};
};

std::shared_ptr<ReplayLoopSlot> replayLoopSlot(dai::Node& node) {
    static std::mutex mutex;
    static std::map<const void*, std::weak_ptr<ReplayLoopSlot>> slots;
    const void* key = node.getParentPipeline().impl();
    std::lock_guard<std::mutex> lock(mutex);
    for(auto sweep = slots.begin(); sweep != slots.end();) {
        if(sweep->second.expired())
            sweep = slots.erase(sweep);
        else
            ++sweep;
    }
    const auto it = slots.find(key);
    if(it != slots.end()) {
        if(auto existing = it->second.lock()) return existing;
        slots.erase(it);
    }
    auto created = std::make_shared<ReplayLoopSlot>();
    slots.emplace(key, created);
    return created;
}

SteadyTime::duration replaySpan(int count, SteadyTime deviceFirst, SteadyTime deviceLast, SteadyTime::duration nominal) {
    if(count < 2) return nominal;
    return (deviceLast - deviceFirst) + nominal;
}

std::int64_t replaySeqStride(std::int64_t seqFirst, std::int64_t seqLast) {
    return std::max<std::int64_t>(seqLast - seqFirst + 1, 1);
}

void noteReplayLoopEntered(ReplayLoopSlot& slot) {
    std::lock_guard<std::mutex> lock(slot.mutex);
    if(slot.runners == 0) {
        slot.startedAt = SteadyTime::clock::now();
        slot.started = true;
    }
    slot.runners++;
}

void noteReplayLoopLeft(ReplayLoopSlot& slot) {
    std::lock_guard<std::mutex> lock(slot.mutex);
    if(slot.runners > 0) slot.runners--;
}

class ReplayLoopSession {
   public:
    explicit ReplayLoopSession(ReplayLoopSlot& slot) : slot(slot) {
        noteReplayLoopEntered(slot);
    }
    ~ReplayLoopSession() {
        noteReplayLoopLeft(slot);
    }
    ReplayLoopSession(const ReplayLoopSession&) = delete;
    ReplayLoopSession& operator=(const ReplayLoopSession&) = delete;

   private:
    ReplayLoopSlot& slot;
};

// loopIndex, shared span, sequence stride.
std::int64_t playedPass(int sent, int count) {
    if(count <= 0 || sent <= 0) return 0;
    return static_cast<std::int64_t>(sent) / count;
}

std::tuple<std::int64_t, SteadyTime::duration, std::int64_t> updateReplayLoop(ReplayLoopSlot& slot,
                                                                              bool recorded,
                                                                              int count,
                                                                              std::int64_t seqFirst,
                                                                              std::int64_t seqLast,
                                                                              SteadyTime deviceFirst,
                                                                              SteadyTime deviceLast,
                                                                              SteadyTime::duration nominal,
                                                                              std::int64_t playedPass) {
    std::lock_guard<std::mutex> lock(slot.mutex);
    if(recorded) {
        const auto stride = replaySeqStride(seqFirst, seqLast);
        if(stride > slot.maxSeqStride) slot.maxSeqStride = stride;
        // count == 0 means the container length is not known yet. Do not seed max span with one nominal period.
        if(count > 0) {
            const auto span = replaySpan(count, deviceFirst, deviceLast, nominal);
            if(span > slot.maxSpan) slot.maxSpan = span;
        }
    }
    const auto seqStride = std::max<std::int64_t>(slot.maxSeqStride, 1);
    if(!slot.started || slot.maxSpan.count() == 0) return {0, slot.maxSpan, seqStride};
    const auto elapsed = SteadyTime::clock::now() - slot.startedAt;
    if(elapsed.count() <= 0) return {0, slot.maxSpan, seqStride};
    const auto elapsedIndex = static_cast<std::int64_t>(elapsed / slot.maxSpan);
    if(playedPass < 0) playedPass = 0;
    return {std::min(elapsedIndex, playedPass), slot.maxSpan, seqStride};
}

SteadyTime::duration pacedDelta(SteadyTime::duration raw, SteadyTime::duration nominal) {
    if(nominal.count() > 0) {
        if(raw > nominal && raw - nominal > nominal) return nominal;
    } else if(raw.count() > 0) {
        return nominal;
    }
    return raw;
}

void applyRecorded(const std::shared_ptr<Buffer>& buffer,
                   std::int64_t seqFirst,
                   SteadyTime deviceFirst,
                   const RecordedTiming& recorded,
                   std::int64_t loopIndex,
                   SteadyTime::duration maxSpan,
                   std::int64_t maxSeqStride) {
    const auto seqOffset = seqFirst + loopIndex * maxSeqStride;
    const auto tsOffset = deviceFirst + maxSpan * loopIndex;
    const auto sequence = recorded.sequence - seqFirst + seqOffset;
    const auto deviceOut = recorded.timestampDevice - deviceFirst + tsOffset;
    const auto amount = deviceOut - recorded.timestampDevice;
    const auto originalSequence = buffer->getSequenceNum();
    const auto originalTimestamp = buffer->getTimestamp();
    const auto originalDevice = buffer->getTimestampDevice();
    buffer->setSequenceNum(sequence);
    buffer->setTimestamp(recorded.timestamp + amount);
    buffer->setTimestampDevice(deviceOut);
    if(recorded.timestampSystem.has_value()) {
        buffer->setTimestampSystem(*recorded.timestampSystem + std::chrono::duration_cast<std::chrono::system_clock::duration>(amount));
    } else {
        buffer->setTimestampSystem(std::nullopt);
    }
    offsetRGBDChildren(
        buffer, buffer->getSequenceNum() - originalSequence, buffer->getTimestamp() - originalTimestamp, buffer->getTimestampDevice() - originalDevice);
}

void ensureGst() {
    static std::once_flag once;
    std::call_once(once, [] { gst_init(nullptr, nullptr); });
}

bool capsAreLinearNv12(GstCaps* caps) {
    if(caps == nullptr || gst_caps_get_size(caps) == 0 || gst_caps_is_any(caps)) return false;
    GstCapsFeatures* features = gst_caps_get_features(caps, 0);
    if(features != nullptr && !gst_caps_features_is_any(features)) {
        const guint count = gst_caps_features_get_size(features);
        for(guint index = 0; index < count; index++) {
            const char* name = gst_caps_features_get_nth(features, index);
            if(name != nullptr && std::strncmp(name, "memory:", 7) == 0 && std::strcmp(name, GST_CAPS_FEATURE_MEMORY_SYSTEM_MEMORY) != 0) return false;
        }
    }
    GstStructure* structure = gst_caps_get_structure(caps, 0);
    if(structure == nullptr) return false;
    const char* format = gst_structure_get_string(structure, "format");
    if(format == nullptr || std::strcmp(format, "NV12") != 0) return false;
    const char* compression = gst_structure_get_string(structure, "compression");
    if(compression != nullptr && compression[0] != '\0') {
        const auto text = lowerCopy(compression);
        if(text != "none" && text != "linear" && text != "uncompressed") return false;
    }
    return true;
}

bool setIntIfPresent(GstElement* element, const char* name, int value) {
    if(element == nullptr || g_object_class_find_property(G_OBJECT_GET_CLASS(element), name) == nullptr) return false;
    g_object_set(element, name, value, nullptr);
    return true;
}

class GstPlayer {
   public:
    ~GstPlayer() {
        close();
    }

    bool open(const std::filesystem::path& path, const Chain& chain) {
        close();
        ensureGst();
        fullRange = chain.fullRange;
        pipeline = gst_pipeline_new("depthai-replay");
        if(pipeline == nullptr) return false;
        GstElement* source = gst_element_factory_make("filesrc", "source");
        GstElement* demux = chain.demux == nullptr ? nullptr : gst_element_factory_make(chain.demux, "demux");
        GstElement* parser = chain.parser == nullptr ? nullptr : gst_element_factory_make(chain.parser, "parser");
        GstElement* decoder = gst_element_factory_make(chain.decoder, "decoder");
        GstElement* linearizer = chain.linearizer == nullptr ? nullptr : gst_element_factory_make(chain.linearizer, "linearizer");
        GstElement* convert = chain.mmapOutput ? nullptr : gst_element_factory_make("videoconvert", "convert");
        GstElement* filter = gst_element_factory_make("capsfilter", "filter");
        appsink = gst_element_factory_make("appsink", "sink");
        // The bin owns every successfully created element, including on a failed open.
        for(auto* element : {source, demux, parser, decoder, linearizer, convert, filter, appsink}) {
            if(element != nullptr) gst_bin_add(GST_BIN(pipeline), element);
        }
        if(source == nullptr || decoder == nullptr || filter == nullptr || appsink == nullptr || (chain.demux != nullptr && demux == nullptr)
           || (chain.parser != nullptr && parser == nullptr) || (chain.linearizer != nullptr && linearizer == nullptr)
           || (!chain.mmapOutput && convert == nullptr)) {
            close();
            return false;
        }
        if(chain.mmapOutput && !setIntIfPresent(decoder, "output-io-mode", 2)) {
            close();
            return false;
        }
        if(chain.mmapOutput) setIntIfPresent(decoder, "capture-io-mode", 2);
        g_object_set(source, "location", path.string().c_str(), nullptr);
        GstCaps* caps = gst_caps_from_string("video/x-raw,format=NV12");
        if(caps != nullptr && gst_caps_get_size(caps) > 0) {
            gst_caps_set_features(caps, 0, gst_caps_features_new(GST_CAPS_FEATURE_MEMORY_SYSTEM_MEMORY, nullptr));
        }
        g_object_set(filter, "caps", caps, nullptr);
        if(caps != nullptr) gst_caps_unref(caps);
        g_object_set(appsink, "emit-signals", FALSE, "sync", FALSE, "max-buffers", 1, "drop", FALSE, nullptr);
        GstElement* decodeHead = parser != nullptr ? parser : decoder;
        if(demux != nullptr) {
            if(!gst_element_link(source, demux)) {
                close();
                return false;
            }
            g_signal_connect(demux, "pad-added", G_CALLBACK(onPadAdded), decodeHead);
        } else if(!gst_element_link(source, decodeHead)) {
            close();
            return false;
        }
        if(parser != nullptr && !gst_element_link(parser, decoder)) {
            close();
            return false;
        }
        GstElement* previous = decoder;
        for(GstElement* next : {linearizer, convert, filter, appsink}) {
            if(next == nullptr) continue;
            if(!gst_element_link(previous, next)) {
                close();
                return false;
            }
            previous = next;
        }
        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            close();
            return false;
        }
        GstBus* bus = gst_element_get_bus(pipeline);
        GstMessage* message = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_ASYNC_DONE));
        const bool timedOut = message == nullptr;
        bool failed = message != nullptr && GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR;
        if(message != nullptr) gst_message_unref(message);
        GstSample* sample = nullptr;
        if(!failed && timedOut) {
            // A chain that posts neither an error nor a sample must not spend another pull timeout.
            sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 0);
            failed = sample == nullptr;
        } else if(!failed) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while(sample == nullptr && std::chrono::steady_clock::now() < deadline) {
                GstMessage* errorMessage = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
                if(errorMessage != nullptr) {
                    gst_message_unref(errorMessage);
                    failed = true;
                    break;
                }
                sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 200 * GST_MSECOND);
                if(sample == nullptr && gst_app_sink_is_eos(GST_APP_SINK(appsink))) break;
            }
        }
        gst_object_unref(bus);
        const bool linear = !failed && sample != nullptr && readGeometry(sample);
        if(sample != nullptr) gst_sample_unref(sample);
        if(!linear) {
            close();
            return false;
        }
        gst_element_set_state(pipeline, GST_STATE_NULL);
        if(gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            close();
            return false;
        }
        return true;
    }

    GstSample* pull(bool& eos) {
        std::lock_guard<std::mutex> lock(mutex);
        eos = false;
        if(appsink == nullptr) {
            eos = true;
            return nullptr;
        }
        if(gst_app_sink_is_eos(GST_APP_SINK(appsink))) {
            eos = true;
            return nullptr;
        }
        GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 200 * GST_MSECOND);
        if(sample == nullptr && gst_app_sink_is_eos(GST_APP_SINK(appsink))) eos = true;
        return sample;
    }

    void restart() {
        std::lock_guard<std::mutex> lock(mutex);
        if(pipeline == nullptr) return;
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex);
        if(pipeline != nullptr) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
        pipeline = nullptr;
        appsink = nullptr;
    }

    int width = 0;
    int height = 0;
    int stride = 0;
    int yOffset = 0;
    int uvOffset = 0;
    double fps = 0;
    bool fullRange = false;

    bool readGeometry(GstSample* sample) {
        GstCaps* caps = gst_sample_get_caps(sample);
        if(!capsAreLinearNv12(caps)) return false;
        GstStructure* structure = gst_caps_get_structure(caps, 0);
        if(!gst_structure_get_int(structure, "width", &width) || !gst_structure_get_int(structure, "height", &height)) return false;
        int numerator = 0;
        int denominator = 1;
        if(gst_structure_get_fraction(structure, "framerate", &numerator, &denominator) && denominator > 0 && numerator > 0) {
            fps = static_cast<double>(numerator) / static_cast<double>(denominator);
        }
        GstVideoInfo info{};
        if(!gst_video_info_from_caps(&info, caps) || GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_NV12) return false;
        stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
        const auto yPlane = GST_VIDEO_INFO_PLANE_OFFSET(&info, 0);
        const auto offset = GST_VIDEO_INFO_PLANE_OFFSET(&info, 1);
        if(yPlane > static_cast<std::size_t>(std::numeric_limits<int>::max()) || offset > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            return false;
        yOffset = static_cast<int>(yPlane);
        uvOffset = static_cast<int>(offset);
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        if(buffer == nullptr) return false;
        if(GstVideoMeta* meta = gst_buffer_get_video_meta(buffer)) {
            if(meta->n_planes < 2 || meta->offset[0] > static_cast<std::size_t>(std::numeric_limits<int>::max())
               || meta->offset[1] > static_cast<std::size_t>(std::numeric_limits<int>::max()))
                return false;
            stride = meta->stride[0];
            yOffset = static_cast<int>(meta->offset[0]);
            uvOffset = static_cast<int>(meta->offset[1]);
        }
        const auto alignedHeight = (static_cast<long long>(height) + 31LL) & ~31LL;
        const auto linearLimit = static_cast<long long>(stride) * alignedHeight + 4095LL;
        return width > 0 && height > 0 && stride > 0 && yOffset >= 0 && uvOffset >= yOffset
               && static_cast<std::size_t>(stride) >= ((static_cast<std::size_t>(width) + 1) / 2) * 2
               && static_cast<std::size_t>(uvOffset - yOffset) >= static_cast<std::size_t>(stride) * static_cast<std::size_t>(height)
               && static_cast<long long>(uvOffset - yOffset) <= linearLimit;
    }

    int estimatedCount() const {
        if(pipeline == nullptr || !(fps > 0.0)) return 0;
        gint64 duration = 0;
        if(!gst_element_query_duration(pipeline, GST_FORMAT_TIME, &duration) || duration <= 0) return 0;
        return static_cast<int>(std::llround(static_cast<double>(duration) / static_cast<double>(GST_SECOND) * fps));
    }

   private:
    static void onPadAdded(GstElement*, GstPad* pad, gpointer user) {
        auto* target = static_cast<GstElement*>(user);
        GstPad* sink = gst_element_get_static_pad(target, "sink");
        if(sink != nullptr && !gst_pad_is_linked(sink)) gst_pad_link(pad, sink);
        if(sink != nullptr) gst_object_unref(sink);
    }

    GstElement* pipeline = nullptr;
    GstElement* appsink = nullptr;
    std::mutex mutex;
};

// A null sample without EOS is a stall (try_pull waits 200 ms). Give up after about five seconds.
GstSample* pullSample(GstPlayer& player, bool& eos, const std::function<bool()>& alive) {
    eos = false;
    int stalls = 0;
    while(alive()) {
        GstSample* sample = player.pull(eos);
        if(sample != nullptr || eos) return sample;
        if(++stalls >= 25) throw std::runtime_error("ReplayVideo frame decode failed");
    }
    return nullptr;
}

std::uint8_t clampByte(int value) {
    if(value < 0) return 0;
    if(value > 255) return 255;
    return static_cast<std::uint8_t>(value);
}

void scalePlane(const std::uint8_t* src, int srcStride, int srcW, int srcH, std::uint8_t* dst, int dstStride, int dstW, int dstH, bool area) {
    for(int y = 0; y < dstH; y++) {
        for(int x = 0; x < dstW; x++) {
            if(area) {
                const int x0 = x * srcW / dstW;
                const int x1 = std::max(x0 + 1, (x + 1) * srcW / dstW);
                const int y0 = y * srcH / dstH;
                const int y1 = std::max(y0 + 1, (y + 1) * srcH / dstH);
                int sum = 0;
                int count = 0;
                for(int yy = y0; yy < y1 && yy < srcH; yy++) {
                    for(int xx = x0; xx < x1 && xx < srcW; xx++) {
                        sum += src[yy * srcStride + xx];
                        count++;
                    }
                }
                dst[y * dstStride + x] = static_cast<std::uint8_t>(count == 0 ? 0 : sum / count);
            } else {
                const float gx = (static_cast<float>(x) + 0.5f) * static_cast<float>(srcW) / static_cast<float>(dstW) - 0.5f;
                const float gy = (static_cast<float>(y) + 0.5f) * static_cast<float>(srcH) / static_cast<float>(dstH) - 0.5f;
                const int x0 = std::clamp(static_cast<int>(std::floor(gx)), 0, srcW - 1);
                const int y0 = std::clamp(static_cast<int>(std::floor(gy)), 0, srcH - 1);
                const int x1 = std::min(x0 + 1, srcW - 1);
                const int y1 = std::min(y0 + 1, srcH - 1);
                const float tx = std::clamp(gx - static_cast<float>(x0), 0.f, 1.f);
                const float ty = std::clamp(gy - static_cast<float>(y0), 0.f, 1.f);
                const float top = src[y0 * srcStride + x0] * (1.f - tx) + src[y0 * srcStride + x1] * tx;
                const float bottom = src[y1 * srcStride + x0] * (1.f - tx) + src[y1 * srcStride + x1] * tx;
                dst[y * dstStride + x] = clampByte(static_cast<int>(top * (1.f - ty) + bottom * ty + 0.5f));
            }
        }
    }
}

std::shared_ptr<AcceleratorMemory> memoryFrom(std::size_t size) {
    return AcceleratorMemory::create(size);
}

std::shared_ptr<ImgFrame> frameFromNv12(const std::uint8_t* nv12,
                                        int stride,
                                        int uvOffset,
                                        int srcW,
                                        int srcH,
                                        ImgFrame::Type outType,
                                        int outW,
                                        int outH,
                                        bool fullRange,
                                        const std::shared_ptr<Memory>& zeroCopy) {
    auto frame = std::make_shared<ImgFrame>();
    const bool sameSize = outW == srcW && outH == srcH;
    // Qualcomm NV12 stride is often wider than the width. Keep the decoder buffer and publish the real stride.
    if(outType == ImgFrame::Type::NV12 && sameSize && zeroCopy != nullptr) {
        frame->data = zeroCopy;
        frame->setSize(static_cast<unsigned>(outW), static_cast<unsigned>(outH));
        frame->setType(ImgFrame::Type::NV12);
        const int rowStride = stride > 0 ? stride : outW;
        frame->fb.stride = static_cast<unsigned>(rowStride);
        frame->fb.p1Offset = 0;
        frame->fb.p2Offset = static_cast<unsigned>(uvOffset > 0 ? uvOffset : rowStride * outH);
        frame->fb.p3Offset = frame->fb.p2Offset;
        frame->sourceFb = frame->fb;
        frame->transformation = ImgTransformation(static_cast<size_t>(outW), static_cast<size_t>(outH));
        return frame;
    }
    if(nv12 == nullptr) throw std::runtime_error("ReplayVideo frame decode failed");

    const bool shrink = static_cast<std::int64_t>(outW) * outH < static_cast<std::int64_t>(srcW) * srcH;
    std::vector<std::uint8_t> y(static_cast<std::size_t>(outW) * outH);
    std::vector<std::uint8_t> uv(static_cast<std::size_t>(outW / 2) * (outH / 2) * 2);
    if(sameSize) {
        for(int row = 0; row < srcH; row++) std::memcpy(y.data() + static_cast<std::size_t>(row) * outW, nv12 + static_cast<std::size_t>(row) * stride, outW);
        const std::uint8_t* srcUv = nv12 + uvOffset;
        for(int row = 0; row < srcH / 2; row++) {
            std::memcpy(uv.data() + static_cast<std::size_t>(row) * outW, srcUv + static_cast<std::size_t>(row) * stride, outW);
        }
    } else {
        scalePlane(nv12, stride, srcW, srcH, y.data(), outW, outW, outH, shrink);
        const int chromaW = srcW / 2 + srcW % 2;
        const int chromaH = srcH / 2 + srcH % 2;
        std::vector<std::uint8_t> u(static_cast<std::size_t>(chromaW) * chromaH);
        std::vector<std::uint8_t> v(u.size());
        const std::uint8_t* srcUv = nv12 + uvOffset;
        for(int row = 0; row < chromaH; row++) {
            for(int col = 0; col < chromaW; col++) {
                u[static_cast<std::size_t>(row) * chromaW + col] = srcUv[static_cast<std::size_t>(row) * stride + col * 2];
                v[static_cast<std::size_t>(row) * chromaW + col] = srcUv[static_cast<std::size_t>(row) * stride + col * 2 + 1];
            }
        }
        std::vector<std::uint8_t> uOut(static_cast<std::size_t>(outW / 2) * (outH / 2));
        std::vector<std::uint8_t> vOut(uOut.size());
        scalePlane(u.data(), chromaW, chromaW, chromaH, uOut.data(), outW / 2, outW / 2, outH / 2, shrink);
        scalePlane(v.data(), chromaW, chromaW, chromaH, vOut.data(), outW / 2, outW / 2, outH / 2, shrink);
        for(std::size_t i = 0; i < uOut.size(); i++) {
            uv[i * 2] = uOut[i];
            uv[i * 2 + 1] = vOut[i];
        }
    }

    frame->setSize(static_cast<unsigned>(outW), static_cast<unsigned>(outH));
    frame->transformation = ImgTransformation(static_cast<size_t>(outW), static_cast<size_t>(outH));
    if(outType == ImgFrame::Type::GRAY8) {
        frame->data = memoryFrom(y.size());
        std::memcpy(frame->data->getData().data(), y.data(), y.size());
        frame->setType(ImgFrame::Type::GRAY8);
        frame->fb.stride = outW;
    } else if(outType == ImgFrame::Type::NV12 || outType == ImgFrame::Type::YUV420p) {
        const auto bytes = y.size() + y.size() / 2;
        frame->data = memoryFrom(bytes);
        auto* dst = frame->data->getData().data();
        std::memcpy(dst, y.data(), y.size());
        if(outType == ImgFrame::Type::NV12) {
            std::memcpy(dst + y.size(), uv.data(), uv.size());
            frame->setType(ImgFrame::Type::NV12);
            frame->fb.p2Offset = static_cast<int>(y.size());
            frame->fb.p3Offset = frame->fb.p2Offset;
        } else {
            for(std::size_t i = 0; i < uv.size() / 2; i++) {
                dst[y.size() + i] = uv[i * 2];
                dst[y.size() + uv.size() / 2 + i] = uv[i * 2 + 1];
            }
            frame->setType(ImgFrame::Type::YUV420p);
            frame->fb.p2Offset = static_cast<int>(y.size());
            frame->fb.p3Offset = static_cast<int>(y.size() + uv.size() / 2);
        }
        frame->fb.stride = outW;
    } else {
        const auto bytes = static_cast<std::size_t>(outW) * outH * 3;
        frame->data = memoryFrom(bytes);
        auto* dst = frame->data->getData().data();
        for(int row = 0; row < outH; row++) {
            for(int col = 0; col < outW; col++) {
                const int luma = y[static_cast<std::size_t>(row) * outW + col];
                const int chromaIndex = (row / 2) * (outW / 2) + (col / 2);
                const int cb = uv[static_cast<std::size_t>(chromaIndex) * 2];
                const int cr = uv[static_cast<std::size_t>(chromaIndex) * 2 + 1];
                int red = 0;
                int green = 0;
                int blue = 0;
                if(fullRange) {
                    red = luma + ((cr - 128) * 359 >> 8);
                    green = luma - ((cb - 128) * 88 >> 8) - ((cr - 128) * 183 >> 8);
                    blue = luma + ((cb - 128) * 454 >> 8);
                } else {
                    const int c = luma - 16;
                    red = (298 * c + 409 * (cr - 128) + 128) >> 8;
                    green = (298 * c - 100 * (cb - 128) - 208 * (cr - 128) + 128) >> 8;
                    blue = (298 * c + 516 * (cb - 128) + 128) >> 8;
                }
                const auto offset = (static_cast<std::size_t>(row) * outW + col) * 3;
                if(outType == ImgFrame::Type::BGR888i) {
                    dst[offset] = clampByte(blue);
                    dst[offset + 1] = clampByte(green);
                    dst[offset + 2] = clampByte(red);
                } else {
                    dst[offset] = clampByte(red);
                    dst[offset + 1] = clampByte(green);
                    dst[offset + 2] = clampByte(blue);
                }
            }
        }
        frame->setType(outType);
        frame->fb.stride = outW * 3;
    }
    frame->fb.p1Offset = 0;
    frame->sourceFb = frame->fb;
    return frame;
}

void copyFrameIdentity(const std::shared_ptr<ImgFrame>& dst, const std::shared_ptr<ImgFrame>& src) {
    dst->sourceFb = src->sourceFb;
    dst->cam = src->cam;
    dst->instanceNum = src->instanceNum;
    dst->category = src->category;
    dst->transformation = src->transformation;
}

bool isReplayOutType(ImgFrame::Type type) {
    return type == ImgFrame::Type::NV12 || type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RGB888i
           || type == ImgFrame::Type::BGR888i;
}

std::string replayTypeName(ImgFrame::Type type) {
    switch(type) {
        case ImgFrame::Type::NV12:
            return "NV12";
        case ImgFrame::Type::YUV420p:
            return "YUV420p";
        case ImgFrame::Type::GRAY8:
            return "GRAY8";
        case ImgFrame::Type::RGB888i:
            return "RGB888i";
        case ImgFrame::Type::BGR888i:
            return "BGR888i";
        default:
            return "unsupported";
    }
}

void rgbToYuv(int red, int green, int blue, int& y, int& u, int& v, bool fullRange) {
    if(fullRange) {
        y = clampByte((299 * red + 587 * green + 114 * blue + 500) / 1000);
        u = clampByte(((-169 * red - 331 * green + 500 * blue + 500) / 1000) + 128);
        v = clampByte(((500 * red - 419 * green - 81 * blue + 500) / 1000) + 128);
        return;
    }
    y = clampByte(((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
    u = clampByte(((-38 * red - 74 * green + 112 * blue + 128) >> 8) + 128);
    v = clampByte(((112 * red - 94 * green - 18 * blue + 128) >> 8) + 128);
}

std::vector<std::uint8_t> nv12FromInterleaved(const std::uint8_t* src, int stride, int srcW, int srcH, bool rgb, bool fullRange) {
    const int tightStride = srcW + srcW % 2;
    const int chromaW = srcW / 2 + srcW % 2;
    const int chromaH = srcH / 2 + srcH % 2;
    std::vector<std::uint8_t> nv12(static_cast<std::size_t>(tightStride) * static_cast<std::size_t>(srcH + chromaH), 128);
    std::vector<int> uAcc(static_cast<std::size_t>(chromaW) * static_cast<std::size_t>(chromaH));
    std::vector<int> vAcc(uAcc.size());
    std::vector<int> count(uAcc.size());
    for(int row = 0; row < srcH; row++) {
        for(int col = 0; col < srcW; col++) {
            const auto* pixel = src + static_cast<std::size_t>(row) * static_cast<std::size_t>(stride) + static_cast<std::size_t>(col) * 3;
            const int red = rgb ? pixel[0] : pixel[2];
            const int green = pixel[1];
            const int blue = rgb ? pixel[2] : pixel[0];
            int y = 0;
            int u = 0;
            int v = 0;
            rgbToYuv(red, green, blue, y, u, v, fullRange);
            nv12[static_cast<std::size_t>(row) * static_cast<std::size_t>(tightStride) + static_cast<std::size_t>(col)] = static_cast<std::uint8_t>(y);
            const auto index = static_cast<std::size_t>(row / 2) * static_cast<std::size_t>(chromaW) + static_cast<std::size_t>(col / 2);
            uAcc[index] += u;
            vAcc[index] += v;
            count[index] += 1;
        }
    }
    auto* uv = nv12.data() + static_cast<std::size_t>(tightStride) * static_cast<std::size_t>(srcH);
    for(int row = 0; row < chromaH; row++) {
        for(int col = 0; col < chromaW; col++) {
            const auto index = static_cast<std::size_t>(row) * static_cast<std::size_t>(chromaW) + static_cast<std::size_t>(col);
            const int samples = std::max(count[index], 1);
            uv[static_cast<std::size_t>(row) * static_cast<std::size_t>(tightStride) + static_cast<std::size_t>(col) * 2] =
                static_cast<std::uint8_t>((uAcc[index] + samples / 2) / samples);
            uv[static_cast<std::size_t>(row) * static_cast<std::size_t>(tightStride) + static_cast<std::size_t>(col) * 2 + 1] =
                static_cast<std::uint8_t>((vAcc[index] + samples / 2) / samples);
        }
    }
    return nv12;
}

std::shared_ptr<ImgFrame> convertDaiFrame(const std::shared_ptr<ImgFrame>& src, ImgFrame::Type outType, int outW, int outH) {
    const int srcW = static_cast<int>(src->getWidth());
    const int srcH = static_cast<int>(src->getHeight());
    if(src->getType() == outType && srcW == outW && srcH == outH) return src;
    const auto bytes = src->getData();
    if(bytes.data() == nullptr || srcW <= 0 || srcH <= 0) throw std::runtime_error("ReplayVideo frame decode failed");
    const int rowStride = src->fb.stride > 0 ? static_cast<int>(src->fb.stride) : srcW;
    const auto p1 = static_cast<std::size_t>(src->fb.p1Offset);
    const bool explicitOffsets = src->fb.p1Offset != 0 || src->fb.p2Offset != 0 || src->fb.p3Offset != 0;
    auto finish = [&](std::shared_ptr<ImgFrame> frame) {
        copyFrameIdentity(frame, src);
        return frame;
    };
    auto fromNv12 = [&](const std::uint8_t* nv12, int stride, int uvOffset, bool fullRange) {
        return finish(frameFromNv12(nv12, stride, uvOffset, srcW, srcH, outType, outW, outH, fullRange, nullptr));
    };
    auto planeFits = [&](std::size_t offset, std::size_t stride, int rows, int cols) {
        const auto rowBytes = static_cast<std::size_t>(cols);
        if(rows <= 0 || stride < rowBytes || offset > bytes.size() || rowBytes > bytes.size() - offset) return false;
        return static_cast<std::size_t>(rows - 1) <= (bytes.size() - offset - rowBytes) / stride;
    };
    if(src->getType() == ImgFrame::Type::NV12) {
        const auto uv =
            explicitOffsets ? static_cast<std::size_t>(src->fb.p2Offset) : p1 + static_cast<std::size_t>(rowStride) * static_cast<std::size_t>(srcH);
        if(uv < p1 || !planeFits(p1, static_cast<std::size_t>(rowStride), srcH, srcW)) {
            throw std::runtime_error("ReplayVideo frame payload is too small or has an invalid plane layout");
        }
        const int chromaH = srcH / 2 + srcH % 2;
        if(!planeFits(uv, static_cast<std::size_t>(rowStride), chromaH, srcW + srcW % 2)) {
            throw std::runtime_error("ReplayVideo frame payload is too small or has an invalid plane layout");
        }
        return fromNv12(bytes.data() + p1, rowStride, static_cast<int>(uv - p1), true);
    }
    if(src->getType() == ImgFrame::Type::GRAY8) {
        if(!planeFits(p1, static_cast<std::size_t>(rowStride), srcH, srcW)) {
            throw std::runtime_error("ReplayVideo frame payload is too small or has an invalid plane layout");
        }
        const int chromaH = srcH / 2 + srcH % 2;
        std::vector<std::uint8_t> nv12(static_cast<std::size_t>(rowStride) * static_cast<std::size_t>(srcH + chromaH), 128);
        for(int row = 0; row < srcH; row++) {
            std::memcpy(nv12.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(rowStride),
                        bytes.data() + p1 + static_cast<std::size_t>(row) * static_cast<std::size_t>(rowStride),
                        static_cast<std::size_t>(srcW));
        }
        return fromNv12(nv12.data(), rowStride, rowStride * srcH, true);
    }
    if(src->getType() == ImgFrame::Type::YUV420p) {
        const int chromaW = srcW / 2 + srcW % 2;
        const int chromaH = srcH / 2 + srcH % 2;
        const int chromaStride = std::max(1, (rowStride + 1) / 2);
        const auto p2 =
            explicitOffsets ? static_cast<std::size_t>(src->fb.p2Offset) : p1 + static_cast<std::size_t>(rowStride) * static_cast<std::size_t>(srcH);
        const auto p3 =
            explicitOffsets ? static_cast<std::size_t>(src->fb.p3Offset) : p2 + static_cast<std::size_t>(chromaStride) * static_cast<std::size_t>(chromaH);
        if(!planeFits(p1, static_cast<std::size_t>(rowStride), srcH, srcW) || !planeFits(p2, static_cast<std::size_t>(chromaStride), chromaH, chromaW)
           || !planeFits(p3, static_cast<std::size_t>(chromaStride), chromaH, chromaW)) {
            throw std::runtime_error("ReplayVideo frame payload is too small or has an invalid plane layout");
        }
        const int tightStride = srcW + srcW % 2;
        std::vector<std::uint8_t> nv12(static_cast<std::size_t>(tightStride) * static_cast<std::size_t>(srcH + chromaH));
        for(int row = 0; row < srcH; row++) {
            std::memcpy(nv12.data() + static_cast<std::size_t>(row) * static_cast<std::size_t>(tightStride),
                        bytes.data() + p1 + static_cast<std::size_t>(row) * static_cast<std::size_t>(rowStride),
                        static_cast<std::size_t>(srcW));
        }
        auto* uv = nv12.data() + static_cast<std::size_t>(tightStride) * static_cast<std::size_t>(srcH);
        for(int row = 0; row < chromaH; row++) {
            for(int col = 0; col < chromaW; col++) {
                uv[static_cast<std::size_t>(row) * static_cast<std::size_t>(tightStride) + static_cast<std::size_t>(col) * 2] =
                    bytes[p2 + static_cast<std::size_t>(row) * static_cast<std::size_t>(chromaStride) + static_cast<std::size_t>(col)];
                uv[static_cast<std::size_t>(row) * static_cast<std::size_t>(tightStride) + static_cast<std::size_t>(col) * 2 + 1] =
                    bytes[p3 + static_cast<std::size_t>(row) * static_cast<std::size_t>(chromaStride) + static_cast<std::size_t>(col)];
            }
        }
        return fromNv12(nv12.data(), tightStride, tightStride * srcH, true);
    }
    if(src->getType() == ImgFrame::Type::RGB888i || src->getType() == ImgFrame::Type::BGR888i) {
        const int pixelStride = src->fb.stride > 0 ? static_cast<int>(src->fb.stride) : srcW * 3;
        if(!planeFits(p1, static_cast<std::size_t>(pixelStride), srcH, srcW * 3)) {
            throw std::runtime_error("ReplayVideo frame payload is too small or has an invalid plane layout");
        }
        const auto nv12 = nv12FromInterleaved(bytes.data() + p1, pixelStride, srcW, srcH, src->getType() == ImgFrame::Type::RGB888i, false);
        const int tightStride = srcW + srcW % 2;
        return fromNv12(nv12.data(), tightStride, tightStride * srcH, false);
    }
    throw std::runtime_error(std::string("ReplayVideo output type is not supported: ") + replayTypeName(src->getType()));
}

using SamplePtr = std::unique_ptr<GstSample, decltype(&gst_sample_unref)>;

class SampleMapping {
   public:
    explicit SampleMapping(GstSample* sample) : buffer(gst_sample_get_buffer(sample)) {
        if(buffer == nullptr || !gst_buffer_map(buffer, &map, GST_MAP_READ)) throw std::runtime_error("ReplayVideo frame decode failed");
    }
    ~SampleMapping() {
        gst_buffer_unmap(buffer, &map);
    }
    SampleMapping(const SampleMapping&) = delete;
    SampleMapping& operator=(const SampleMapping&) = delete;

    GstMapInfo map{};

   private:
    GstBuffer* buffer = nullptr;
};

std::shared_ptr<ImgFrame> frameFromSample(GstPlayer& player, GstSample* sample, ImgFrame::Type type, int width, int height) {
    if(!player.readGeometry(sample)) throw std::runtime_error("ReplayVideo frame decode failed");
    auto* buffer = gst_sample_get_buffer(sample);
    const auto required =
        static_cast<std::size_t>(player.uvOffset) + static_cast<std::size_t>(player.stride) * ((static_cast<std::size_t>(player.height) + 1) / 2);
    if(player.yOffset < 0 || player.uvOffset < player.yOffset || gst_buffer_get_size(buffer) < required) {
        throw std::runtime_error("ReplayVideo decoded frame is too small");
    }
    if(type == ImgFrame::Type::NV12 && width == player.width && height == player.height && player.yOffset == 0) {
        return frameFromNv12(nullptr,
                             player.stride,
                             player.uvOffset,
                             player.width,
                             player.height,
                             type,
                             width,
                             height,
                             player.fullRange,
                             std::make_shared<GstBufferMemory>(buffer));
    }
    const SampleMapping mapping(sample);
    if(mapping.map.size < required || static_cast<std::size_t>(player.yOffset) > mapping.map.size) {
        throw std::runtime_error("ReplayVideo decoded frame is too small");
    }
    return frameFromNv12(mapping.map.data + player.yOffset,
                         player.stride,
                         player.uvOffset - player.yOffset,
                         player.width,
                         player.height,
                         type,
                         width,
                         height,
                         player.fullRange,
                         nullptr);
}

void saveNv12Sample(GstPlayer& player, GstSample* sample, const std::filesystem::path& path) {
    if(!player.readGeometry(sample)) throw std::runtime_error("ReplayVideo frame decode failed");
    const SampleMapping mapping(sample);
    const auto required =
        static_cast<std::size_t>(player.uvOffset) + static_cast<std::size_t>(player.stride) * ((static_cast<std::size_t>(player.height) + 1) / 2);
    if(player.yOffset < 0 || player.uvOffset < player.yOffset || mapping.map.size < required || static_cast<std::size_t>(player.yOffset) > mapping.map.size) {
        throw std::runtime_error("ReplayVideo decoded frame is too small");
    }
    std::ofstream output;
    output.exceptions(std::ios::failbit | std::ios::badbit);
    output.open(path, std::ios::binary | std::ios::trunc);
    const auto* y = mapping.map.data + player.yOffset;
    for(int row = 0; row < player.height; ++row) {
        output.write(reinterpret_cast<const char*>(y + static_cast<std::size_t>(row) * player.stride), player.width);
        if(player.width % 2 != 0) output.put(0);
    }
    const auto* uv = mapping.map.data + player.uvOffset;
    for(int row = 0; row < player.height / 2 + player.height % 2; ++row) {
        output.write(reinterpret_cast<const char*>(uv + static_cast<std::size_t>(row) * player.stride), player.width + player.width % 2);
    }
    output.close();
}

std::int64_t sequenceOfFrameFile(const std::filesystem::path& path) {
    const auto stem = path.stem().string();
    const auto split = stem.rfind('_');
    if(split == std::string::npos || split == 0) return -1;
    const auto number = stem.substr(split + 1);
    if(number.empty() || !std::all_of(number.begin(), number.end(), [](unsigned char c) { return c >= '0' && c <= '9'; })) return -1;
    try {
        return std::stoll(number);
    } catch(const std::exception&) {
        return -1;
    }
}

std::vector<std::filesystem::path> listFrameFiles(const std::filesystem::path& directory) {
    std::vector<std::pair<std::int64_t, std::filesystem::path>> frames;
    std::set<std::string> streams;
    std::string stream;
    std::string extension;
    for(const auto& entry : std::filesystem::directory_iterator(directory)) {
        if(!entry.is_regular_file()) continue;
        const auto name = entry.path().filename().string();
        if(name.empty() || name.front() == '.') continue;
        const auto sequence = sequenceOfFrameFile(entry.path());
        if(sequence < 0) continue;
        auto ext = lowerCopy(entry.path().extension().string());
        if(ext != ".dai" && ext != ".jpg" && ext != ".jpeg" && ext != ".png") continue;
        if(ext == ".jpeg") ext = ".jpg";
        const auto stem = entry.path().stem().string();
        const auto prefix = stem.substr(0, stem.rfind('_'));
        if(stream.empty()) stream = prefix;
        streams.insert(prefix);
        if(extension.empty()) extension = ext;
        if(extension != ext) throw std::runtime_error("ReplayVideo directory has more than one extension");
        frames.emplace_back(sequence, entry.path());
    }
    if(frames.empty()) throw std::runtime_error("ReplayVideo source is empty: " + directory.string());
    if(streams.size() > 1) {
        std::string names;
        for(const auto& prefix : streams) names += (names.empty() ? "" : ", ") + prefix;
        throw std::runtime_error("ReplayVideo directory has more than one stream: " + names);
    }
    std::sort(frames.begin(), frames.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    for(std::size_t index = 1; index < frames.size(); index++) {
        if(frames[index].first == frames[index - 1].first) {
            throw std::runtime_error("ReplayVideo duplicate frame number " + std::to_string(frames[index].first) + " in " + directory.string());
        }
    }
    if(extension == ".dai") {
        for(const auto& root : {directory, directory.parent_path()}) {
            const auto marker = root / (".dai_closed_" + stream);
            if(!std::filesystem::exists(marker)) continue;
            std::ifstream input(marker);
            std::string text;
            std::getline(input, text);
            while(!text.empty() && (text.back() == '\r' || text.back() == ' ')) text.pop_back();
            if(text == "open") throw std::runtime_error("ReplayVideo recording is still open: " + marker.string());
        }
    }
    std::vector<std::filesystem::path> paths;
    paths.reserve(frames.size());
    for(const auto& frame : frames) paths.push_back(frame.second);
    return paths;
}

bool nameEnds(const std::string& name, const char* suffix) {
    const auto length = std::strlen(suffix);
    return name.size() >= length && name.compare(name.size() - length, length, suffix) == 0;
}

bool headerHas(const char* header, std::streamsize got, const char* token) {
    const auto length = std::strlen(token);
    if(got < static_cast<std::streamsize>(length)) return false;
    for(std::streamsize index = 0; index + static_cast<std::streamsize>(length) <= got; index++) {
        if(std::memcmp(header + index, token, length) == 0) return true;
    }
    return false;
}

void requireReadablePayload(const std::filesystem::path& path, const std::string& name) {
    if(std::filesystem::is_directory(path)) return;
    std::error_code error;
    const auto size = std::filesystem::file_size(path, error);
    if(error || size == 0) throw std::runtime_error("ReplayVideo source is unreadable: " + path.string());
    std::ifstream input(path, std::ios::binary);
    char header[64] = {};
    input.read(header, sizeof(header));
    const auto got = input.gcount();
    if(got <= 0) throw std::runtime_error("ReplayVideo source is unreadable: " + path.string());
    const bool mp4 = nameEnds(name, ".mp4") || nameEnds(name, ".mov");
    const bool avi = nameEnds(name, ".avi");
    const bool jpeg = nameEnds(name, ".jpg") || nameEnds(name, ".jpeg");
    const bool png = nameEnds(name, ".png");
    if(mp4 && !headerHas(header, got, "ftyp")) throw std::runtime_error("ReplayVideo source is unreadable: " + path.string());
    if(avi && (!headerHas(header, got, "RIFF") || !headerHas(header, got, "AVI"))) {
        throw std::runtime_error("ReplayVideo source is unreadable: " + path.string());
    }
    if(jpeg && (got < 2 || static_cast<unsigned char>(header[0]) != 0xff || static_cast<unsigned char>(header[1]) != 0xd8)) {
        throw std::runtime_error("ReplayVideo source is unreadable: " + path.string());
    }
    static const unsigned char pngSig[8] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
    if(png && (got < 8 || std::memcmp(header, pngSig, 8) != 0)) throw std::runtime_error("ReplayVideo source is unreadable: " + path.string());
}

std::filesystem::path extractArchive(const std::filesystem::path& archive, const std::string& hash) {
    const auto name = unpackScratchName(archive, hash);
    const auto root = std::filesystem::path("/data/depthai/replay_cache") / name;
    const auto marker = root / ".extracted-v2";
    if(std::filesystem::exists(marker)) return root;
    if(replayCacheScratchHolders(name) > 1) throw std::runtime_error("Replay cache scratch is in use: " + name);
    // Replace partial extractions and the older extractor's cache entries.
    std::filesystem::remove_all(root);
    try {
        const auto listing = replay::extractReplayArchive(archive, root);
        if(listing.entryBytes == 0) throw std::runtime_error("ReplayVideo archive has no source");
        const auto temporary = root / ".extracted-v2.tmp";
        std::ofstream output;
        output.exceptions(std::ios::failbit | std::ios::badbit);
        output.open(temporary);
        output << "complete";
        output.close();
        std::filesystem::rename(temporary, marker);
    } catch(...) {
        std::error_code error;
        if(replayCacheScratchHolders(name) <= 1) std::filesystem::remove_all(root, error);
        throw;
    }
    return root;
}

std::filesystem::path findNamed(const std::filesystem::path& root, const std::string& entry, bool metadataOnly) {
    const auto direct = root / entry;
    if(!metadataOnly && std::filesystem::is_directory(direct)) return direct;
    if(metadataOnly) {
        const auto mcap = root / (entry + ".mcap");
        if(std::filesystem::is_regular_file(mcap)) return mcap;
    } else {
        for(const auto* ext : {".mp4", ".avi", ".mcap"}) {
            const auto path = root / (entry + ext);
            if(std::filesystem::is_regular_file(path)) return path;
        }
    }
    throw std::runtime_error(std::string(metadataOnly ? "ReplayMetadataOnly" : "ReplayVideo") + " archive entry is missing: " + entry);
}

}  // namespace

struct ReplayVideo::State {
    std::unique_ptr<RecordingReadLease> recording;
    std::filesystem::path media;
    std::string hash;
    int srcW = 0;
    int srcH = 0;
    int outW = 0;
    int outH = 0;
    int predecodedStride = 0;
    int predecodedUvOffset = 0;
    int count = 0;
    ImgFrame::Type outType = ImgFrame::Type::YUV420p;
    bool passthrough = false;
    bool fullRange = false;
    bool loop = true;
    bool stopAtEnd = true;
    bool fpsSet = false;
    float fps = 0.f;
    bool recorded = false;
    bool dai = false;
    bool compressedImages = false;
    std::atomic<bool> stopping{false};
    std::int64_t seqFirst = 0;
    std::int64_t seqLast = 0;
    SteadyTime deviceFirst{};
    SteadyTime deviceLast{};
    SteadyTime::duration nominal{};
    std::shared_ptr<ReplayLoopSlot> loopSlot;
    std::vector<std::shared_ptr<ImgFrame>> metadata;
    std::vector<std::filesystem::path> frames;
    std::unique_ptr<GstPlayer> player;
    std::vector<std::unique_ptr<ScratchDirLease>> scratchLeases;
};

ReplayVideo::ReplayVideo() : ReplayVideo(std::make_unique<Properties>()) {}

ReplayVideo::ReplayVideo(std::unique_ptr<Properties> props) : ::dai::node::ReplayVideo(std::move(props)) {}

ReplayVideo::~ReplayVideo() = default;

struct ReplayMetadataOnly::State {
    std::unique_ptr<RecordingReadLease> recording;
    std::filesystem::path media;
    std::string hash;
    int count = 0;
    std::int32_t datatype = 0;
    bool loop = true;
    bool stopAtEnd = true;
    bool fpsSet = false;
    float fps = 0.f;
    std::int64_t seqFirst = 0;
    std::int64_t seqLast = 0;
    SteadyTime deviceFirst{};
    SteadyTime deviceLast{};
    SteadyTime::duration nominal{};
    std::shared_ptr<ReplayLoopSlot> loopSlot;
    std::vector<std::unique_ptr<ScratchDirLease>> scratchLeases;
};

ReplayMetadataOnly::ReplayMetadataOnly() : ReplayMetadataOnly(std::make_unique<Properties>()) {}

ReplayMetadataOnly::ReplayMetadataOnly(std::unique_ptr<Properties> props) : ::dai::node::ReplayMetadataOnly(std::move(props)) {}

ReplayMetadataOnly::~ReplayMetadataOnly() = default;

namespace {

std::unique_ptr<GstPlayer> openFirst(const std::filesystem::path& path, const std::vector<Chain>& chains) {
    for(const auto& chain : chains) {
        auto player = std::make_unique<GstPlayer>();
        if(player->open(path, chain)) return player;
    }
    throw std::runtime_error("ReplayVideo missing decoder support for " + path.string());
}

std::vector<Chain> chainsFor(const std::string& name) {
    if(name.size() >= 4 && (name.rfind(".mp4") != std::string::npos || name.rfind(".mov") != std::string::npos)) {
        return {{"qtdemux", "h264parse", "avdec_h264", false},
                {"qtdemux", "h265parse", "avdec_h265", false},
                {"qtdemux", "h264parse", "v4l2h264dec", false, true, nullptr},
                {"qtdemux", "h265parse", "v4l2h265dec", false, true, nullptr},
                {"qtdemux", "h264parse", "qtic2vdec", false, false, "qtivtransform"},
                {"qtdemux", "h265parse", "qtic2vdec", false, false, "qtivtransform"},
                {"qtdemux", nullptr, "avdec_mjpeg", true}};
    }
    if(name.size() >= 4 && name.rfind(".avi") != std::string::npos) {
        return {{"avidemux", nullptr, "avdec_ffv1", false}, {"avidemux", nullptr, "avdec_mjpeg", true}};
    }
    if(name.rfind(".jpg") != std::string::npos || name.rfind(".jpeg") != std::string::npos) return {{nullptr, nullptr, "jpegdec", true}};
    if(name.rfind(".png") != std::string::npos) return {{nullptr, nullptr, "pngdec", false}};
    return {};
}

struct DecodedImage {
    std::unique_ptr<GstPlayer> player;
    SamplePtr sample{nullptr, gst_sample_unref};
};

DecodedImage decodeImage(const std::filesystem::path& path, const std::function<bool()>& alive) {
    DecodedImage decoded;
    decoded.player = openFirst(path, chainsFor(fileNameOf(path)));
    bool eos = false;
    decoded.sample.reset(pullSample(*decoded.player, eos, alive));
    if(!decoded.sample && !alive()) return decoded;
    if(!decoded.sample || !decoded.player->readGeometry(decoded.sample.get())) throw std::runtime_error("ReplayVideo image decode failed: " + path.string());
    return decoded;
}

}  // namespace

void readMetadata(const std::filesystem::path& path, ReplayVideo::State& state, int frameCount) {
    utility::BytePlayer player;
    const auto schema = player.init(path.string());
    const auto datatype = replay::datatypeFromSchema(schema);
    if(static_cast<DatatypeEnum>(datatype) != DatatypeEnum::ImgFrame) {
        throw std::runtime_error("ReplayVideo metadata type is not ImgFrame: " + schema);
    }
    bool first = true;
    std::int64_t previousSeq = 0;
    SteadyTime previousDevice{};
    while(auto buffer = replay::nextRecordedBuffer(player, datatype)) {
        auto frame = std::dynamic_pointer_cast<ImgFrame>(buffer);
        if(frame == nullptr) throw std::runtime_error("ReplayVideo metadata type is not ImgFrame: " + schema);
        if(first) {
            state.seqFirst = frame->getSequenceNum();
            state.deviceFirst = frame->getTimestampDevice();
            first = false;
        } else {
            if(frame->getSequenceNum() <= previousSeq) {
                throw std::runtime_error("ReplayVideo sequence numbers do not increase: " + std::to_string(previousSeq) + " then "
                                         + std::to_string(frame->getSequenceNum()));
            }
            if(frame->getTimestampDevice() < previousDevice) throw std::runtime_error("ReplayVideo timestamps decrease");
        }
        previousSeq = frame->getSequenceNum();
        previousDevice = frame->getTimestampDevice();
        state.metadata.push_back(std::move(frame));
    }
    if(state.metadata.empty()) throw std::runtime_error("ReplayVideo metadata file is empty: " + path.string());
    if(frameCount > 0 && static_cast<int>(state.metadata.size()) != frameCount) {
        throw std::runtime_error("ReplayVideo metadata count " + std::to_string(state.metadata.size()) + " does not match frame count "
                                 + std::to_string(frameCount));
    }
    const auto& front = state.metadata.front();
    if(state.srcW > 0 && (static_cast<int>(front->getWidth()) != state.srcW || static_cast<int>(front->getHeight()) != state.srcH)) {
        throw std::runtime_error("ReplayVideo metadata frame size does not match: message 0 is " + std::to_string(front->getWidth()) + "x"
                                 + std::to_string(front->getHeight()));
    }
    state.seqLast = previousSeq;
    state.deviceLast = previousDevice;
    state.recorded = true;
    // An unknown container duration must not be replaced by the metadata length. Playback discovers it at EOS.
    if(frameCount > 0) state.count = frameCount;
}

namespace {

std::shared_ptr<ImgFrame> loadDaiFrame(const std::filesystem::path& path) {
#ifndef DEPTHAI_ENABLE_PROTOBUF
    (void)path;
    throw std::runtime_error("ReplayVideo node requires protobuf support");
#else
    auto frame = std::make_shared<ImgFrame>();
    frame->load(path);
    return frame;
#endif
}

}  // namespace

void requireFrameCount(const ReplayVideo::State& state, int frameCount) {
    if(!state.recorded || frameCount <= 0) return;
    if(static_cast<int>(state.metadata.size()) == frameCount) return;
    throw std::runtime_error("ReplayVideo metadata count " + std::to_string(state.metadata.size()) + " does not match frame count "
                             + std::to_string(frameCount));
}

void ReplayVideo::buildStage1() {
    if(state != nullptr) return;
    if(properties.fpsSet && !std::isfinite(properties.fps)) throw std::runtime_error("ReplayVideo fps is not a finite number");
    if(properties.fpsSet && properties.fps < 0.f) throw std::runtime_error("ReplayVideo fps must be 0 or more");
    if(properties.sizeSet && (properties.width <= 0 || properties.height <= 0 || (properties.width % 2) != 0 || (properties.height % 2) != 0)) {
        throw std::runtime_error("ReplayVideo size " + std::to_string(properties.width) + "x" + std::to_string(properties.height) + " is not supported");
    }
    if(properties.resolvedPath.empty()) throw std::runtime_error("ReplayVideo source is not prepared");
    if(properties.persistentUpload) replayCacheMarkPersistent(properties.sourceHash, properties.resolvedMetadataPath);
    state = std::make_unique<State>();
    try {
        state->loopSlot = replayLoopSlot(*this);
        state->hash = properties.sourceHash;
        state->loop = properties.loop;
        state->stopAtEnd = properties.stopPipelineAtEnd;
        state->fpsSet = properties.fpsSet && properties.fps > 0.f && std::isfinite(properties.fps);
        state->fps = properties.fps;
        const auto originalName = fileNameOf(properties.sourcePath);
        std::filesystem::path media(properties.resolvedPath);
        auto recording = std::make_unique<RecordingReadLease>();
        recording->acquire(media);
        if(!std::filesystem::exists(media)) throw std::runtime_error("ReplayVideo source is missing: " + media.string());
        if(!std::filesystem::is_directory(media) && (isArchiveName(originalName) || isArchiveName(fileNameOf(media)))) {
            holdScratch(state->scratchLeases, unpackScratchName(media, state->hash));
            const auto root = extractArchive(media, state->hash);
            media = properties.archiveEntrySet ? findNamed(root, properties.archiveEntry, false) : replay::selectReplayArchiveSource(root);
        }
        state->media = media;
        const auto mediaName = fileNameOf(media);
        if(std::filesystem::is_directory(media) || mediaName.rfind(".dai") != std::string::npos) {
            state->frames = std::filesystem::is_directory(media) ? listFrameFiles(media) : std::vector<std::filesystem::path>{media};
            state->dai = lowerCopy(state->frames.front().extension().string()) == ".dai";
            state->compressedImages = !state->dai;
            state->count = static_cast<int>(state->frames.size());
            for(const auto& frame : state->frames) requireReadablePayload(frame, fileNameOf(frame));
            if(state->dai) {
                bool first = true;
                std::int64_t previousSeq = 0;
                SteadyTime previousDevice{};
                for(std::size_t index = 0; index < state->frames.size(); index++) {
                    auto probe = loadDaiFrame(state->frames[index]);
                    if(index == 0) {
                        state->srcW = static_cast<int>(probe->getWidth());
                        state->srcH = static_cast<int>(probe->getHeight());
                        const auto sourceType = probe->getType();
                        state->passthrough = !properties.outFrameTypeSet && !properties.sizeSet;
                        if(state->passthrough) {
                            state->outType = sourceType;
                        } else if(!isReplayOutType(sourceType)) {
                            throw std::runtime_error(std::string("ReplayVideo output type is not supported: ") + replayTypeName(sourceType));
                        } else {
                            state->outType = properties.outFrameTypeSet ? static_cast<ImgFrame::Type>(properties.outFrameType) : sourceType;
                            if(!isReplayOutType(state->outType)) {
                                throw std::runtime_error(std::string("ReplayVideo output type is not supported: ") + replayTypeName(state->outType));
                            }
                        }
                    }
                    if(first) {
                        state->seqFirst = probe->getSequenceNum();
                        state->deviceFirst = probe->getTimestampDevice();
                        first = false;
                    } else {
                        if(probe->getSequenceNum() <= previousSeq) {
                            throw std::runtime_error("ReplayVideo sequence numbers do not increase: " + std::to_string(previousSeq) + " then "
                                                     + std::to_string(probe->getSequenceNum()));
                        }
                        if(probe->getTimestampDevice() < previousDevice) throw std::runtime_error("ReplayVideo timestamps decrease");
                    }
                    previousSeq = probe->getSequenceNum();
                    previousDevice = probe->getTimestampDevice();
                }
                state->seqLast = previousSeq;
                state->deviceLast = previousDevice;
                state->recorded = true;
            } else {
                for(std::size_t index = 0; index < state->frames.size(); ++index) {
                    const auto decoded = decodeImage(state->frames[index], [this] { return !state->stopping.load(); });
                    if(!decoded.sample) throw std::runtime_error("ReplayVideo preparation cancelled");
                    if(index == 0) {
                        state->srcW = decoded.player->width;
                        state->srcH = decoded.player->height;
                        state->fullRange = decoded.player->fullRange;
                    } else if(decoded.player->width != state->srcW || decoded.player->height != state->srcH) {
                        throw std::runtime_error("ReplayVideo frame size changed: " + state->frames[index].string());
                    }
                }
            }
        } else {
            // The cache file has no extension. An unpacked frame does. Prefer the file
            // that is actually opened, and fall back to the caller's source name.
            const auto fromMedia = chainsFor(mediaName);
            const auto chains = !fromMedia.empty() ? fromMedia : chainsFor(originalName);
            if(chains.empty()) throw std::runtime_error("ReplayVideo unknown extension");
            requireReadablePayload(media, !fromMedia.empty() ? mediaName : originalName);
            state->player = openFirst(media, chains);
            state->fullRange = state->player->fullRange;
            bool eos = false;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            SamplePtr sample(pullSample(*state->player, eos, [&] { return std::chrono::steady_clock::now() < deadline && !state->stopping.load(); }),
                             gst_sample_unref);
            if(!sample || !state->player->readGeometry(sample.get())) throw std::runtime_error("ReplayVideo frame decode failed: " + media.string());
            sample.reset();
            state->player->restart();
            state->srcW = state->player->width;
            state->srcH = state->player->height;
            state->count = state->player->estimatedCount();
            state->fullRange = state->player->fullRange;
        }
        if(!properties.sizeSet && state->srcW > 0 && ((state->srcW % 2) != 0 || (state->srcH % 2) != 0)) {
            throw std::runtime_error("ReplayVideo size " + std::to_string(state->srcW) + "x" + std::to_string(state->srcH) + " is not supported");
        }
        state->outW = properties.sizeSet ? properties.width : state->srcW;
        state->outH = properties.sizeSet ? properties.height : state->srcH;
        if(!state->dai) {
            state->outType = properties.outFrameTypeSet ? static_cast<ImgFrame::Type>(properties.outFrameType) : ImgFrame::Type::YUV420p;
        }
        if(!state->passthrough && !isReplayOutType(state->outType)) {
            throw std::runtime_error(std::string("ReplayVideo output type is not supported: ") + replayTypeName(state->outType));
        }
        std::filesystem::path metadata;
        if(!properties.resolvedMetadataPath.empty())
            metadata = properties.resolvedMetadataPath;
        else if(properties.metadataPathSet)
            metadata = properties.metadataPath;
        else {
            auto sibling = state->media;
            if(std::filesystem::is_directory(sibling) && !state->frames.empty()) {
                const auto stem = state->frames.front().stem().string();
                sibling /= stem.substr(0, stem.rfind('_')) + ".mcap";
            } else {
                sibling.replace_extension(".mcap");
            }
            if(std::filesystem::exists(sibling)) metadata = sibling;
        }
        if(properties.metadataPathSet && !metadata.empty() && !std::filesystem::is_regular_file(metadata)) {
            throw std::runtime_error("ReplayVideo metadata file is missing: " + metadata.string());
        }
        if(state->dai && !metadata.empty() && logger) logger->warn("ReplayVideo ignored metadata file for a .dai stream: {}", metadata.string());
        if(!state->dai && !metadata.empty() && std::filesystem::exists(metadata)) readMetadata(metadata, *state, state->count);
        if(state->recorded) state->nominal = nominalOf(state->fpsSet, state->fps, state->deviceFirst, state->deviceLast, state->count);
        if(properties.decodeMode == static_cast<std::int32_t>(ReplayDecodeMode::PRE_DECODE) && (state->player != nullptr || state->compressedImages)) {
            const auto scratchName = preScratchName(state->hash);
            holdScratch(state->scratchLeases, scratchName);
            const auto directory = std::filesystem::path("/data/depthai/replay_cache") / scratchName / ("node_" + std::to_string(id));
            std::filesystem::create_directories(directory);
            state->predecodedStride = state->srcW + state->srcW % 2;
            state->predecodedUvOffset = state->predecodedStride * state->srcH;
            int index = 0;
            while(!state->stopping.load()) {
                DecodedImage decoded;
                GstPlayer* player = state->player.get();
                SamplePtr sample(nullptr, gst_sample_unref);
                if(state->compressedImages) {
                    if(index >= state->count) break;
                    decoded = decodeImage(state->frames[static_cast<std::size_t>(index)], [this] { return !state->stopping.load(); });
                    if(!decoded.sample) break;
                    player = decoded.player.get();
                    sample = std::move(decoded.sample);
                } else {
                    bool eos = false;
                    sample.reset(pullSample(*player, eos, [this] { return !state->stopping.load(); }));
                    if(!sample) {
                        if(!eos) throw std::runtime_error("ReplayVideo frame decode failed");
                        break;
                    }
                }
                char name[32];
                std::snprintf(name, sizeof(name), "frame_%08d.nv12", index);
                saveNv12Sample(*player, sample.get(), directory / name);
                ++index;
                const int stride = state->count > 0 ? std::max(state->count / 10, 1) : 100;
                if(logger && (index < 10 || index % stride == 0 || index == state->count)) {
                    logger->info("Replay decode {} {}/{} frames", state->hash, index, std::max(state->count, index));
                }
            }
            if(index == 0) throw std::runtime_error("ReplayVideo frame decode failed");
            state->count = index;
            requireFrameCount(*state, index);
            if(state->recorded) state->nominal = nominalOf(state->fpsSet, state->fps, state->deviceFirst, state->deviceLast, state->count);
            state->frames.clear();
            for(int frame = 0; frame < index; frame++) {
                char name[32];
                std::snprintf(name, sizeof(name), "frame_%08d.nv12", frame);
                state->frames.push_back(directory / name);
            }
            if(state->player) state->player->close();
            state->player.reset();
            state->compressedImages = false;
        }
        if(state->loopSlot) {
            updateReplayLoop(
                *state->loopSlot, state->recorded, state->count, state->seqFirst, state->seqLast, state->deviceFirst, state->deviceLast, state->nominal, 0);
        }
        state->recording = std::move(recording);
    } catch(...) {
        state.reset();
        throw;
    }
}

void ReplayVideo::stop() {
    if(state != nullptr) {
        state->stopping = true;
        if(state->player != nullptr) state->player->close();
    }
    ThreadedNode::stop();
}

void ReplayVideo::run() {
    if(state == nullptr) throw std::runtime_error("ReplayVideo source is not prepared");
    if(state->loopSlot == nullptr) state->loopSlot = replayLoopSlot(*this);
    [[maybe_unused]] ReplayLoopSession loopSession(*state->loopSlot);
    replayCacheArm(state->hash, properties.resolvedMetadataPath);
    replay::PlaybackPacer pacing(state->fpsSet ? state->fps : 0.f);
    std::optional<SteadyTime> previousDevice;
    bool naturalEnd = false;
    int sent = 0;
    auto syncLoop = [&]() {
        return updateReplayLoop(*state->loopSlot,
                                state->recorded,
                                state->count,
                                state->seqFirst,
                                state->seqLast,
                                state->deviceFirst,
                                state->deviceLast,
                                state->nominal,
                                playedPass(sent, state->count));
    };
    while(mainLoop() && !state->stopping.load()) {
        const int index = state->count > 0 ? sent % state->count : sent;
        if(sent > 0 && index == 0 && state->count > 0) {
            if(!state->loop) {
                naturalEnd = true;
                break;
            }
            if(state->player != nullptr) state->player->restart();
        }

        std::shared_ptr<ImgFrame> frame;
        RecordedTiming timing;
        if(state->dai) {
            frame = loadDaiFrame(state->frames[static_cast<std::size_t>(index)]);
            timing.present = true;
            timing.sequence = frame->getSequenceNum();
            timing.timestamp = frame->getTimestamp();
            timing.timestampDevice = frame->getTimestampDevice();
            timing.timestampSystem = frame->getTimestampSystem();
            if(!state->passthrough) frame = convertDaiFrame(frame, state->outType, state->outW, state->outH);
        } else if(state->compressedImages) {
            const auto decoded = decodeImage(state->frames[static_cast<std::size_t>(index)], [this] { return mainLoop() && !state->stopping.load(); });
            if(!decoded.sample) break;
            frame = frameFromSample(*decoded.player, decoded.sample.get(), state->outType, state->outW, state->outH);
        } else if(state->player != nullptr) {
            bool eos = false;
            SamplePtr sample(pullSample(*state->player, eos, [this] { return mainLoop() && !state->stopping.load(); }), gst_sample_unref);
            if(sample == nullptr) {
                if(state->stopping.load() || !mainLoop()) break;
                const int got = state->count <= 0 ? sent : ((sent % state->count) == 0 && sent > 0 ? state->count : sent % state->count);
                if(got <= 0) throw std::runtime_error("ReplayVideo frame decode failed");
                requireFrameCount(*state, got);
                if(state->count <= 0) {
                    state->count = got;
                    if(state->recorded) {
                        state->nominal = nominalOf(state->fpsSet, state->fps, state->deviceFirst, state->deviceLast, state->count);
                        syncLoop();
                    }
                } else if(got != state->count) {
                    throw std::runtime_error("ReplayVideo frame decode failed");
                }
                if(!state->loop) {
                    naturalEnd = true;
                    break;
                }
                state->player->restart();
                continue;
            }
            frame = frameFromSample(*state->player, sample.get(), state->outType, state->outW, state->outH);
        } else {
            std::ifstream input(state->frames[static_cast<std::size_t>(index)], std::ios::binary);
            const auto chromaRows = state->srcH / 2 + state->srcH % 2;
            std::vector<std::uint8_t> bytes(static_cast<std::size_t>(state->predecodedUvOffset)
                                            + static_cast<std::size_t>(state->predecodedStride) * chromaRows);
            input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
            if(!input) throw std::runtime_error("ReplayVideo predecoded frame cannot be read: " + state->frames[static_cast<std::size_t>(index)].string());
            frame = frameFromNv12(bytes.data(),
                                  state->predecodedStride,
                                  state->predecodedUvOffset,
                                  state->srcW,
                                  state->srcH,
                                  state->outType,
                                  state->outW,
                                  state->outH,
                                  state->fullRange,
                                  nullptr);
        }
        if(!state->dai && state->recorded && index < static_cast<int>(state->metadata.size())) {
            const auto& metadata = state->metadata[static_cast<std::size_t>(index)];
            timing.present = true;
            timing.sequence = metadata->getSequenceNum();
            timing.timestamp = metadata->getTimestamp();
            timing.timestampDevice = metadata->getTimestampDevice();
            timing.timestampSystem = metadata->getTimestampSystem();
            frame->sourceFb = metadata->sourceFb;
            frame->cam = metadata->cam;
            frame->instanceNum = metadata->instanceNum;
            frame->category = metadata->category;
            if(metadata->transformation.getSize().first != 0) frame->transformation = metadata->transformation;
        }
        if(state->recorded && timing.present) {
            const auto [loopIndex, maxSpan, maxSeqStride] = syncLoop();
            applyRecorded(frame, state->seqFirst, state->deviceFirst, timing, loopIndex, maxSpan, maxSeqStride);
        } else {
            frame->setSequenceNum(sent);
            frame->setTimestampSystem(std::nullopt);
        }
        if(!state->passthrough && state->srcW > 0 && (state->outW != state->srcW || state->outH != state->srcH)
           && frame->transformation.getSize().first == static_cast<std::size_t>(state->srcW)
           && frame->transformation.getSize().second == static_cast<std::size_t>(state->srcH)) {
            frame->transformation.addScale(static_cast<float>(state->outW) / static_cast<float>(state->srcW),
                                           static_cast<float>(state->outH) / static_cast<float>(state->srcH));
        }
        std::optional<replay::PlaybackClock::duration> recordedInterval;
        if(state->recorded && timing.present) {
            if(index == 0 && sent > 0)
                recordedInterval = state->nominal;
            else if(previousDevice)
                recordedInterval = pacedDelta(timing.timestampDevice - *previousDevice, state->nominal);
            previousDevice = timing.timestampDevice;
        }
        const auto sendTime = pacing.waitToSend(recordedInterval, [this] { return mainLoop() && !state->stopping.load(); });
        if(!sendTime) break;
        if(!state->recorded || !timing.present) {
            frame->setTimestamp(*sendTime);
            frame->setTimestampDevice(*sendTime);
        }
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(frame);
        }
        pacing.sent();
        sent++;
        if(!state->loop && state->count > 0 && sent >= state->count) {
            naturalEnd = true;
            break;
        }
    }
    if(naturalEnd && mainLoop()) {
        if(logger) logger->info("Replay finished");
        if(state->stopAtEnd && !state->loop) stopPipeline();
    }
}

void ReplayMetadataOnly::buildStage1() {
    if(state != nullptr) return;
    if(properties.fpsSet && !std::isfinite(properties.fps)) throw std::runtime_error("ReplayMetadataOnly fps is not a finite number");
    if(properties.fpsSet && properties.fps < 0.f) throw std::runtime_error("ReplayMetadataOnly fps must be 0 or more");
    if(properties.resolvedPath.empty()) throw std::runtime_error("ReplayMetadataOnly source is not prepared");
    if(properties.persistentUpload) replayCacheMarkPersistent(properties.sourceHash);
    state = std::make_unique<State>();
    try {
        state->loopSlot = replayLoopSlot(*this);
        state->hash = properties.sourceHash;
        state->loop = properties.loop;
        state->stopAtEnd = properties.stopPipelineAtEnd;
        state->fpsSet = properties.fpsSet && properties.fps > 0.f && std::isfinite(properties.fps);
        state->fps = properties.fps;
        std::filesystem::path media(properties.resolvedPath);
        auto recording = std::make_unique<RecordingReadLease>();
        recording->acquire(media);
        if(!std::filesystem::exists(media)) throw std::runtime_error("ReplayMetadataOnly source is missing: " + media.string());
        const auto originalName = fileNameOf(properties.sourcePath);
        if(isArchiveName(originalName) || isArchiveName(fileNameOf(media)) || std::filesystem::is_directory(media)) {
            std::filesystem::path root = media;
            if(!std::filesystem::is_directory(media)) {
                holdScratch(state->scratchLeases, unpackScratchName(media, state->hash));
                root = extractArchive(media, state->hash);
            }
            if(properties.archiveEntry.empty()) throw std::runtime_error("ReplayMetadataOnly source is not an mcap file: " + media.string());
            media = findNamed(root, properties.archiveEntry, true);
        }
        utility::BytePlayer player;
        const auto schema = player.init(media.string());
        state->datatype = replay::datatypeFromSchema(schema);
        bool first = true;
        std::int64_t previousSeq = 0;
        SteadyTime previousDevice{};
        while(auto buffer = replay::nextRecordedBuffer(player, state->datatype)) {
            if(first) {
                state->seqFirst = buffer->getSequenceNum();
                state->deviceFirst = buffer->getTimestampDevice();
                first = false;
            } else {
                if(buffer->getSequenceNum() <= previousSeq) {
                    throw std::runtime_error("ReplayMetadataOnly sequence numbers do not increase: " + std::to_string(previousSeq) + " then "
                                             + std::to_string(buffer->getSequenceNum()));
                }
                if(buffer->getTimestampDevice() < previousDevice) throw std::runtime_error("ReplayMetadataOnly timestamps decrease");
            }
            previousSeq = buffer->getSequenceNum();
            previousDevice = buffer->getTimestampDevice();
            state->count++;
        }
        if(state->count == 0) throw std::runtime_error("ReplayMetadataOnly metadata file is empty: " + media.string());
        state->seqLast = previousSeq;
        state->deviceLast = previousDevice;
        state->nominal = nominalOf(state->fpsSet, state->fps, state->deviceFirst, state->deviceLast, state->count);
        if(state->loopSlot) {
            updateReplayLoop(*state->loopSlot, true, state->count, state->seqFirst, state->seqLast, state->deviceFirst, state->deviceLast, state->nominal, 0);
        }
        state->media = media;
        state->recording = std::move(recording);
    } catch(...) {
        state.reset();
        throw;
    }
}

void ReplayMetadataOnly::run() {
    if(state == nullptr || state->media.empty()) throw std::runtime_error("ReplayMetadataOnly source is not prepared");
    if(state->loopSlot == nullptr) state->loopSlot = replayLoopSlot(*this);
    [[maybe_unused]] ReplayLoopSession loopSession(*state->loopSlot);
    replayCacheArm(state->hash);
    utility::BytePlayer player;
    player.init(state->media.string());
    replay::PlaybackPacer pacing(state->fpsSet ? state->fps : 0.f);
    bool naturalEnd = false;
    int sent = 0;
    SteadyTime previousDevice{};
    bool havePrevious = false;
    while(mainLoop()) {
        const int index = sent % std::max(state->count, 1);
        if(sent > 0 && index == 0) {
            if(!state->loop) {
                naturalEnd = true;
                break;
            }
            player.restart();
            havePrevious = false;
        }
        auto buffer = replay::nextRecordedBuffer(player, state->datatype);
        if(buffer == nullptr) {
            if(!state->loop) {
                naturalEnd = true;
                break;
            }
            player.restart();
            havePrevious = false;
            continue;
        }
        RecordedTiming timing;
        timing.present = true;
        timing.sequence = buffer->getSequenceNum();
        timing.timestamp = buffer->getTimestamp();
        timing.timestampDevice = buffer->getTimestampDevice();
        timing.timestampSystem = buffer->getTimestampSystem();
        std::optional<replay::PlaybackClock::duration> recordedInterval;
        if(sent > 0) {
            if(index == 0)
                recordedInterval = state->nominal;
            else if(havePrevious)
                recordedInterval = pacedDelta(timing.timestampDevice - previousDevice, state->nominal);
        }
        const auto [loopIndex, maxSpan, maxSeqStride] = updateReplayLoop(*state->loopSlot,
                                                                         true,
                                                                         state->count,
                                                                         state->seqFirst,
                                                                         state->seqLast,
                                                                         state->deviceFirst,
                                                                         state->deviceLast,
                                                                         state->nominal,
                                                                         playedPass(sent, state->count));
        applyRecorded(buffer, state->seqFirst, state->deviceFirst, timing, loopIndex, maxSpan, maxSeqStride);
        previousDevice = timing.timestampDevice;
        havePrevious = true;
        if(!pacing.waitToSend(recordedInterval, [this] { return mainLoop(); })) break;
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(buffer);
        }
        pacing.sent();
        sent++;
        if(!state->loop && sent >= state->count) {
            naturalEnd = true;
            break;
        }
    }
    if(naturalEnd && mainLoop()) {
        if(logger) logger->info("Replay finished");
        if(state->stopAtEnd && !state->loop) stopPipeline();
    }
}

std::tuple<int, int, double> replayCacheSourceSize(const std::string& absolutePath) {
    const std::filesystem::path path(absolutePath);
    if(!path.is_absolute()) throw std::runtime_error("ReplayVideo device path must be absolute: " + absolutePath);
    if(!std::filesystem::exists(path)) throw std::runtime_error("ReplayVideo source is missing: " + absolutePath);
    if(std::filesystem::is_directory(path)) {
        const auto frames = listFrameFiles(path);
        if(lowerCopy(frames.front().extension().string()) == ".dai") {
            auto frame = loadDaiFrame(frames.front());
            return {static_cast<int>(frame->getWidth()), static_cast<int>(frame->getHeight()), 0.0};
        }
        auto player = openFirst(frames.front(), chainsFor(fileNameOf(frames.front())));
        bool eos = false;
        GstSample* sample = player->pull(eos);
        if(sample == nullptr || !player->readGeometry(sample)) throw std::runtime_error("ReplayVideo source size is not set");
        const auto size = std::tuple<int, int, double>{player->width, player->height, player->fps};
        gst_sample_unref(sample);
        return size;
    }
    const auto chains = chainsFor(fileNameOf(path));
    if(chains.empty()) throw std::runtime_error("ReplayVideo source size is not set");
    auto player = openFirst(path, chains);
    bool eos = false;
    GstSample* sample = nullptr;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while(sample == nullptr && std::chrono::steady_clock::now() < deadline) {
        sample = player->pull(eos);
        if(eos) break;
    }
    if(sample == nullptr || !player->readGeometry(sample)) throw std::runtime_error("ReplayVideo source size is not set");
    const auto size = std::tuple<int, int, double>{player->width, player->height, player->fps};
    gst_sample_unref(sample);
    return size;
}

}  // namespace gate
}  // namespace dai
