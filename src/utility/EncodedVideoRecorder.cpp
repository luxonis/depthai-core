#include "EncodedVideoRecorder.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#ifdef DEPTHAI_ENABLE_MP4V2
    #include <mp4v2.h>
#endif

namespace dai {
namespace utility {
namespace {
constexpr unsigned int MP4V2_TIMESCALE = 90000;

enum class NALU { P = 1, I = 5, SPS = 7, PPS = 8, INVALID = 0x00 };

struct NaluView {
    dai::span<const uint8_t> data;
    NALU type = NALU::INVALID;
};

struct H26xNals {
    dai::span<const uint8_t> data;
    size_t index = 0;

    explicit H26xNals(dai::span<const uint8_t> data) : data(data) {}

    static bool findStartCode(dai::span<const uint8_t> data, size_t offset, size_t& start, size_t& startCodePos) {
        for(size_t i = offset; i + 3 < data.size(); ++i) {
            if(data[i] == 0 && data[i + 1] == 0) {
                if(data[i + 2] == 1) {
                    startCodePos = i;
                    start = i + 3;
                    return true;
                }
                if(data[i + 2] == 0 && data[i + 3] == 1) {
                    startCodePos = i;
                    start = i + 4;
                    return true;
                }
            }
        }
        return false;
    }

    std::optional<NaluView> next() {
        if(index >= data.size()) return std::nullopt;
        size_t start = 0;
        size_t startCodePos = 0;
        if(!findStartCode(data, index, start, startCodePos)) {
            if(index == 0 && !data.empty()) {
                index = data.size();
                return NaluView{data, static_cast<NALU>(data[0] & 0x1F)};
            }
            return std::nullopt;
        }
        size_t nextStart = 0;
        size_t nextStartCodePos = 0;
        size_t end = data.size();
        if(findStartCode(data, start, nextStart, nextStartCodePos)) {
            end = nextStartCodePos;
        }
        if(end <= start) {
            index = end;
            return std::nullopt;
        }
        index = end;
        auto nal = data.subspan(start, end - start);
        return NaluView{nal, static_cast<NALU>(nal[0] & 0x1F)};
    }
};

}  // namespace

struct EncodedVideoRecorder::State {
#ifdef DEPTHAI_ENABLE_MP4V2
    MP4FileHandle mp4Writer = MP4_INVALID_FILE_HANDLE;
    MP4TrackId mp4Track = MP4_INVALID_TRACK_ID;
    unsigned int width = 0;
    unsigned int height = 0;
    MP4Duration duration = 0;
    bool mjpeg = false;
    bool waitForKeyframe = false;
    bool sawKeyframe = false;
    bool spsInstalled = false;
    bool ppsInstalled = false;
    std::vector<uint8_t> cachedSps;
    std::vector<uint8_t> cachedPps;

    static bool sameBytes(span<const uint8_t> left, const std::vector<uint8_t>& right) {
        return left.size() == right.size() && std::equal(left.begin(), left.end(), right.begin());
    }

    void remember(NALU type, span<const uint8_t> data) {
        if(type == NALU::SPS)
            cachedSps.assign(data.begin(), data.end());
        else if(type == NALU::PPS)
            cachedPps.assign(data.begin(), data.end());
    }

    void addParameterSet(NALU type, span<const uint8_t> data) {
        if(data.size() > std::numeric_limits<uint16_t>::max()) throw std::runtime_error("Record H264 parameter set is too large");
        const auto size = static_cast<uint16_t>(data.size());
        if(type == NALU::SPS) {
            if(data.size() < 4) throw std::runtime_error("Record invalid H264 SPS");
            MP4SetTrackIntegerProperty(mp4Writer, mp4Track, "mdia.minf.stbl.stsd.avc1.avcC.AVCProfileIndication", data[1]);
            MP4SetTrackIntegerProperty(mp4Writer, mp4Track, "mdia.minf.stbl.stsd.avc1.avcC.profile_compatibility", data[2]);
            MP4SetTrackIntegerProperty(mp4Writer, mp4Track, "mdia.minf.stbl.stsd.avc1.avcC.AVCLevelIndication", data[3]);
            MP4AddH264SequenceParameterSet(mp4Writer, mp4Track, data.data(), size);
            spsInstalled = true;
        } else {
            MP4AddH264PictureParameterSet(mp4Writer, mp4Track, data.data(), size);
            ppsInstalled = true;
        }
    }

    void requireTrack(span<const uint8_t> sps = {}) {
        if(mp4Track != MP4_INVALID_TRACK_ID) return;
        if(mjpeg) {
            mp4Track = MP4AddVideoTrack(mp4Writer, MP4V2_TIMESCALE, duration, width, height, MP4_JPEG_VIDEO_TYPE);
        } else {
            // A triggered segment can start before the next SPS/keyframe. Keep its
            // samples; a later SPS supplies the decoder configuration for this track.
            const auto profileSource = sps.size() >= 4 ? sps : span<const uint8_t>(cachedSps);
            const uint8_t avcProfile = profileSource.size() >= 4 ? profileSource[1] : 77;
            const uint8_t compatibility = profileSource.size() >= 4 ? profileSource[2] : 0;
            const uint8_t level = profileSource.size() >= 4 ? profileSource[3] : 40;
            mp4Track = MP4AddH264VideoTrack(mp4Writer, MP4V2_TIMESCALE, duration, width, height, avcProfile, compatibility, level, 3);
        }
        if(mp4Track == MP4_INVALID_TRACK_ID) throw std::runtime_error("Failed to create MP4 video track");
        MP4SetVideoProfileLevel(mp4Writer, 0x7F);
        if(mjpeg) return;
        if(!cachedSps.empty() && !spsInstalled) addParameterSet(NALU::SPS, cachedSps);
        if(!cachedPps.empty() && !ppsInstalled) addParameterSet(NALU::PPS, cachedPps);
    }

    void parameterSet(const NaluView& nal) {
        const bool already = (nal.type == NALU::SPS && spsInstalled && sameBytes(nal.data, cachedSps))
                             || (nal.type == NALU::PPS && ppsInstalled && sameBytes(nal.data, cachedPps));
        if(already) return;
        remember(nal.type, nal.data);
        if(mp4Track == MP4_INVALID_TRACK_ID) {
            requireTrack(nal.type == NALU::SPS ? nal.data : span<const uint8_t>{});
            return;
        }
        addParameterSet(nal.type, nal.data);
    }

    void writeSample(span<const uint8_t> data, bool sync) {
        requireTrack();
        errno = 0;
        if(!MP4WriteSample(mp4Writer, mp4Track, data.data(), static_cast<uint32_t>(data.size()), duration, 0, sync)) {
            throw std::runtime_error(errno == ENOSPC ? "ENOSPC writing MP4 file" : "Failed to write sample to MP4 file");
        }
    }

    bool writeH264(span<const uint8_t> data) {
        H26xNals nals(data);
        std::vector<uint8_t> sample;
        sample.reserve(data.size());
        bool sync = false;
        while(auto nal = nals.next()) {
            if(nal->type == NALU::SPS || nal->type == NALU::PPS) {
                parameterSet(*nal);
                continue;
            }
            if(nal->type == NALU::I) sync = true;
            const auto size = static_cast<uint32_t>(nal->data.size());
            constexpr auto MAX_SAMPLE = std::numeric_limits<uint32_t>::max();
            if(sample.size() > MAX_SAMPLE - 4 || nal->data.size() > MAX_SAMPLE - 4 - sample.size()) throw std::runtime_error("Record H264 sample is too large");
            sample.push_back(static_cast<uint8_t>(size >> 24));
            sample.push_back(static_cast<uint8_t>(size >> 16));
            sample.push_back(static_cast<uint8_t>(size >> 8));
            sample.push_back(static_cast<uint8_t>(size));
            sample.insert(sample.end(), nal->data.begin(), nal->data.end());
        }
        if(sample.empty()) {
            if(waitForKeyframe && !sawKeyframe) return false;
            throw std::runtime_error("Record H264 frame contains no sample");
        }
        // A short segment that starts mid-GOP cannot be decoded. Hold those slices
        // until the IDR requested for this file arrives. The first written frame is
        // then the keyframe, so the segment's skip count stays zero.
        if(waitForKeyframe && !sawKeyframe && !sync) return false;
        writeSample(sample, sync);
        if(sync) sawKeyframe = true;
        return true;
    }

    bool write(span<const uint8_t> data) {
        if(mjpeg) {
            writeSample(data, true);
            return true;
        }
        return writeH264(data);
    }

#endif
};

EncodedVideoRecorder::EncodedVideoRecorder() = default;
EncodedVideoRecorder::~EncodedVideoRecorder() {
    try {
        close();
    } catch(const std::exception&) {
        // Explicit close reports errors. Destruction must release resources without throwing.
    }
}

void EncodedVideoRecorder::init(const std::string& filePath, unsigned int width, unsigned int height, double fps, bool mjpeg) {
#ifdef DEPTHAI_ENABLE_MP4V2
    if(state) throw std::runtime_error("EncodedVideoRecorder already initialized");
    if(std::filesystem::path(filePath).extension() != ".mp4") throw std::runtime_error("Encoded video can only be recorded in .mp4 format");
    if(width == 0 || height == 0 || width > 65535 || height > 65535) throw std::runtime_error("Record invalid video size");
    if(!std::isfinite(fps) || fps <= 0 || fps > MP4V2_TIMESCALE) throw std::runtime_error("Record invalid video fps");
    const double ticks = MP4V2_TIMESCALE / fps;
    if(!std::isfinite(ticks) || ticks >= static_cast<double>(std::numeric_limits<long long>::max())) throw std::runtime_error("Record video fps is too small");
    auto opened = std::make_unique<State>();
    opened->width = width;
    opened->height = height;
    opened->duration = static_cast<MP4Duration>(std::llround(ticks));
    opened->mjpeg = mjpeg;
    opened->mp4Writer = MP4Create(filePath.c_str());
    if(opened->mp4Writer == MP4_INVALID_FILE_HANDLE) throw std::runtime_error("Failed to create MP4 file");
    state = std::move(opened);
    MP4SetTimeScale(state->mp4Writer, MP4V2_TIMESCALE);
#else
    (void)filePath;
    (void)width;
    (void)height;
    (void)fps;
    (void)mjpeg;
    throw std::runtime_error("Encoded video requires DEPTHAI_ENABLE_MP4V2=ON");
#endif
}

void EncodedVideoRecorder::setH264ParameterSets(span<const uint8_t> sps, span<const uint8_t> pps) {
#ifdef DEPTHAI_ENABLE_MP4V2
    if(!state) throw std::runtime_error("EncodedVideoRecorder not initialized");
    if(state->mjpeg) throw std::runtime_error("H264 parameter sets do not apply to MJPEG");
    if(sps.size() < 4) throw std::runtime_error("Record invalid H264 SPS");
    if(pps.empty()) throw std::runtime_error("Record invalid H264 PPS");
    if(state->mp4Track != MP4_INVALID_TRACK_ID) throw std::runtime_error("H264 parameter sets must be set before the first sample");
    state->cachedSps.assign(sps.begin(), sps.end());
    state->cachedPps.assign(pps.begin(), pps.end());
    state->spsInstalled = false;
    state->ppsInstalled = false;
#else
    (void)sps;
    (void)pps;
    throw std::runtime_error("Encoded video requires DEPTHAI_ENABLE_MP4V2=ON");
#endif
}

H264ParameterSets EncodedVideoRecorder::copyH264ParameterSets() const {
#ifdef DEPTHAI_ENABLE_MP4V2
    if(!state) return {};
    return {state->cachedSps, state->cachedPps};
#else
    return {};
#endif
}

H264ParameterSets EncodedVideoRecorder::peekH264ParameterSets(span<const uint8_t> data) {
    H264ParameterSets sets;
    H26xNals nals(data);
    while(auto nal = nals.next()) {
        if(nal->type == NALU::SPS && nal->data.size() >= 4)
            sets.sps.assign(nal->data.begin(), nal->data.end());
        else if(nal->type == NALU::PPS && !nal->data.empty())
            sets.pps.assign(nal->data.begin(), nal->data.end());
    }
    return sets;
}

void EncodedVideoRecorder::setWaitForKeyframe(bool wait) {
#ifdef DEPTHAI_ENABLE_MP4V2
    if(!state) throw std::runtime_error("EncodedVideoRecorder not initialized");
    state->waitForKeyframe = wait;
#else
    (void)wait;
    throw std::runtime_error("Encoded video requires DEPTHAI_ENABLE_MP4V2=ON");
#endif
}

bool EncodedVideoRecorder::write(span<const uint8_t> data) {
#ifdef DEPTHAI_ENABLE_MP4V2
    if(!state) throw std::runtime_error("EncodedVideoRecorder not initialized");
    if(data.size() > std::numeric_limits<uint32_t>::max()) throw std::runtime_error("Record encoded frame is too large");
    return state->write(data);
#else
    (void)data;
    throw std::runtime_error("Encoded video requires DEPTHAI_ENABLE_MP4V2=ON");
#endif
}

void EncodedVideoRecorder::close() {
#ifdef DEPTHAI_ENABLE_MP4V2
    if(!state) return;
    const auto handle = state->mp4Writer;
    state.reset();
    errno = 0;
    MP4Close(handle);
    if(errno == ENOSPC) throw std::runtime_error("ENOSPC closing MP4 file");
#endif
}

}  // namespace utility
}  // namespace dai
