#include "SeekEagle.hpp"

#include <dirent.h>
#include <dlfcn.h>
#include <elf.h>
#include <fcntl.h>
#include <gbm.h>
#include <gpiod.h>
#include <limits.h>
#include <link.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <sys/eventfd.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <type_traits>
#include <vector>

#include "camera/CameraSingleton.hpp"
#include "camera/HalUtil.hpp"
#include "camera/LuxonisSidechannel.hpp"
#include "depthai/utility/VectorMemory.hpp"

namespace dai {
namespace gate {
namespace seek_eagle {

namespace {

// Package layout: <firmware dir>/seek-eagle/ holds libseekcamera.so.5.2 (Seek SDK 5.2.0.11 for the Raspberry Pi 5) and
// stub libraries with the SONAMEs of its other DT_NEEDED entries; everything it imports from them is replaced below, so
// the stubs only satisfy the loader.
constexpr const char* kPackageSubdir = "seek-eagle";
constexpr const char* kSdkLib = "libseekcamera.so.5.2";
constexpr const char* kStubLibs[] = {"libv4l2.so.0", "libv4l2subdev.so.0", "libmediactl.so.0"};
// Each Eagle runs an SDK copy of its own: Eagle i > 0 loads a byte copy of the library (<kSdkLib>.<i>), which the
// loader maps as a separate object.
constexpr int kMaxEagles = 10;

// Nodes the SDK looks for on a Raspberry Pi 5 with Seek's seekfpa driver; they only exist inside this process.
constexpr const char* kDevDir = "/dev";
constexpr const char* kMediaNode = "/dev/media0";
constexpr const char* kVideoNode = "/dev/video0";
constexpr const char* kResetNode = "/dev/seekfpa-reset";
constexpr const char* kI2cLink = "/dev/seekfpa-i2c-bus";
constexpr unsigned kV4lMajor = 81;
constexpr unsigned kVideoMinor = 200;
constexpr unsigned kSensorI2cAddress = 0x10;  // the SDK finds its media device by the sensor entity "sefpa <bus>-0010"
constexpr uint32_t kPitchAlignment = 16;      // bytesperline alignment of the RP1 CFE capture node
// Seek's QSPI flash passthrough modes (324x492, 324x16, 324x32) stream the core's calibration as video at a lower link
// rate than imaging. The SDK reads a calibration table as 324x492 YUYV frames (648-byte lines of flash data); the core
// sends the frame for the requested flash address again and again, about 501 lines each. camx captures their first 492
// lines in the eaglei1e sensor mode kFlashSensorMode (648x492, 8 bit).
constexpr unsigned kFlashPassthroughWidth = 324;
constexpr uint32_t kFlashWireWidth = 2 * kFlashPassthroughWidth;
constexpr uint32_t kFlashCaptureHeight = 492;
constexpr int kFlashSensorMode = 1;
constexpr int kFlashBuffers = 4;
constexpr auto kFlashWarmup = std::chrono::milliseconds(500);  // longest wait for a new flash session's first frame
constexpr int kFlashReadAttempts = 4;                          // SDK reads of a table over MIPI before it uses I2C
// In passthrough the core chains rows within one HS burst and damages some packets, now
// and then a whole row. The camera driver can let a CSIPHY's damaged packets and frames of another height pass instead
// of stopping; the frames are checked here before the SDK gets them. Without that driver parameter the flash format is
// refused and the SDK reads the calibration over I2C.
constexpr const char* kLenientRxParam = "/sys/module/camera/parameters/lux_lenient_rx_phys";
// Core registers the SDK writes for a flash read (I2C: 16-bit register address, then the data)
constexpr uint16_t kFlashAddrReg = 0x5001;        // 3 bytes: flash address the core streams from
constexpr uint16_t kFlashStartReg = 0x5017;       // 1 starts the read
constexpr uint16_t kQspiPassthroughReg = 0x4104;  // 0 ends the passthrough: the core streams imaging frames again
// A core the SDK has not calibrated before is calibrated at the first start, as flash passthrough frames (seconds) or
// over I2C (about 15 minutes), before the SDK starts capturing.
constexpr auto kStartMaxWait = std::chrono::minutes(30);
constexpr auto kCalibrationLogInterval = std::chrono::seconds(60);

// Real hardware: the cam-sensor node of the Eagle's slot (cell-index = the socket's "csi") names its CCI master and reset
// line, e.g. on the DEV-KIT R2 CSI-0: adapter cam-cci2-m1 and TLMM GPIO116, CSI-4: cam-cci1-m1 and TLMM GPIO120
constexpr const char* kDeviceTreeSoc = "/proc/device-tree/soc";
constexpr const char* kI2cFallback = "/dev/i2c-10";
constexpr unsigned long kResetIoctl = _IOW('s', 0x01, uint32_t);  // seekfpa SEFPA_IOCTL_SET_RESET_GPIO
constexpr const char* kSeekHome = "/home/root";                   // persistent calibration cache in $HOME/.seekthermal
constexpr const char* kDacSweepFile = "/tmp/dac-sweep.bin";

// Seek SDK API
constexpr uint32_t kIoTypeVideo = 0x04;              // SEEKCAMERA_IO_TYPE_VIDEO
constexpr uint32_t kFormatThermographyFloat = 0x10;  // SEEKCAMERA_FRAME_FORMAT_THERMOGRAPHY_FLOAT
constexpr uint32_t kFormatColorYuy2 = 0x400;         // SEEKCAMERA_FRAME_FORMAT_COLOR_YUY2
constexpr uint32_t kFormatPreAgc = 0x08;             // SEEKCAMERA_FRAME_FORMAT_PRE_AGC
constexpr int kEventConnect = 0;                     // SEEKCAMERA_MANAGER_EVENT_CONNECT
constexpr int kTemperatureUnitCelsius = 0;           // SEEKCAMERA_TEMPERATURE_UNIT_CELSIUS
constexpr int kColorPaletteWhiteHot = 0;             // SEEKCAMERA_COLOR_PALETTE_WHITE_HOT
constexpr int kShutterModeAuto = 0;                  // SEEKCAMERA_SHUTTER_MODE_AUTO
constexpr int kShutterModeManual = 1;                // SEEKCAMERA_SHUTTER_MODE_MANUAL
constexpr int kFilterGradientCorrection = 0;         // SEEKCAMERA_FILTER_GRADIENT_CORRECTION
constexpr int kFilterFlatSceneCorrection = 1;        // SEEKCAMERA_FILTER_FLAT_SCENE_CORRECTION
constexpr int kFilterSharpen = 2;                    // SEEKCAMERA_FILTER_SHARPEN_CORRECTION
constexpr int kFilterDisabled = 0, kFilterEnabled = 1;
constexpr int kFscId = 0;                            // SEEKCAMERA_FLAT_SCENE_CORRECTION_ID_0
constexpr int kFscStoreHostOnly = 1;                 // SEEKCAMERA_FLAT_SCENE_CORRECTION_STORE_MODE_HOST_ONLY
using MemoryAccessCallback = void (*)(size_t progress, void* user);  // seekcamera_memory_access_callback_t

struct Sdk {
    void* handle{nullptr};  // the SDK copy; its setters are looked up by name (sdkSet)
    int (*managerCreate)(void** manager, uint32_t discoveryMode);
    int (*managerRegisterEventCallback)(void* manager, void (*cb)(void*, int, int, void*), void* user);
    int (*registerFrameAvailableCallback)(void* camera, void (*cb)(void*, void*, void*), void* user);
    int (*captureSessionStart)(void* camera, uint32_t formats);
    int (*frameGetFrameByFormat)(const void* cameraFrame, uint32_t format, void** frame);
    size_t (*frameGetWidth)(void* frame);
    size_t (*frameGetHeight)(void* frame);
    size_t (*frameGetLineStride)(void* frame);
    void* (*frameGetData)(void* frame);
    const char* (*errorGetStr)(int error);
    int (*readFlashRegion)(void* ctx, uint32_t addr, uint8_t* out, uint32_t len, void* setup, void* setupArg, uint32_t* windows,
                           uint32_t maxWindows, uint32_t* nWindows);
};

struct Meta {
    dai::Timestamp ts{};
    uint64_t seq{0};
    dai::ImgFrame::CameraSettings cam{};
    std::string output;
};

struct VideoBuffer {
    uint8_t* mem{nullptr};
    bool queued{false};
    bool done{false};
    uint32_t sequence{0};
    timeval ts{};
    Meta meta;
};

struct FakeDir {
    std::vector<std::pair<std::string, unsigned char>> entries;
    size_t pos{0};
    dirent ent{};
};

// Entity of the emulated media device; libmediactl hands out pointers to these as struct media_entity*
struct MediaEntity {
    media_entity_desc desc{};
};

struct State {
    std::mutex m;
    std::condition_variable cv;
    std::mutex publishMtx;  // held while a frame is published, so attach(nullptr) waits for it
    std::shared_ptr<spdlog::async_logger> logger;
    Sdk sdk{};
    void* manager{nullptr};
    bool loaded{false};
    bool failed{false};
    std::string i2cDevice{kI2cFallback};
    std::vector<MediaEntity> entities;  // built once before the SDK is loaded, never changed afterwards
    std::string resetChipLabel;         // reset line of the slot: gpiochip label and line, from the device tree
    unsigned resetGpio{0};
    gpiod_chip* resetChip{nullptr};
    gpiod_line* resetLine{nullptr};
    pid_t fakePid{0};
    // emulated nodes; the fds are eventfds, so the SDK can poll() the capture node with the real libc
    std::vector<int> mediaFds, resetFds, videoFds;
    std::vector<FakeDir*> dirs;
    // the SDK's fds on the real I2C bus and its traffic to the core, which shows a calibration in progress
    std::vector<int> i2cFds;
    std::chrono::steady_clock::time_point lastI2c{};
    uint64_t i2cReadBytes{0};
    int socket{-1};  // depthai socket of the Eagle
    int cameraId{-1};
    int csiPhy{-1};  // CSIPHY of the Eagle's socket (its "csi" in the camera configuration)
    bool flashCloseRequested{false};
    // flash read: the address in the core's read registers, the one of the read last started and when (QTimer ns), and
    // the rows of a frame the SDK reads
    uint32_t flashAddr{0}, flashReadAddr{UINT32_MAX};
    int64_t flashReadStart{INT64_MAX};
    uint32_t flashReadRows{0};
    uint64_t flashInAtOpen{UINT64_MAX};  // nFlashIn when the camx flash session was opened, UINT64_MAX while closed
    std::vector<uint8_t> flashPrev;  // the last camx frame of that read, as the SDK gets it
    std::vector<uint8_t> flashGood;  // a frame of that read the core sent twice in a row
    // emulated capture device; a flash passthrough capture (flash) is not the imaging stream (streaming)
    uint32_t width{328}, height{248}, bytesPerLine{672}, sizeImage{672 * 248};
    std::vector<VideoBuffer> bufs;
    size_t bufStride{0};
    std::deque<int> queued, done;
    bool flash{false};
    bool streaming{false};
    bool flashStreaming{false};
    int streamFd{-1};
    uint32_t sequence{0};
    Meta lastDequeued;
    uint64_t nWire{0}, nNoBuffer{0}, nOut{0}, nFlashIn{0};
    // output: the SDK frame formats of the capture session, the settings it starts with and those waiting for the next SDK
    // frame; the SDK camera once connected and the size of its frames
    CameraSensor::FrameCallback cb;
    uint32_t formats{0};
    bool publishTemperature{false}, publishRaw{false};
    std::atomic<bool> fscBusy{false};  // a flat scene correction is being stored or cleared
    dai::ThermalConfig initialConfig;
    std::vector<dai::ThermalConfig> pendingConfigs;
    std::set<std::string> warnedFields;  // ThermalConfig fields the Eagle does not support, warned about once
    void* camera{nullptr};
    uint32_t outWidth{320}, outHeight{240};
    // The linear AGC is done here, from the temperature frames, with the palette's colors as YUV
    bool linearAgc{false};
    dai::ThermalConfig::ThermalLinearAgcLockMode linearLockMode{dai::ThermalConfig::ThermalLinearAgcLockMode::AUTO};
    std::optional<float> linearMin, linearMax;
    std::array<std::array<uint8_t, 3>, 256> paletteYuv{};
    // The SDK flips its color frames only: the temperature frames, and the color frames made here, are flipped here
    bool flipH{false}, flipV{false};
};

// The Eagle a call belongs to: set by the hooks of its SDK copy, its SDK callbacks, its flash session and the API
thread_local int tEagle = 0;
struct AsEagle {
    explicit AsEagle(int eagle) : prev(std::exchange(tEagle, eagle)) {}
    ~AsEagle() {
        tEagle = prev;
    }
    const int prev;
};
std::array<State, kMaxEagles>& states() {
    static std::array<State, kMaxEagles> s;
    return s;
}
State& st() {
    return states()[tEagle];
}
// Eagle of depthai socket `socket`: the first call for a socket takes the next free one (they are taken in order); -1 if
// none is left
int eagleOf(CameraBoardSocket socket) {
    static std::mutex m;
    std::lock_guard<std::mutex> l(m);
    auto& s = states();
    for(int i = 0; i < kMaxEagles; i++) {
        if(s[i].socket < 0) s[i].socket = static_cast<int>(socket);
        if(s[i].socket == static_cast<int>(socket)) return i;
    }
    return -1;
}

#define SEEK_LOG(level, ...)                             \
    do {                                                 \
        if(st().logger) st().logger->level(__VA_ARGS__); \
    } while(0)

int fail(int err) {
    errno = err;
    return -1;
}
bool contains(const std::vector<int>& v, int fd) {
    return std::find(v.begin(), v.end(), fd) != v.end();
}
bool isPath(const char* path, const char* node) {
    return path != nullptr && std::strcmp(path, node) == 0;
}
int newNodeFd() {
    return eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK | EFD_SEMAPHORE);
}

// ---------------------------------------------------------------------------------------------------------------------
// Emulated capture node "rp1-cfe-csi2_ch0" (V4L2 single-planar capture, MMAP buffers), fed by submitWireFrame()
void freeBuffers(State& s) {
    for(auto& b : s.bufs) munmap(b.mem, s.bufStride);
    s.bufs.clear();
    s.queued.clear();
    s.done.clear();
}
void stopStreaming(State& s) {
    if(s.flashStreaming) SEEK_LOG(info, "Seek Eagle: SDK flash capture stopped");
    s.streaming = false;
    s.flashStreaming = false;
    s.flashReadAddr = UINT32_MAX;
    s.flashReadStart = INT64_MAX;
    s.flashPrev.clear();
    s.flashGood.clear();
    for(auto& b : s.bufs) b.queued = b.done = false;
    s.queued.clear();
    s.done.clear();
    uint64_t v;
    if(s.streamFd >= 0)
        while(::read(s.streamFd, &v, sizeof v) > 0) {
        }
    s.streamFd = -1;
}
void fillFormat(uint32_t width, uint32_t height, uint32_t bytesPerLine, v4l2_format* f) {
    f->fmt.pix.width = width;
    f->fmt.pix.height = height;
    f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    f->fmt.pix.field = V4L2_FIELD_NONE;
    f->fmt.pix.bytesperline = bytesPerLine;
    f->fmt.pix.sizeimage = bytesPerLine * height;
    f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
}
// caller holds s.m
int videoIoctl(State& s, int fd, unsigned long req, void* arg) {
    switch(req) {
        case VIDIOC_S_FMT:
        case VIDIOC_TRY_FMT: {
            auto* f = static_cast<v4l2_format*>(arg);
            if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || f->fmt.pix.width == 0 || f->fmt.pix.height == 0) return fail(EINVAL);
            const uint32_t width = f->fmt.pix.width, height = f->fmt.pix.height;
            const uint32_t bytesPerLine = (width * 2 + kPitchAlignment - 1) & ~(kPitchAlignment - 1);
            if(req == VIDIOC_S_FMT) {
                if(!s.bufs.empty()) return fail(EBUSY);
                s.width = width;
                s.height = height;
                s.bytesPerLine = bytesPerLine;
                s.sizeImage = bytesPerLine * height;
                s.flash = width == kFlashPassthroughWidth;
                SEEK_LOG(info,
                         "Seek Eagle: SDK {} format {}x{}, pitch {} (SDK asked for {})",
                         s.flash ? "flash capture" : "capture",
                         width,
                         height,
                         bytesPerLine,
                         f->fmt.pix.bytesperline);
            }
            fillFormat(width, height, bytesPerLine, f);
            return 0;
        }
        case VIDIOC_G_FMT: {
            auto* f = static_cast<v4l2_format*>(arg);
            if(f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) return fail(EINVAL);
            fillFormat(s.width, s.height, s.bytesPerLine, f);
            return 0;
        }
        case VIDIOC_REQBUFS: {
            auto* r = static_cast<v4l2_requestbuffers*>(arg);
            if(r->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || r->memory != V4L2_MEMORY_MMAP || s.streaming) return fail(EINVAL);
            freeBuffers(s);
            const size_t page = static_cast<size_t>(getpagesize());
            s.bufStride = (s.sizeImage + page - 1) / page * page;
            r->count = std::min<uint32_t>(r->count, 32);
            for(uint32_t i = 0; i < r->count; i++) {
                void* mem = ::mmap(nullptr, s.bufStride, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
                if(mem == MAP_FAILED) {
                    freeBuffers(s);
                    return fail(ENOMEM);
                }
                s.bufs.push_back({static_cast<uint8_t*>(mem)});
            }
            return 0;
        }
        case VIDIOC_QUERYBUF:
        case VIDIOC_QBUF:
        case VIDIOC_DQBUF: {
            auto* b = static_cast<v4l2_buffer*>(arg);
            if(b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || b->memory != V4L2_MEMORY_MMAP) return fail(EINVAL);
            if(req == VIDIOC_DQBUF) {
                if(s.done.empty()) return fail(EAGAIN);
                const int idx = s.done.front();
                s.done.pop_front();
                uint64_t v;
                if(s.streamFd >= 0) (void)::read(s.streamFd, &v, sizeof v);
                auto& vb = s.bufs[idx];
                vb.done = false;
                b->index = idx;
                b->bytesused = s.sizeImage;
                b->flags = V4L2_BUF_FLAG_MAPPED | V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
                b->field = V4L2_FIELD_NONE;
                b->sequence = vb.sequence;
                b->timestamp = vb.ts;
                b->m.offset = idx * s.bufStride;
                b->length = s.sizeImage;
                s.lastDequeued = vb.meta;
                return 0;
            }
            if(b->index >= s.bufs.size()) return fail(EINVAL);
            auto& vb = s.bufs[b->index];
            if(req == VIDIOC_QBUF) {
                if(vb.queued || vb.done) return fail(EINVAL);
                vb.queued = true;
                s.queued.push_back(static_cast<int>(b->index));
                b->bytesused = 0;
            }
            b->m.offset = b->index * s.bufStride;
            b->length = s.sizeImage;
            b->flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC | (vb.queued ? V4L2_BUF_FLAG_QUEUED | V4L2_BUF_FLAG_MAPPED : 0);
            return 0;
        }
        case VIDIOC_STREAMON:
            if(s.bufs.empty()) return fail(EINVAL);
            (s.flash ? s.flashStreaming : s.streaming) = true;
            s.streamFd = fd;
            s.cv.notify_all();
            SEEK_LOG(info,
                     "Seek Eagle: SDK {} started ({}x{}, pitch {}, {} buffers)",
                     s.flash ? "flash capture" : "capture",
                     s.width,
                     s.height,
                     s.bytesPerLine,
                     s.bufs.size());
            return 0;
        case VIDIOC_STREAMOFF:
            if(s.flashStreaming) s.flashCloseRequested = true;  // each flash capture gets a camx session of its own
            stopStreaming(s);
            return 0;
        default:
            SEEK_LOG(warn, "Seek Eagle: unexpected capture ioctl 0x{:x}", req);
            return fail(ENOTTY);
    }
}

// caller holds s.m and checked that a buffer is queued: copies a wire frame (rows at `stride`) into it, padded to the
// node's pitch, and hands it to the SDK
void fillBuffer(State& s, const uint8_t* data, uint32_t width, uint32_t height, int stride, Meta meta) {
    const int idx = s.queued.front();
    s.queued.pop_front();
    auto& b = s.bufs[idx];
    const uint32_t rowBytes = std::min(width, s.bytesPerLine);
    for(uint32_t y = 0; y < s.height; y++) {
        uint8_t* row = b.mem + static_cast<size_t>(y) * s.bytesPerLine;
        if(y < height) {
            std::memcpy(row, data + static_cast<size_t>(y) * stride, rowBytes);
            std::memset(row + rowBytes, 0, s.bytesPerLine - rowBytes);
        } else {
            std::memset(row, 0, s.bytesPerLine);
        }
    }
    b.queued = false;
    b.done = true;
    b.sequence = s.sequence++;
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    b.ts = {now.tv_sec, now.tv_nsec / 1000};
    b.meta = std::move(meta);
    s.done.push_back(idx);
    const uint64_t one = 1;
    (void)::write(s.streamFd, &one, sizeof one);
}

// ---------------------------------------------------------------------------------------------------------------------
// Flash passthrough capture: a camx session of its own in the flash sensor mode. The SDK gives up on a table within about
// half a second, so the session is opened when the SDK sets the flash format of a capture (the core is in passthrough
// then), runs across the tables the SDK reads in that capture and is closed before the SDK ends the passthrough.
class FlashSession : public camera3_callback_ops_t {
   public:
    FlashSession() {
        process_capture_result = onResult;
        notify = onNotify;
        request_stream_buffers = onRequestBuffers;
        return_stream_buffers = onReturnBuffers;
        stream.stream_type = CAMERA3_STREAM_OUTPUT;
        stream.width = kFlashWireWidth;
        stream.height = kFlashCaptureHeight;
        stream.format = HAL_PIXEL_FORMAT_RAW_OPAQUE;
        stream.data_space = static_cast<android_dataspace>(0x0);
        stream.rotation = CAMERA3_STREAM_ROTATION_0;
        stream.usage = GRALLOC_USAGE_HW_VIDEO_ENCODER | GRALLOC_USAGE_SW_READ_OFTEN | GRALLOC_USAGE_SW_WRITE_OFTEN;
        stream.max_buffers = kFlashBuffers;
        stream.group_id = -1;
        stream.priv = nullptr;
    }

    // true while a session runs; false (the SDK then reads over I2C) if the camera driver cannot let the damaged packets
    // pass or camx refuses the session
    bool open(int cameraId, int csiPhy) {
        std::lock_guard<std::mutex> l(m);
        if(dev != nullptr) return true;
        if(failed) return false;
        eagle = tEagle;
        phy = csiPhy;
        const auto t0 = std::chrono::steady_clock::now();
        if(csiPhy < 0 || !setLenientRx(csiPhy, true)) {
            giveUp("the camera driver has no " + std::string(kLenientRxParam) + " or the Eagle's CSIPHY is unknown");
            return false;
        }
        auto* module = CameraSingleton::getInstance().cameraModule;
        hw_device_t* hw = nullptr;
        if(module == nullptr || module->common.methods->open(&module->common, std::to_string(cameraId).c_str(), &hw) != 0 || hw == nullptr) {
            giveUp("cannot open camera " + std::to_string(cameraId));
            return false;
        }
        dev = reinterpret_cast<camera3_device_t*>(hw);
        if(dev->ops->initialize(dev, this) != 0) {
            giveUp("initialize failed");
            return false;
        }
        android::hardware::camera::common::V1_0::helper::CameraMetadata params;
        const int32_t offlineIfe = 0;
        const uint8_t onBps[sizeof(int32_t)] = {0};
        params.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.sessionParameters.EnableOfflineIFE"), &offlineIfe, 1);
        params.update(HalUtil::getVendorTag("com.qti.chi.bpsrealtimecam.cameraRunningOnBPS"), onBps, sizeof onBps);
        camera3_stream_t* streams[] = {&stream};
        camera3_stream_configuration_t config{};
        config.num_streams = 1;
        config.streams = streams;
        // forced sensor mode, encoded as CameraSensor does
        config.operation_mode = CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE | ((kFlashSensorMode + 1) << 20) | (1 << 24);
        config.session_parameters = params.getAndLock();
        const int r = dev->ops->configure_streams(dev, &config);
        params.unlock(config.session_parameters);
        if(r != 0) {
            giveUp("configure_streams failed: " + std::to_string(r));
            return false;
        }
        settings = dev->ops->construct_default_request_settings(dev, CAMERA3_TEMPLATE_PREVIEW);
        if(settings == nullptr) {
            giveUp("no default request settings");
            return false;
        }
        if(!pool) pool = std::make_shared<CameraPool>(kFlashBuffers, std::nullopt);
        frames = errors = 0;
        {
            std::lock_guard<std::mutex> sl(st().m);
            st().flashInAtOpen = st().nFlashIn;
        }
        running = true;
        thread = std::thread(&FlashSession::loop, this);
        SEEK_LOG(info,
                 "Seek Eagle: flash capture session open (camx sensor mode {}, {}x{}, CSIPHY{}) in {} ms",
                 kFlashSensorMode,
                 kFlashWireWidth,
                 kFlashCaptureHeight,
                 csiPhy,
                 std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count());
        return true;
    }

    void close() {
        std::lock_guard<std::mutex> l(m);
        if(dev == nullptr) return;
        running = false;
        if(settings != nullptr) dev->ops->flush(dev);  // returns the requests in flight; the loop then sees !running
        if(thread.joinable()) thread.join();
        dev->common.close(&dev->common);
        dev = nullptr;
        settings = nullptr;
        setLenientRx(phy, false);
        {
            std::lock_guard<std::mutex> sl(st().m);
            st().flashInAtOpen = UINT64_MAX;
        }
        SEEK_LOG(info, "Seek Eagle: flash capture session closed ({} frames, {} errors)", frames.load(), errors.load());
    }

   private:
    // the parameter holds the CSIPHYs of all open flash sessions
    static bool setLenientRx(int csiPhy, bool on) {
        static std::mutex pm;
        static uint32_t phys = 0;
        std::lock_guard<std::mutex> l(pm);
        phys = on ? phys | 1u << csiPhy : phys & ~(1u << csiPhy);
        std::ofstream f(kLenientRxParam);
        f << phys << "\n";
        f.flush();
        return static_cast<bool>(f);
    }
    void giveUp(const std::string& why) {  // caller holds m
        SEEK_LOG(error, "Seek Eagle: flash capture session: {}; the SDK reads the calibration over I2C", why);
        if(dev != nullptr) dev->common.close(&dev->common);
        dev = nullptr;
        settings = nullptr;
        if(phy >= 0) setLenientRx(phy, false);
        failed = true;
    }
    void loop() {
        AsEagle e(eagle);
        for(uint32_t frameNumber = 0; running; frameNumber++) {
            camera3_stream_buffer_t buf = pool->getFreeBuffer(&stream);
            if(!running) {
                pool->returnBuffer(&buf);
                break;
            }
            camera3_capture_request_t req{};
            req.frame_number = frameNumber;
            req.settings = settings;
            req.num_output_buffers = 1;
            req.output_buffers = &buf;
            if(dev->ops->process_capture_request(dev, &req) != 0) {
                pool->returnBuffer(&buf);
                SEEK_LOG(error, "Seek Eagle: flash capture request {} failed", frameNumber);
                break;
            }
        }
    }
    static void onResult(const camera3_callback_ops* ops, const camera3_capture_result_t* result);
    static void onNotify(const camera3_callback_ops* ops, const camera3_notify_msg_t* msg) {
        auto* self = const_cast<FlashSession*>(static_cast<const FlashSession*>(ops));
        AsEagle e(self->eagle);
        if(msg->type == CAMERA3_MSG_SHUTTER) {
            std::lock_guard<std::mutex> l(self->shutterMtx);
            self->shutters[msg->message.shutter.frame_number % self->shutters.size()] = {msg->message.shutter.frame_number,
                                                                                       static_cast<int64_t>(msg->message.shutter.timestamp)};
            return;
        }
        if(msg->type != CAMERA3_MSG_ERROR) return;
        self->errors++;
        if(msg->message.error.error_code == CAMERA3_MSG_ERROR_DEVICE) SEEK_LOG(error, "Seek Eagle: flash capture session: camx device error");
    }
    static camera3_buffer_request_status_t onRequestBuffers(
        const camera3_callback_ops*, uint32_t, const camera3_buffer_request_t*, uint32_t*, camera3_stream_buffer_ret_t*) {
        return CAMERA3_BUF_REQ_FAILED_UNKNOWN;
    }
    static void onReturnBuffers(const camera3_callback_ops*, uint32_t, const camera3_stream_buffer_t* const*) {}

    std::mutex m;
    camera3_device_t* dev{nullptr};
    camera3_stream_t stream{};
    std::shared_ptr<CameraPool> pool;  // kept for the process: CameraPool does not free its buffers
    const camera_metadata_t* settings{nullptr};
    std::thread thread;
    std::atomic<bool> running{false};
    std::atomic<uint64_t> frames{0}, errors{0};
    bool failed{false};
    int eagle{0};  // the Eagle the session belongs to
    int phy{-1};
    std::mutex shutterMtx;
    std::array<std::pair<uint32_t, int64_t>, 8> shutters{};  // frame number -> start of frame (QTimer ns)
};
FlashSession& flashSession() {
    static FlashSession f[kMaxEagles];
    return f[tEagle];
}

// camx flash frame (started at sofNs, QTimer) -> the SDK's flash capture. A frame may have lost a row
// or belong to the previous read, so the SDK only gets a frame of the current read the core sent twice in a
// row, as often as it asks (it keeps the second frame after each read setup).
void submitFlashFrame(const uint8_t* data, int stride, int64_t sofNs) {
    auto& s = st();
    std::unique_lock<std::mutex> l(s.m);
    const uint64_t in = ++s.nFlashIn;
    s.cv.notify_all();
    if(!s.flash || sofNs <= s.flashReadStart) return;
    // only the rows the SDK reads: the core streams on past the end of a table, and that data does not repeat
    const uint32_t rows = std::min({s.height, kFlashCaptureHeight, s.flashReadRows ? s.flashReadRows : s.height});
    const size_t rowBytes = std::min(kFlashWireWidth, s.bytesPerLine);
    const auto row = [&](uint32_t y) { return data + static_cast<size_t>(y) * stride; };
    const bool hadPrev = s.flashPrev.size() == rows * rowBytes;
    uint32_t same = 0;  // leading rows equal to the last frame's
    while(hadPrev && same < rows && std::memcmp(s.flashPrev.data() + same * rowBytes, row(same), rowBytes) == 0) same++;
    const bool confirmed = hadPrev && same == rows;
    const bool newGood = confirmed && s.flashGood.empty();
    if(newGood) s.flashGood = s.flashPrev;
    if(!confirmed) {
        s.flashPrev.resize(rows * rowBytes);
        for(uint32_t y = 0; y < rows; y++) std::memcpy(s.flashPrev.data() + y * rowBytes, row(y), rowBytes);
    }
    if(!s.flashGood.empty() && s.flashStreaming && !s.queued.empty()) {
        fillBuffer(s, s.flashGood.data(), static_cast<uint32_t>(rowBytes), rows, static_cast<int>(rowBytes), {});
    }
    const uint32_t addr = s.flashReadAddr;
    const int64_t sinceStartMs = (sofNs - s.flashReadStart) / 1000000;
    l.unlock();
    if(newGood)
        SEEK_LOG(debug, "Seek Eagle: flash 0x{:06x}: frame {} ({} ms after the read start) equals the one before", addr, in, sinceStartMs);
    else if(hadPrev && !confirmed)
        SEEK_LOG(debug, "Seek Eagle: flash 0x{:06x}: frame {} differs from the one before from row {} on", addr, in, same);
}

void FlashSession::onResult(const camera3_callback_ops* ops, const camera3_capture_result_t* result) {
    auto* self = const_cast<FlashSession*>(static_cast<const FlashSession*>(ops));
    AsEagle e(self->eagle);
    for(uint32_t i = 0; i < result->num_output_buffers; i++) {
        const camera3_stream_buffer_t& b = result->output_buffers[i];
        if(b.status == CAMERA3_BUFFER_STATUS_OK) {
            const auto* hnd = static_cast<const CameraPool::private_handle_t*>(*b.buffer);
            void* p = ::mmap(nullptr, hnd->size, PROT_READ, MAP_SHARED, hnd->fd, 0);
            gbm_bo* bo = self->pool->getBo(&b);
            if(p != MAP_FAILED) {
                const int stride = bo ? static_cast<int>(gbm_bo_get_stride(bo)) : CameraPool::camxRawOpaqueStride(kFlashWireWidth);
                int64_t sofNs = 0;
                {
                    std::lock_guard<std::mutex> tl(self->shutterMtx);
                    const auto& e = self->shutters[result->frame_number % self->shutters.size()];
                    if(e.first == result->frame_number) sofNs = e.second;
                }
                submitFlashFrame(static_cast<const uint8_t*>(p), stride, sofNs);
                ::munmap(p, hnd->size);
                self->frames++;
            }
        } else {
            self->errors++;
        }
        self->pool->returnBuffer(&b);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Emulated media device: the graph of a Raspberry Pi 5 with the seekfpa driver, sensor "sefpa <bus>-0010" -> RP1 CFE
// "csi2" (sink pad 0, source pad 4) -> capture node "rp1-cfe-csi2_ch0". The SDK picks the media device by the sensor
// entity, enables both links and sets the csi2 pad formats; only the capture node does anything here.
std::vector<MediaEntity> makeMediaGraph(const std::string& i2cDevice) {
    unsigned bus = 0;
    std::sscanf(i2cDevice.c_str(), "/dev/i2c-%u", &bus);
    char sensor[32];
    std::snprintf(sensor, sizeof sensor, "sefpa %u-%04x", bus, kSensorI2cAddress);
    auto entity = [](uint32_t id, const char* name, uint32_t function, uint16_t pads, uint16_t links, uint32_t minor) {
        MediaEntity e;
        e.desc.id = id;
        std::strncpy(e.desc.name, name, sizeof e.desc.name - 1);
        e.desc.type = function;
        e.desc.pads = pads;
        e.desc.links = links;
        e.desc.dev.major = kV4lMajor;
        e.desc.dev.minor = minor;
        return e;
    };
    return {entity(1, "rp1-cfe-csi2_ch0", MEDIA_ENT_F_IO_V4L, 1, 1, kVideoMinor),
            entity(2, "csi2", MEDIA_ENT_F_VID_IF_BRIDGE, 5, 2, kVideoMinor + 1),
            entity(3, sensor, MEDIA_ENT_F_CAM_SENSOR, 1, 1, kVideoMinor + 2)};
}
MediaEntity* findEntity(const std::string& name) {
    for(auto& e : st().entities)
        if(name == e.desc.name) return &e;
    return nullptr;
}
int mediaIoctl(unsigned long req, void* arg) {
    if(req == MEDIA_IOC_ENUM_ENTITIES) {
        auto* d = static_cast<media_entity_desc*>(arg);
        const bool next = d->id & MEDIA_ENT_ID_FLAG_NEXT;
        const uint32_t id = d->id & ~MEDIA_ENT_ID_FLAG_NEXT;
        for(const auto& e : st().entities) {
            if(next ? e.desc.id > id : e.desc.id == id) {
                *d = e.desc;
                return 0;
            }
        }
        return fail(EINVAL);
    }
    if(req == MEDIA_IOC_SETUP_LINK) {
        const auto* l = static_cast<const media_link_desc*>(arg);
        SEEK_LOG(info, "Seek Eagle: SDK media link {}:{} -> {}:{} flags {}", l->source.entity, l->source.index, l->sink.entity, l->sink.index, l->flags);
        return 0;
    }
    SEEK_LOG(warn, "Seek Eagle: unexpected media ioctl 0x{:x}", req);
    return fail(ENOTTY);
}

// seekfpa reset device: drive the reset line like the seekfpa driver does (then hold >= 10 ms / give the core ~1.5 s
// to boot before it is addressed). Level 1 keeps the module's PY32 power/shutter controller running.
int resetIoctl(unsigned long req, void* arg) {
    if(req != kResetIoctl || arg == nullptr) return fail(ENOTTY);
    const uint32_t value = *static_cast<uint32_t*>(arg);
    if(value > 1) return fail(EINVAL);
    auto& s = st();
    {
        std::lock_guard<std::mutex> l(s.m);
        if(s.resetLine == nullptr) {
            s.resetChip = s.resetChipLabel.empty() ? nullptr : gpiod_chip_open_by_label(s.resetChipLabel.c_str());
            gpiod_line* line = s.resetChip ? gpiod_chip_get_line(s.resetChip, s.resetGpio) : nullptr;
            if(line == nullptr || gpiod_line_request_output(line, "seek-eagle-reset", static_cast<int>(value)) < 0) {
                SEEK_LOG(warn, "Seek Eagle: cannot request reset GPIO line {} on {}: {}", s.resetGpio, s.resetChipLabel, strerror(errno));
                if(s.resetChip) gpiod_chip_close(s.resetChip);
                s.resetChip = nullptr;
                return fail(EIO);
            }
            s.resetLine = line;
        } else if(gpiod_line_set_value(s.resetLine, static_cast<int>(value)) < 0) {
            return fail(EIO);
        }
    }
    SEEK_LOG(info, "Seek Eagle: reset line (GPIO{}) -> {}", s.resetGpio, value);
    std::this_thread::sleep_for(std::chrono::milliseconds(value ? 50 : 1500));
    return 0;
}

// ---------------------------------------------------------------------------------------------------------------------
// Replacements for the libseekcamera imports (installed into its GOT, each SDK copy's bound to its Eagle, see hooks()).
// Anything that is not an emulated node goes to the real libc; I2C goes to the real CCI bus.
int hookOpen(const char* path, int flags, mode_t mode) {
    auto& s = st();
    if(isPath(path, kMediaNode) || isPath(path, kResetNode)) {
        const int fd = newNodeFd();
        if(fd < 0) return fd;
        std::lock_guard<std::mutex> l(s.m);
        (isPath(path, kMediaNode) ? s.mediaFds : s.resetFds).push_back(fd);
        return fd;
    }
    const int fd = ::open(path, flags, mode);
    if(fd >= 0 && path != nullptr && s.i2cDevice == path) {
        std::lock_guard<std::mutex> l(s.m);
        s.i2cFds.push_back(fd);
    }
    return fd;
}
// caller holds s.m
void noteI2c(State& s, int fd, size_t readBytes) {
    if(!contains(s.i2cFds, fd)) return;
    s.lastI2c = std::chrono::steady_clock::now();
    s.i2cReadBytes += readBytes;
}
// the clock of camx frame timestamps: the QTimer, as the camera driver converts it
int64_t qtimerNs() {
    uint64_t ticks = 0;
    asm volatile("mrs %0, cntvct_el0" : "=r"(ticks));
    return static_cast<int64_t>(ticks * 10000 / 192);
}
// caller holds s.m: follows the SDK's writes to the core's flash read registers. true if the write ends the
// passthrough: the core then streams imaging frames again before the SDK stops its capture, and the flash capture has to
// be closed before the write goes out (an imaging frame is shorter than the flash mode and camx gives up on it).
bool noteI2cWrite(State& s, int fd, const uint8_t* d, size_t n) {
    if(n < 3 || !contains(s.i2cFds, fd)) return false;
    const uint16_t reg = static_cast<uint16_t>((d[0] << 8) | d[1]);
    bool endsPassthrough = false;
    for(size_t i = 2; i < n; i++) {
        const auto r = static_cast<uint16_t>(reg + i - 2);
        if(r >= kFlashAddrReg && r < kFlashAddrReg + 3) {
            const unsigned shift = 8 * (r - kFlashAddrReg);
            s.flashAddr = (s.flashAddr & ~(0xffu << shift)) | (uint32_t{d[i]} << shift);
        } else if(r == kFlashStartReg && d[i] == 1 && s.flashAddr != s.flashReadAddr) {
            s.flashReadAddr = s.flashAddr;
            s.flashReadStart = qtimerNs();
            s.flashPrev.clear();
            s.flashGood.clear();
        } else if(r == kQspiPassthroughReg && d[i] == 0) {
            endsPassthrough = true;
        }
    }
    return endsPassthrough;
}
ssize_t hookRead(int fd, void* buf, size_t count) {
    const ssize_t n = ::read(fd, buf, count);
    auto& s = st();
    std::lock_guard<std::mutex> l(s.m);
    noteI2c(s, fd, n > 0 ? static_cast<size_t>(n) : 0);
    return n;
}
ssize_t hookWrite(int fd, const void* buf, size_t count) {
    auto& s = st();
    std::unique_lock<std::mutex> l(s.m);
    const bool closeFlash = noteI2cWrite(s, fd, static_cast<const uint8_t*>(buf), count);
    l.unlock();
    if(closeFlash) flashSession().close();
    const ssize_t n = ::write(fd, buf, count);
    l.lock();
    noteI2c(s, fd, 0);
    return n;
}
int hookClose(int fd) {
    auto& s = st();
    bool closeFlash = false;
    {
        std::lock_guard<std::mutex> l(s.m);
        for(auto* v : {&s.mediaFds, &s.resetFds, &s.videoFds, &s.i2cFds}) {
            auto it = std::find(v->begin(), v->end(), fd);
            if(it == v->end()) continue;
            v->erase(it);
            if(fd == s.streamFd) {
                closeFlash = s.flashStreaming;
                stopStreaming(s);
            }
            if(v == &s.videoFds && s.videoFds.empty()) freeBuffers(s);
            break;
        }
    }
    if(closeFlash) flashSession().close();
    return ::close(fd);
}
int hookIoctl(int fd, unsigned long req, void* arg) {
    auto& s = st();
    std::unique_lock<std::mutex> l(s.m);
    if(contains(s.mediaFds, fd)) return mediaIoctl(req, arg);
    if(contains(s.videoFds, fd)) {
        const int r = videoIoctl(s, fd, req, arg);
        const bool closeFlash = std::exchange(s.flashCloseRequested, false);
        l.unlock();
        if(closeFlash) flashSession().close();
        return r;
    }
    if(contains(s.resetFds, fd)) {
        l.unlock();
        return resetIoctl(req, arg);
    }
    size_t readBytes = 0;
    bool closeFlash = false;
    if(req == I2C_RDWR && arg != nullptr) {
        const auto* rdwr = static_cast<const i2c_rdwr_ioctl_data*>(arg);
        for(uint32_t i = 0; i < rdwr->nmsgs; i++) {
            if(rdwr->msgs[i].flags & I2C_M_RD)
                readBytes += rdwr->msgs[i].len;
            else
                closeFlash |= noteI2cWrite(s, fd, rdwr->msgs[i].buf, rdwr->msgs[i].len);
        }
    }
    noteI2c(s, fd, readBytes);
    l.unlock();
    if(closeFlash) flashSession().close();
    return ::ioctl(fd, req, arg);
}
void* hookMmap(void* addr, size_t len, int prot, int flags, int fd, off_t off) {
    auto& s = st();
    {
        std::lock_guard<std::mutex> l(s.m);
        if(contains(s.videoFds, fd)) {
            const size_t idx = s.bufStride ? static_cast<size_t>(off) / s.bufStride : SIZE_MAX;
            if(idx >= s.bufs.size() || static_cast<size_t>(off) % s.bufStride != 0 || len > s.bufStride) {
                errno = EINVAL;
                return MAP_FAILED;
            }
            return s.bufs[idx].mem;
        }
    }
    return ::mmap(addr, len, prot, flags, fd, off);
}
int hookMunmap(void* addr, size_t len) {
    auto& s = st();
    {
        std::lock_guard<std::mutex> l(s.m);
        for(const auto& b : s.bufs)
            if(b.mem == addr) return 0;  // capture buffers belong to the emulated device
    }
    return ::munmap(addr, len);
}
DIR* hookOpendir(const char* path) {
    if(!isPath(path, kDevDir)) return ::opendir(path);
    auto* d = new FakeDir;
    d->entries = {{".", DT_DIR}, {"..", DT_DIR}, {"media0", DT_CHR}, {"video0", DT_CHR}, {"seekfpa-i2c-bus", DT_LNK}, {"seekfpa-reset", DT_CHR}};
    std::lock_guard<std::mutex> l(st().m);
    st().dirs.push_back(d);
    return reinterpret_cast<DIR*>(d);
}
FakeDir* fakeDir(DIR* dir) {
    std::lock_guard<std::mutex> l(st().m);
    auto& v = st().dirs;
    auto it = std::find(v.begin(), v.end(), reinterpret_cast<FakeDir*>(dir));
    return it == v.end() ? nullptr : *it;
}
dirent* hookReaddir(DIR* dir) {
    FakeDir* d = fakeDir(dir);
    if(d == nullptr) return ::readdir(dir);
    if(d->pos >= d->entries.size()) return nullptr;
    const auto& e = d->entries[d->pos++];
    std::memset(&d->ent, 0, sizeof d->ent);
    d->ent.d_ino = 1000 + d->pos;
    d->ent.d_type = e.second;
    std::strncpy(d->ent.d_name, e.first.c_str(), sizeof d->ent.d_name - 1);
    return &d->ent;
}
int hookClosedir(DIR* dir) {
    FakeDir* d = fakeDir(dir);
    if(d == nullptr) return ::closedir(dir);
    {
        std::lock_guard<std::mutex> l(st().m);
        auto& v = st().dirs;
        v.erase(std::find(v.begin(), v.end(), d));
    }
    delete d;
    return 0;
}
int hookXstat(int /*ver*/, const char* path, struct stat* sb) {
    if(isPath(path, kVideoNode)) {
        std::memset(sb, 0, sizeof *sb);
        sb->st_mode = S_IFCHR | 0660;
        sb->st_rdev = makedev(kV4lMajor, kVideoMinor);
        return 0;
    }
    return ::stat(path, sb);  // aarch64: the old __xstat ABI uses the same struct stat
}
int hookAccess(const char* path, int mode) {
    if(isPath(path, kVideoNode) || isPath(path, kMediaNode) || isPath(path, kResetNode)) return 0;
    return ::access(path, mode);
}
ssize_t hookReadlink(const char* path, char* buf, size_t size) {
    if(!isPath(path, kI2cLink)) return ::readlink(path, buf, size);
    const std::string& target = st().i2cDevice;
    const size_t n = std::min(size, target.size());
    std::memcpy(buf, target.data(), n);
    return static_cast<ssize_t>(n);
}
char* hookRealpath(const char* path, char* resolved) {
    const char* target = nullptr;
    if(isPath(path, kI2cLink))
        target = st().i2cDevice.c_str();
    else if(isPath(path, kMediaNode) || isPath(path, kVideoNode) || isPath(path, kResetNode))
        target = path;
    if(target == nullptr) return ::realpath(path, resolved);
    if(resolved == nullptr) return ::strdup(target);
    std::snprintf(resolved, PATH_MAX, "%s", target);
    return resolved;
}
char* hookGetenv(const char* name) {
    if(name != nullptr && std::strcmp(name, "HOME") == 0) return const_cast<char*>(kSeekHome);
    return ::getenv(name);
}
// Each SDK copy maps the DAC sweep file of its own: Eagle i > 0 gets <kDacSweepFile>.<i>
FILE* hookFopen(const char* path, const char* mode) {
    if(tEagle == 0 || !isPath(path, kDacSweepFile)) return ::fopen(path, mode);
    return ::fopen((std::string(kDacSweepFile) + "." + std::to_string(tEagle)).c_str(), mode);
}
// fork() is faked, nothing runs: the emulated capture node gets its format from VIDIOC_S_FMT.
pid_t hookFork() {
    std::lock_guard<std::mutex> l(st().m);
    st().fakePid = 0x3fffff00;
    return st().fakePid;
}
int hookExecl(const char* path, const char* /*arg*/, ...) {
    SEEK_LOG(warn, "Seek Eagle: SDK tried to exec {}", path ? path : "(null)");
    return fail(ENOSYS);
}
pid_t hookWaitpid(pid_t pid, int* status, int options) {
    {
        std::lock_guard<std::mutex> l(st().m);
        if(pid == st().fakePid && pid != 0) {
            if(status) *status = 0;
            return pid;
        }
    }
    return ::waitpid(pid, status, options);
}
int hookFlock(int fd, int op) {
    {
        std::lock_guard<std::mutex> l(st().m);
        if(contains(st().videoFds, fd)) return 0;
    }
    return ::flock(fd, op);
}
int hookV4l2Open(const char* path) {
    if(!isPath(path, kVideoNode)) return fail(ENODEV);
    const int fd = newNodeFd();
    if(fd < 0) return fd;
    std::lock_guard<std::mutex> l(st().m);
    st().videoFds.push_back(fd);
    return fd;
}
int hookV4l2Close(int fd) {
    return hookClose(fd);
}
int hookV4l2Ioctl(int fd, unsigned long req, void* arg) {
    auto& s = st();
    std::unique_lock<std::mutex> l(s.m);
    if(!contains(s.videoFds, fd)) {
        l.unlock();
        return ::ioctl(fd, req, arg);
    }
    const int r = videoIoctl(s, fd, req, arg);
    const bool closeFlash = std::exchange(s.flashCloseRequested, false);
    l.unlock();
    if(closeFlash) flashSession().close();
    return r;
}
int hookV4l2Munmap(void* addr, size_t len) {
    return hookMunmap(addr, len);
}
// libmediactl / libv4l2subdev: the media device handle is opaque to the SDK, entities are MediaEntity objects
char gMediaDevice;
void* hookMediaDeviceNew(const char* devnode) {
    return isPath(devnode, kMediaNode) ? &gMediaDevice : nullptr;
}
int hookMediaDeviceEnumerate(void* media) {
    return media == &gMediaDevice ? 0 : -ENODEV;
}
unsigned hookMediaGetEntitiesCount(void* media) {
    return media == &gMediaDevice ? static_cast<unsigned>(st().entities.size()) : 0;
}
void* hookMediaGetEntity(void* media, unsigned index) {
    auto& entities = st().entities;
    return media == &gMediaDevice && index < entities.size() ? &entities[index] : nullptr;
}
void* hookMediaGetEntityByName(void* media, const char* name) {
    return media == &gMediaDevice && name != nullptr ? findEntity(name) : nullptr;
}
const media_entity_desc* hookMediaEntityGetInfo(void* entity) {
    for(const auto& e : st().entities)
        if(&e == entity) return &e.desc;
    return nullptr;
}
// "'source':pad->'sink':pad[flags]"; both entities must exist, the links are always there in the emulated graph
int hookMediaParseSetupLink(void* media, const char* p, char** endp) {
    if(endp != nullptr) *endp = const_cast<char*>(p);
    const char* end = p != nullptr ? std::strchr(p, ']') : nullptr;
    if(media != &gMediaDevice || end == nullptr) return -EINVAL;
    const std::string link(p, end + 1);
    size_t pos = 0;
    for(int side = 0; side < 2; side++) {
        pos = link.find_first_of("'\"", pos);
        const size_t close = pos == std::string::npos ? pos : link.find(link[pos], pos + 1);
        if(close == std::string::npos || findEntity(link.substr(pos + 1, close - pos - 1)) == nullptr) {
            SEEK_LOG(warn, "Seek Eagle: SDK media link with an unknown entity: {}", link);
            return -ENOENT;
        }
        pos = close + 1;
    }
    if(endp != nullptr) *endp = const_cast<char*>(end + 1);
    SEEK_LOG(info, "Seek Eagle: SDK media link {}", link);
    return 0;
}
void hookMediaDeviceUnref(void* /*media*/) {}
void hookMediaDebugSetHandler(void* /*media*/, void* /*handler*/, void* /*priv*/) {}
int hookSubdevParseSetupFormats(void* media, int /*which*/, const char* formats) {
    if(media != &gMediaDevice) return -ENODEV;
    unsigned width = 0, height = 0;
    const char* size = formats != nullptr ? std::strchr(formats, '/') : nullptr;
    const bool flash = size != nullptr && std::sscanf(size + 1, "%ux%u", &width, &height) == 2 && width == kFlashPassthroughWidth;
    SEEK_LOG(info, "Seek Eagle: SDK sub-device formats {}", formats ? formats : "");
    if(!flash) {
        flashSession().close();
        return 0;
    }
    return flashSession().open(st().cameraId, st().csiPhy) ? 0 : -EINVAL;
}

// The SDK's flash read of one table (v4l2mod_read_flash_region, called through its PLT): tells how much of each frame
// the SDK reads; windows it could not read over MIPI it reads over I2C afterwards (minutes for a large table). The SDK
// gives up on a table after a few frames, sooner than a new camx session delivers its first one and sometimes before two
// frames without a lost row came, so the read waits for the session's frames and is repeated while it is incomplete.
int hookReadFlashRegion(void* ctx, uint32_t addr, uint8_t* out, uint32_t len, void* setup, void* setupArg, uint32_t* windows, uint32_t maxWindows,
                        uint32_t* nWindows) {
    auto& s = st();
    {
        std::unique_lock<std::mutex> l(s.m);
        s.flashReadRows = (len + kFlashWireWidth - 1) / kFlashWireWidth;
        s.cv.wait_for(l, kFlashWarmup, [&s] { return s.flashInAtOpen == UINT64_MAX || s.nFlashIn > s.flashInAtOpen; });
    }
    const auto t0 = std::chrono::steady_clock::now();
    int r = 0, attempts = 0;
    while(true) {
        r = s.sdk.readFlashRegion(ctx, addr, out, len, setup, setupArg, windows, maxWindows, nWindows);
        if(r != 0 || nWindows == nullptr || *nWindows == 0 || ++attempts >= kFlashReadAttempts) break;
        std::lock_guard<std::mutex> l(s.m);
        if(s.flashInAtOpen == UINT64_MAX) break;  // no flash session
    }
    SEEK_LOG(info,
             "Seek Eagle: flash 0x{:06x}: {} bytes read in {} ms ({} repeats), {} of them left for I2C (status {})",
             addr,
             len,
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count(),
             attempts,
             nWindows ? std::min<uint64_t>(uint64_t{*nWindows} * kFlashPassthroughWidth, len) : 0,
             r);
    return r;
}

// Overwrites libseekcamera's PLT slots (R_AARCH64_JUMP_SLOT, lazy binding: still unresolved and writable after dlopen)
size_t patchImports(void* handle, const std::vector<std::pair<const char*, void*>>& hooks, std::vector<std::string>& patched) {
    link_map* lm = nullptr;
    if(dlinfo(handle, RTLD_DI_LINKMAP, &lm) != 0 || lm == nullptr) return 0;
    const ElfW(Addr) base = lm->l_addr;
    auto ptr = [base](ElfW(Addr) v) { return v < base ? v + base : v; };
    const ElfW(Rela)* jmprel = nullptr;
    size_t pltRelSize = 0;
    const ElfW(Sym)* symtab = nullptr;
    const char* strtab = nullptr;
    for(const ElfW(Dyn)* d = lm->l_ld; d->d_tag != DT_NULL; d++) {
        if(d->d_tag == DT_JMPREL) jmprel = reinterpret_cast<const ElfW(Rela)*>(ptr(d->d_un.d_ptr));
        if(d->d_tag == DT_PLTRELSZ) pltRelSize = d->d_un.d_val;
        if(d->d_tag == DT_SYMTAB) symtab = reinterpret_cast<const ElfW(Sym)*>(ptr(d->d_un.d_ptr));
        if(d->d_tag == DT_STRTAB) strtab = reinterpret_cast<const char*>(ptr(d->d_un.d_ptr));
    }
    if(jmprel == nullptr || symtab == nullptr || strtab == nullptr) return 0;
    const uintptr_t page = static_cast<uintptr_t>(getpagesize());
    for(size_t i = 0; i < pltRelSize / sizeof(ElfW(Rela)); i++) {
        const auto& r = jmprel[i];
        if(ELF64_R_TYPE(r.r_info) != R_AARCH64_JUMP_SLOT) continue;
        const char* name = strtab + symtab[ELF64_R_SYM(r.r_info)].st_name;
        for(const auto& h : hooks) {
            if(std::strcmp(name, h.first) != 0) continue;
            auto** slot = reinterpret_cast<void**>(base + r.r_offset);
            const uintptr_t pageStart = reinterpret_cast<uintptr_t>(slot) & ~(page - 1);
            if(mprotect(reinterpret_cast<void*>(pageStart), page, PROT_READ | PROT_WRITE) != 0) return 0;
            *slot = h.second;
            patched.emplace_back(name);
            break;
        }
    }
    return patched.size();
}

// ---------------------------------------------------------------------------------------------------------------------
// Calls the setter `name`(camera, args...) of the current Eagle's SDK copy; the arguments must have the C types of the
// setter (float, uint32_t, bool, int for an enum)
template <typename... A>
void sdkSet(void* camera, const char* name, A... args) {
    auto& s = st();
    auto* fn = reinterpret_cast<int (*)(void*, A...)>(dlsym(s.sdk.handle, name));
    if(fn == nullptr) {
        SEEK_LOG(error, "Seek Eagle: the Seek SDK has no {}", name);
        return;
    }
    const int r = fn(camera, args...);
    if(r != 0) SEEK_LOG(warn, "Seek Eagle: {} failed: {}", name, s.sdk.errorGetStr(r));
}

// The colors of an SDK palette as YUV, BT.601 video range like the SDK's YUY2 frames
void loadPalette(void* camera, int palette) {
    auto& s = st();
    std::array<uint32_t, 256> bgra{};  // seekcamera_color_palette_data_t; the SDK takes exactly its size in bytes
    auto* get = reinterpret_cast<int (*)(void*, int, uint32_t*, size_t)>(dlsym(s.sdk.handle, "seekcamera_get_color_palette_data"));
    const int r = get != nullptr ? get(camera, palette, bgra.data(), sizeof(bgra)) : -1;
    if(r != 0) SEEK_LOG(warn, "Seek Eagle: no colors of palette {}, the linear AGC is gray", palette);
    for(size_t i = 0; i < bgra.size(); i++) {
        const float b = r == 0 ? (bgra[i] & 0xFFU) : i, g = r == 0 ? ((bgra[i] >> 8U) & 0xFFU) : i, red = r == 0 ? ((bgra[i] >> 16U) & 0xFFU) : i;
        s.paletteYuv[i] = {static_cast<uint8_t>(std::lround(16 + (65.481f * red + 128.553f * g + 24.966f * b) / 255)),
                           static_cast<uint8_t>(std::lround(128 + (-37.797f * red - 74.203f * g + 112.0f * b) / 255)),
                           static_cast<uint8_t>(std::lround(128 + (112.0f * red - 93.786f * g - 18.214f * b) / 255))};
    }
}

// Stores or clears the flat scene correction, on a thread of its own: the SDK computes a correction from the frames of
// the running capture session, but saves or deletes one only without it, so the session is restarted. Saved on this
// device only (the Seek directory in $HOME), never written to the core.
void runFsc(int eagle, void* camera, dai::ThermalConfig::ThermalFSCAction action) {
    AsEagle e(eagle);
    auto& s = st();
    const bool store = action == dai::ThermalConfig::ThermalFSCAction::STORE;
    const auto t0 = std::chrono::steady_clock::now();
    if(store) {
        sdkSet(camera, "seekcamera_set_flat_scene_correction_store_mode", kFscStoreHostOnly);
        sdkSet(camera, "seekcamera_store_flat_scene_correction", kFscId, MemoryAccessCallback{nullptr}, static_cast<void*>(nullptr));
    }
    sdkSet(camera, "seekcamera_capture_session_stop");
    if(!store) sdkSet(camera, "seekcamera_delete_flat_scene_correction", kFscId, MemoryAccessCallback{nullptr}, static_cast<void*>(nullptr));
    const int r = s.sdk.captureSessionStart(camera, s.formats);
    // a new capture session sends frames only after a shutter cycle, which the manual shutter mode does not start
    sdkSet(camera, "seekcamera_shutter_trigger");
    std::array<char, 512> path{};
    sdkSet(camera, "seekcamera_get_flat_scene_correction_path", kFscId, path.data(), path.size());
    SEEK_LOG(info,
             "Seek Eagle: flat scene correction {} ({}) in {} ms, capture session restart: {}",
             store ? "stored" : "cleared",
             path.data(),
             std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count(),
             s.sdk.errorGetStr(r));
    s.fscBusy = false;
}

// The set fields of a ThermalConfig -> the SDK; the ThermalConfig enums have the values of the SDK's
void applyConfig(void* camera, const dai::ThermalConfig& c) {
    auto& s = st();
    const auto unsupported = [&s](bool set, const char* field) {
        if(set && s.warnedFields.insert(field).second) SEEK_LOG(warn, "Seek Eagle: ThermalConfig {} is not supported, ignored", field);
    };
    const auto& ambient = c.ambientParams;
    if(ambient.targetEmissivity) sdkSet(camera, "seekcamera_set_scene_emissivity", *ambient.targetEmissivity / 128.0f);
    if(ambient.temperatureOffset) sdkSet(camera, "seekcamera_set_thermography_offset", *ambient.temperatureOffset);
    unsupported(ambient.distance.has_value(), "ambientParams.distance");
    unsupported(ambient.reflectionTemperature.has_value(), "ambientParams.reflectionTemperature");
    unsupported(ambient.atmosphericTemperature.has_value(), "ambientParams.atmosphericTemperature");
    unsupported(ambient.atmosphericTransmittance.has_value(), "ambientParams.atmosphericTransmittance");
    unsupported(ambient.gainMode.has_value(), "ambientParams.gainMode");

    const auto& ffc = c.ffcParams;
    if(ffc.autoFFC) sdkSet(camera, "seekcamera_set_shutter_mode", *ffc.autoFFC ? kShutterModeAuto : kShutterModeManual);
    if(ffc.triggerFFC.value_or(false)) sdkSet(camera, "seekcamera_shutter_trigger");
    unsupported(ffc.minFFCInterval.has_value(), "ffcParams.minFFCInterval");
    unsupported(ffc.maxFFCInterval.has_value(), "ffcParams.maxFFCInterval");
    unsupported(ffc.autoFFCTempThreshold.has_value(), "ffcParams.autoFFCTempThreshold");
    unsupported(ffc.fallProtection.has_value(), "ffcParams.fallProtection");
    unsupported(ffc.minShutterInterval.has_value(), "ffcParams.minShutterInterval");
    unsupported(ffc.closeManualShutter.has_value(), "ffcParams.closeManualShutter");
    unsupported(ffc.antiFallProtectionThresholdHighGainMode.has_value(), "ffcParams.antiFallProtectionThresholdHighGainMode");
    unsupported(ffc.antiFallProtectionThresholdLowGainMode.has_value(), "ffcParams.antiFallProtectionThresholdLowGainMode");

    const auto& image = c.imageParams;
    if(image.colorPalette) {
        sdkSet(camera, "seekcamera_set_color_palette", static_cast<int>(*image.colorPalette));
        loadPalette(camera, static_cast<int>(*image.colorPalette));
    }
    if(image.orientation) {
        const auto o = *image.orientation;
        s.flipH = o == dai::ThermalConfig::Mirror || o == dai::ThermalConfig::MirrorFlip;
        s.flipV = o == dai::ThermalConfig::Flip || o == dai::ThermalConfig::MirrorFlip;
        sdkSet(camera, "seekcamera_set_frame_flip_horizontal", s.flipH);
        sdkSet(camera, "seekcamera_set_frame_flip_vertical", s.flipV);
    }
    if(image.digitalDetailEnhanceLevel)
        sdkSet(camera, "seekcamera_set_filter_state", kFilterSharpen, *image.digitalDetailEnhanceLevel > 0 ? kFilterEnabled : kFilterDisabled);
    if(image.gradientCorrection)
        sdkSet(camera, "seekcamera_set_filter_state", kFilterGradientCorrection, *image.gradientCorrection ? kFilterEnabled : kFilterDisabled);
    if(image.pipelineMode) sdkSet(camera, "seekcamera_set_pipeline_mode", static_cast<int>(*image.pipelineMode));
    unsupported(image.timeNoiseFilterLevel.has_value(), "imageParams.timeNoiseFilterLevel");
    unsupported(image.spatialNoiseFilterLevel.has_value(), "imageParams.spatialNoiseFilterLevel");
    unsupported(image.brightnessLevel.has_value(), "imageParams.brightnessLevel");
    unsupported(image.contrastLevel.has_value(), "imageParams.contrastLevel");

    const auto& agc = c.agcParams;
    if(agc.mode) {
        s.linearAgc = *agc.mode == dai::ThermalConfig::ThermalAgcMode::LINEAR;
        if(!s.linearAgc) sdkSet(camera, "seekcamera_set_agc_mode", static_cast<int>(*agc.mode));
    }
    if(agc.linearLockMode) s.linearLockMode = *agc.linearLockMode;
    if(agc.linearMin) s.linearMin = agc.linearMin;
    if(agc.linearMax) s.linearMax = agc.linearMax;
    const auto& histEq = agc.histEq;
    if(histEq.plateau) sdkSet(camera, "seekcamera_set_histeq_agc_plateau", *histEq.plateau);
    if(histEq.plateauRedistributionMode)
        sdkSet(camera, "seekcamera_set_histeq_agc_plateau_redistribution_mode", static_cast<int>(*histEq.plateauRedistributionMode));
    if(histEq.gainLimit) sdkSet(camera, "seekcamera_set_histeq_agc_gain_limit", *histEq.gainLimit);
    if(histEq.gainLimitFactorMode) sdkSet(camera, "seekcamera_set_histeq_agc_gain_limit_factor_mode", static_cast<int>(*histEq.gainLimitFactorMode));
    if(histEq.gainLimitFactorYMin) sdkSet(camera, "seekcamera_set_histeq_agc_gain_limit_factor_ymin", *histEq.gainLimitFactorYMin);
    if(histEq.gainLimitFactorXMax) sdkSet(camera, "seekcamera_set_histeq_agc_gain_limit_factor_xmax", *histEq.gainLimitFactorXMax);
    if(histEq.alphaTimeSeconds) sdkSet(camera, "seekcamera_set_histeq_agc_alpha_time_seconds", *histEq.alphaTimeSeconds);
    if(histEq.trimLeft) sdkSet(camera, "seekcamera_set_histeq_agc_trim_left", *histEq.trimLeft);
    if(histEq.trimRight) sdkSet(camera, "seekcamera_set_histeq_agc_trim_right", *histEq.trimRight);
    if(histEq.roi) {
        const auto r = histEq.roi->denormalize(static_cast<int>(s.outWidth), static_cast<int>(s.outHeight));
        sdkSet(camera, "seekcamera_set_histeq_agc_roi_left", static_cast<uint32_t>(r.x));
        sdkSet(camera, "seekcamera_set_histeq_agc_roi_top", static_cast<uint32_t>(r.y));
        sdkSet(camera, "seekcamera_set_histeq_agc_roi_width", static_cast<uint32_t>(r.width));
        sdkSet(camera, "seekcamera_set_histeq_agc_roi_height", static_cast<uint32_t>(r.height));
    }
    if(histEq.roiEnable) sdkSet(camera, "seekcamera_set_histeq_agc_roi_enable", *histEq.roiEnable);

    const auto& fsc = c.fscParams;
    if(fsc.enable) sdkSet(camera, "seekcamera_set_filter_state", kFilterFlatSceneCorrection, *fsc.enable ? kFilterEnabled : kFilterDisabled);
    if(fsc.action) {
        if(s.fscBusy.exchange(true)) {
            SEEK_LOG(warn, "Seek Eagle: a flat scene correction is being stored or cleared already, ignored");
        } else {
            std::thread(runFsc, tEagle, camera, *fsc.action).detach();
        }
    }
}

// SDK callbacks; user = the Eagle
void onFrame(void* camera, void* cameraFrame, void* user) {
    AsEagle e(static_cast<int>(reinterpret_cast<intptr_t>(user)));
    auto& s = st();
    std::vector<dai::ThermalConfig> configs;
    Meta meta;
    {
        std::lock_guard<std::mutex> l(s.m);
        std::swap(configs, s.pendingConfigs);
        meta = s.lastDequeued;
        s.nOut++;
    }
    for(const auto& c : configs) applyConfig(camera, c);
    std::lock_guard<std::mutex> p(s.publishMtx);
    if(!s.cb) return;
    // 2 bytes per pixel: temperature FP16 (GRAYF16), color YUY2 (YUV422i), raw counts (RAW16)
    const auto publish = [&](std::vector<uint8_t>&& out, size_t w, size_t h, const char* format, const char* output) {
        const int size = static_cast<int>(out.size());
        s.cb({std::make_shared<dai::VectorMemory>(std::move(out)),
              static_cast<int>(w),
              static_cast<int>(h),
              size,
              static_cast<int>(w * 2),
              static_cast<int>(h),
              std::string(format),
              meta.ts,
              std::string(output),
              meta.cam,
              meta.seq});
    };
    // temperatures in degrees Celsius
    void* frame = nullptr;
    if(s.sdk.frameGetFrameByFormat(cameraFrame, kFormatThermographyFloat, &frame) != 0 || frame == nullptr) return;
    const size_t w = s.sdk.frameGetWidth(frame), h = s.sdk.frameGetHeight(frame);
    const size_t stride = std::max(s.sdk.frameGetLineStride(frame), w * sizeof(float));
    const auto* src = static_cast<const uint8_t*>(s.sdk.frameGetData(frame));
    if(src == nullptr || w == 0 || h == 0 || w % 2 != 0) return;
    s.outWidth = static_cast<uint32_t>(w);
    s.outHeight = static_cast<uint32_t>(h);
    const auto temperatureAt = [&](size_t x, size_t y) {  // x, y of the output frame
        return reinterpret_cast<const float*>(src + (s.flipV ? h - 1 - y : y) * stride)[s.flipH ? w - 1 - x : x];
    };
    if(s.publishTemperature) {
        std::vector<uint8_t> out(w * h * 2);
        auto* dst = reinterpret_cast<__fp16*>(out.data());
        for(size_t y = 0; y < h; y++)
            for(size_t x = 0; x < w; x++) dst[y * w + x] = static_cast<__fp16>(temperatureAt(x, y));
        publish(std::move(out), w, h, "GRAYF16", "temperature");
    }
    // raw counts before the AGC, flipped here like the temperatures
    void* raw = nullptr;
    if(s.publishRaw && s.sdk.frameGetFrameByFormat(cameraFrame, kFormatPreAgc, &raw) == 0 && raw != nullptr && s.sdk.frameGetWidth(raw) == w
       && s.sdk.frameGetHeight(raw) == h && s.sdk.frameGetData(raw) != nullptr) {
        const auto* rsrc = static_cast<const uint8_t*>(s.sdk.frameGetData(raw));
        const size_t rstride = std::max(s.sdk.frameGetLineStride(raw), w * sizeof(uint16_t));
        std::vector<uint8_t> out(w * h * 2);
        auto* dst = reinterpret_cast<uint16_t*>(out.data());
        for(size_t y = 0; y < h; y++) {
            const auto* row = reinterpret_cast<const uint16_t*>(rsrc + (s.flipV ? h - 1 - y : y) * rstride);
            for(size_t x = 0; x < w; x++) dst[y * w + x] = row[s.flipH ? w - 1 - x : x];
        }
        publish(std::move(out), w, h, "RAW16", "raw");
    }
    if(!(s.formats & kFormatColorYuy2)) return;
    std::vector<uint8_t> out(w * h * 2);
    if(s.linearAgc) {
        // the lock range; the frame's minimum or maximum where it is not locked
        using LockMode = dai::ThermalConfig::ThermalLinearAgcLockMode;
        float lo = std::numeric_limits<float>::max(), hi = std::numeric_limits<float>::lowest();
        for(size_t y = 0; y < h; y++) {
            const auto* row = reinterpret_cast<const float*>(src + y * stride);
            const auto [mn, mx] = std::minmax_element(row, row + w);
            lo = std::min(lo, *mn);
            hi = std::max(hi, *mx);
        }
        if((s.linearLockMode == LockMode::MANUAL || s.linearLockMode == LockMode::MANUAL_MIN) && s.linearMin) lo = *s.linearMin;
        if((s.linearLockMode == LockMode::MANUAL || s.linearLockMode == LockMode::MANUAL_MAX) && s.linearMax) hi = *s.linearMax;
        const float scale = 255.0f / std::max(hi - lo, 0.01f);
        const auto level = [&](float t) { return static_cast<size_t>(std::clamp(std::lround((t - lo) * scale), 0L, 255L)); };
        for(size_t y = 0; y < h; y++) {
            for(size_t x = 0; x < w; x += 2) {
                const auto& a = s.paletteYuv[level(temperatureAt(x, y))];
                const auto& b = s.paletteYuv[level(temperatureAt(x + 1, y))];
                uint8_t* d = out.data() + (y * w + x) * 2;
                d[0] = a[0];
                d[1] = static_cast<uint8_t>((a[1] + b[1] + 1) / 2);
                d[2] = b[0];
                d[3] = static_cast<uint8_t>((a[2] + b[2] + 1) / 2);
            }
        }
    } else {
        // the SDK's AGC and palette, flipped by the SDK
        void* color = nullptr;
        if(s.sdk.frameGetFrameByFormat(cameraFrame, kFormatColorYuy2, &color) != 0 || color == nullptr) return;
        const auto* csrc = static_cast<const uint8_t*>(s.sdk.frameGetData(color));
        const size_t cstride = std::max(s.sdk.frameGetLineStride(color), w * 2);
        if(csrc == nullptr || s.sdk.frameGetWidth(color) != w || s.sdk.frameGetHeight(color) != h) return;
        for(size_t y = 0; y < h; y++) std::memcpy(out.data() + y * w * 2, csrc + y * cstride, w * 2);
    }
    publish(std::move(out), w, h, "YUV422i", "color");
}
void onEvent(void* camera, int event, int status, void* user) {
    AsEagle e(static_cast<int>(reinterpret_cast<intptr_t>(user)));
    auto& s = st();
    SEEK_LOG(info, "Seek Eagle: SDK event {} status {}", event, s.sdk.errorGetStr(status));
    if(event != kEventConnect) return;
    uint32_t formats = 0;
    dai::ThermalConfig initialConfig;
    {
        std::lock_guard<std::mutex> l(s.m);
        s.camera = camera;
        formats = s.formats;
        initialConfig = s.initialConfig;
    }
    s.sdk.registerFrameAvailableCallback(camera, onFrame, user);
    // temperatures in degrees Celsius; a grayscale color output (as on the OAK-T) unless the initial config sets a palette
    sdkSet(camera, "seekcamera_set_temperature_unit", kTemperatureUnitCelsius);
    sdkSet(camera, "seekcamera_set_color_palette", kColorPaletteWhiteHot);
    loadPalette(camera, kColorPaletteWhiteHot);
    applyConfig(camera, initialConfig);
    const int r = s.sdk.captureSessionStart(camera, formats);
    SEEK_LOG(info, "Seek Eagle: SDK capture session start (formats {:#x}): {}", formats, s.sdk.errorGetStr(r));
}

// CSIPHY of the Eagle on depthai socket daiSocket: "csi" of that socket in the camera configuration camx uses
// (/etc/luxonis_cameras.json, else the device's entry of /etc/luxonis_cameras_default.json), -1 if not found
int findCsiPhy(int daiSocket) {
    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    const auto socketForCsi = sidechannel != nullptr ? sidechannel->ops.camera_socket_for_csi_get : nullptr;
    for(int csi = 0; socketForCsi != nullptr && csi < LUX_SC_MAX_CAMERA_SOCKETS; ++csi) {
        const auto result = static_cast<lux_sc_camera_socket_for_csi_get_ret_t>(socketForCsi(csi));
        if(result.found && static_cast<int>(result.depthai_socket) == daiSocket) return csi;
    }
    return -1;
}

// Cell `index` (big-endian u32) of a device tree property
std::optional<uint32_t> dtCell(const std::filesystem::path& property, unsigned index = 0) {
    std::ifstream f(property, std::ios::binary);
    unsigned char c[4];
    if(!f.seekg(index * sizeof c) || !f.read(reinterpret_cast<char*>(c), sizeof c)) return std::nullopt;
    return uint32_t{c[0]} << 24 | uint32_t{c[1]} << 16 | uint32_t{c[2]} << 8 | uint32_t{c[3]};
}

// cam-sensor node of the slot (cell-index = csi) under its CCI controller node, empty if none
std::filesystem::path findSensorNode(int csi) {
    std::error_code ec;
    for(const auto& cci : std::filesystem::directory_iterator(kDeviceTreeSoc, ec)) {
        if(csi < 0 || cci.path().filename().string().rfind("qcom,cci", 0) != 0) continue;
        for(const auto& node : std::filesystem::directory_iterator(cci.path(), ec)) {
            if(node.path().filename().string().rfind("qcom,cam-sensor", 0) == 0 && dtCell(node.path() / "cell-index") == static_cast<uint32_t>(csi)) {
                return node.path();
            }
        }
    }
    return {};
}

// I2C adapter of the slot's CCI master: cam-cci<cell-index of the CCI node>-m<cci-master>
std::string findI2cDevice(const std::filesystem::path& node) {
    const auto cci = dtCell(node.parent_path() / "cell-index");
    const auto master = dtCell(node / "cci-master");
    const std::string adapter = cci && master ? "cam-cci" + std::to_string(*cci) + "-m" + std::to_string(*master) : "";
    std::error_code ec;
    for(const auto& e : std::filesystem::directory_iterator("/sys/bus/i2c/devices", ec)) {
        std::ifstream f(e.path() / "name");
        std::string name;
        if(!adapter.empty() && std::getline(f, name) && name == adapter) return "/dev/" + e.path().filename().string();
    }
    return kI2cFallback;
}

// Reset line of the slot: the gpios entry <phandle line flags> that gpio-reset points to. Its controller is the gpiochip
// labelled like the platform device of the phandle's node <name>@<unit address> under /soc, "<unit address>.<name>"
bool findResetGpio(const std::filesystem::path& node, std::string& chipLabel, unsigned& line) {
    const auto index = dtCell(node / "gpio-reset");
    if(!index) return false;
    const auto phandle = dtCell(node / "gpios", *index * 3);
    const auto offset = dtCell(node / "gpios", *index * 3 + 1);
    if(!phandle || !offset) return false;
    std::error_code ec;
    for(const auto& e : std::filesystem::directory_iterator(kDeviceTreeSoc, ec)) {
        const std::string name = e.path().filename().string();
        const auto at = name.find('@');
        if(at != std::string::npos && dtCell(e.path() / "phandle") == *phandle) {
            chipLabel = name.substr(at + 1) + "." + name.substr(0, at);
            line = *offset;
            return true;
        }
    }
    return false;
}

// The hooks of SDK copy I run as Eagle I; variadic imports get trampolines of their own that unpack the arguments
template <int I, auto Hook>
struct AsEagleHook;
template <int I, typename R, typename... A, R (*Hook)(A...)>
struct AsEagleHook<I, Hook> {
    static R call(A... a) {
        AsEagle e(I);
        return Hook(a...);
    }
};
template <int I, auto Hook>
void* bound() {
    return reinterpret_cast<void*>(&AsEagleHook<I, Hook>::call);
}
template <int I>
int openAs(const char* path, int flags, ...) {
    mode_t mode = 0;
    if(flags & (O_CREAT | O_TMPFILE)) {
        va_list ap;
        va_start(ap, flags);
        mode = va_arg(ap, mode_t);
        va_end(ap);
    }
    AsEagle e(I);
    return hookOpen(path, flags, mode);
}
template <int I, int (*Hook)(int, unsigned long, void*)>
int ioctlAs(int fd, unsigned long req, ...) {
    va_list ap;
    va_start(ap, req);
    void* arg = va_arg(ap, void*);
    va_end(ap);
    AsEagle e(I);
    return Hook(fd, req, arg);
}
template <int I>
int v4l2OpenAs(const char* path, int /*flags*/, ...) {
    AsEagle e(I);
    return hookV4l2Open(path);
}
template <int I>
std::vector<std::pair<const char*, void*>> hooksFor() {
    return {
        {"open", reinterpret_cast<void*>(&openAs<I>)},
        {"close", bound<I, hookClose>()},
        {"read", bound<I, hookRead>()},
        {"write", bound<I, hookWrite>()},
        {"ioctl", reinterpret_cast<void*>(&ioctlAs<I, hookIoctl>)},
        {"mmap", bound<I, hookMmap>()},
        {"munmap", bound<I, hookMunmap>()},
        {"opendir", bound<I, hookOpendir>()},
        {"readdir", bound<I, hookReaddir>()},
        {"closedir", bound<I, hookClosedir>()},
        {"__xstat", bound<I, hookXstat>()},
        {"access", bound<I, hookAccess>()},
        {"readlink", bound<I, hookReadlink>()},
        {"realpath", bound<I, hookRealpath>()},
        {"getenv", bound<I, hookGetenv>()},
        {"fopen", bound<I, hookFopen>()},
        {"fork", bound<I, hookFork>()},
        {"execl", reinterpret_cast<void*>(&hookExecl)},  // never runs (fork is faked), so it needs no Eagle
        {"waitpid", bound<I, hookWaitpid>()},
        {"flock", bound<I, hookFlock>()},
        {"v4l2_open", reinterpret_cast<void*>(&v4l2OpenAs<I>)},
        {"v4l2_close", bound<I, hookV4l2Close>()},
        {"v4l2_ioctl", reinterpret_cast<void*>(&ioctlAs<I, hookV4l2Ioctl>)},
        {"v4l2_munmap", bound<I, hookV4l2Munmap>()},
        {"media_device_new", bound<I, hookMediaDeviceNew>()},
        {"media_device_enumerate", bound<I, hookMediaDeviceEnumerate>()},
        {"media_get_entities_count", bound<I, hookMediaGetEntitiesCount>()},
        {"media_get_entity", bound<I, hookMediaGetEntity>()},
        {"media_get_entity_by_name", bound<I, hookMediaGetEntityByName>()},
        {"media_entity_get_info", bound<I, hookMediaEntityGetInfo>()},
        {"media_parse_setup_link", bound<I, hookMediaParseSetupLink>()},
        {"media_device_unref", bound<I, hookMediaDeviceUnref>()},
        {"media_debug_set_handler", bound<I, hookMediaDebugSetHandler>()},
        {"v4l2_subdev_parse_setup_formats", bound<I, hookSubdevParseSetupFormats>()},
        {"v4l2mod_read_flash_region", bound<I, hookReadFlashRegion>()},
    };
}
template <int... I>
std::array<std::vector<std::pair<const char*, void*>>, kMaxEagles> hooksForAll(std::integer_sequence<int, I...>) {
    return {{hooksFor<I>()...}};
}
// imports of SDK copy `eagle` and their replacements
const std::vector<std::pair<const char*, void*>>& hooks(int eagle) {
    static const auto all = hooksForAll(std::make_integer_sequence<int, kMaxEagles>{});
    return all[eagle];
}

bool load() {
    auto& s = st();
    std::error_code ec;
    const auto dir = std::filesystem::read_symlink("/proc/self/exe", ec).parent_path() / kPackageSubdir;
    s.csiPhy = findCsiPhy(s.socket);
    const auto node = findSensorNode(s.csiPhy);
    s.i2cDevice = findI2cDevice(node);
    s.entities = makeMediaGraph(s.i2cDevice);
    if(!findResetGpio(node, s.resetChipLabel, s.resetGpio)) s.logger->error("Seek Eagle: no reset line for CSI {} in the device tree", s.csiPhy);
    for(const char* stub : kStubLibs) {
        if(dlopen((dir / stub).c_str(), RTLD_NOW | RTLD_GLOBAL) == nullptr) {
            s.logger->error("Seek Eagle: cannot load {}: {}", (dir / stub).string(), dlerror());
            return false;
        }
    }
    auto lib = dir / kSdkLib;
    if(tEagle > 0) {
        const auto copy = dir / (std::string(kSdkLib) + "." + std::to_string(tEagle));
        std::filesystem::copy_file(lib, copy, std::filesystem::copy_options::overwrite_existing, ec);
        if(ec) {
            s.logger->error("Seek Eagle: cannot copy the Seek SDK to {}: {}", copy.string(), ec.message());
            return false;
        }
        lib = copy;
    }
    void* h = dlopen(lib.c_str(), RTLD_LAZY | RTLD_LOCAL);
    if(h == nullptr) {
        s.logger->error("Seek Eagle: cannot load the Seek SDK {}: {}", lib.string(), dlerror());
        return false;
    }
    std::vector<std::string> patched;
    if(patchImports(h, hooks(tEagle), patched) != hooks(tEagle).size()) {
        s.logger->error("Seek Eagle: patched {} of {} Seek SDK imports", patched.size(), hooks(tEagle).size());
        return false;
    }
    auto sym = [h](auto& fn, const char* name) {
        fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(dlsym(h, name));
        return fn != nullptr;
    };
    auto& a = s.sdk;
    a.handle = h;
    if(!sym(a.managerCreate, "seekcamera_manager_create") || !sym(a.managerRegisterEventCallback, "seekcamera_manager_register_event_callback")
       || !sym(a.registerFrameAvailableCallback, "seekcamera_register_frame_available_callback")
       || !sym(a.captureSessionStart, "seekcamera_capture_session_start") || !sym(a.frameGetFrameByFormat, "seekcamera_frame_get_frame_by_format")
       || !sym(a.frameGetWidth, "seekframe_get_width") || !sym(a.frameGetHeight, "seekframe_get_height")
       || !sym(a.frameGetLineStride, "seekframe_get_line_stride") || !sym(a.frameGetData, "seekframe_get_data")
       || !sym(a.errorGetStr, "seekcamera_error_get_str") || !sym(a.readFlashRegion, "v4l2mod_read_flash_region")) {
        s.logger->error("Seek Eagle: Seek SDK symbols missing: {}", dlerror());
        return false;
    }
    s.logger->info("Seek Eagle: Seek SDK loaded from {}, {} imports patched, socket {}, CSI {}, I2C {}, reset {} line {}",
                   lib.string(),
                   patched.size(),
                   s.socket,
                   s.csiPhy,
                   s.i2cDevice,
                   s.resetChipLabel,
                   s.resetGpio);
    const int r = a.managerCreate(&s.manager, kIoTypeVideo);
    if(r != 0) {
        s.logger->error("Seek Eagle: seekcamera_manager_create failed: {}", a.errorGetStr(r));
        return false;
    }
    a.managerRegisterEventCallback(s.manager, onEvent, reinterpret_cast<void*>(static_cast<intptr_t>(tEagle)));
    return true;
}

}  // namespace

namespace {
bool waitForImaging(std::shared_ptr<spdlog::async_logger> logger, int timeoutMs, int cameraId) {
    auto& s = st();
    std::unique_lock<std::mutex> l(s.m);
    if(!s.logger) s.logger = std::move(logger);
    s.cameraId = cameraId;
    if(!s.loaded && !s.failed) {
        l.unlock();
        const bool ok = load();
        l.lock();
        (ok ? s.loaded : s.failed) = true;
    }
    if(s.failed) return false;
    // While the SDK calibrates a new core it keeps reading from it over I2C: wait as long as that traffic goes on
    using std::chrono::duration_cast;
    using std::chrono::seconds;
    const auto idle = std::chrono::milliseconds(timeoutMs);
    const auto t0 = std::chrono::steady_clock::now();
    auto nextLog = t0 + kCalibrationLogInterval;
    while(!s.streaming) {
        const auto now = std::chrono::steady_clock::now();
        const auto lastActivity = std::max(s.lastI2c, t0);
        if(now - lastActivity >= idle || now - t0 >= kStartMaxWait) {
            s.logger->error("Seek Eagle: the Seek SDK did not start capturing ({} s, {} KB read over I2C, none in the last {} s)",
                            duration_cast<seconds>(now - t0).count(),
                            s.i2cReadBytes / 1024,
                            duration_cast<seconds>(now - lastActivity).count());
            return false;
        }
        if(now >= nextLog) {
            s.logger->warn("Seek Eagle: the Seek SDK is calibrating the core ({} KB read over I2C in {} s); a core's first calibration takes about 15 minutes",
                           s.i2cReadBytes / 1024,
                           duration_cast<seconds>(now - t0).count());
            nextLog = now + kCalibrationLogInterval;
        }
        s.cv.wait_until(l, std::min({lastActivity + idle, t0 + kStartMaxWait, nextLog}));
    }
    return true;
}
}  // namespace

bool start(std::shared_ptr<spdlog::async_logger> logger, int timeoutMs, int cameraId, CameraBoardSocket socket) {
    const int eagle = eagleOf(socket);
    if(eagle < 0) return false;  // more Eagles than kMaxEagles
    AsEagle e(eagle);
    const bool ok = waitForImaging(std::move(logger), timeoutMs, cameraId);
    flashSession().close();  // normally closed already, when the SDK ended the passthrough
    return ok;
}

void attach(CameraBoardSocket socket, CameraSensor::FrameCallback cb) {
    const int eagle = eagleOf(socket);
    if(eagle < 0) return;
    AsEagle e(eagle);
    auto& s = st();
    std::lock_guard<std::mutex> p(s.publishMtx);
    s.cb = std::move(cb);
    if(!s.cb && s.logger) {
        std::lock_guard<std::mutex> l(s.m);
        s.logger->info("Seek Eagle: {} wire frames in, {} without a free SDK buffer, {} SDK frames out", s.nWire, s.nNoBuffer, s.nOut);
    }
}

void submitWireFrame(CameraBoardSocket socket,
                     const uint8_t* data,
                     uint32_t width,
                     uint32_t height,
                     int stride,
                     const dai::Timestamp& ts,
                     uint64_t seq,
                     const dai::ImgFrame::CameraSettings& cam,
                     const std::string& output) {
    const int eagle = eagleOf(socket);
    if(eagle < 0) return;
    AsEagle e(eagle);
    auto& s = st();
    std::lock_guard<std::mutex> l(s.m);
    s.nWire++;
    if(!s.streaming || s.queued.empty()) {
        s.nNoBuffer++;
        return;
    }
    fillBuffer(s, data, width, height, stride, {ts, seq, cam, output});
}

void setup(CameraBoardSocket socket, bool temperature, bool color, bool raw, const dai::ThermalConfig& initialConfig) {
    const int eagle = eagleOf(socket);
    if(eagle < 0) return;
    AsEagle e(eagle);
    auto& s = st();
    std::lock_guard<std::mutex> l(s.m);
    // the temperature frames are needed for the color output too (linear AGC)
    const uint32_t formats = kFormatThermographyFloat | (color ? kFormatColorYuy2 : 0) | (raw ? kFormatPreAgc : 0);
    if(s.camera == nullptr) {
        s.formats = formats;
        s.publishTemperature = temperature;
        s.publishRaw = raw;
        s.initialConfig = initialConfig;
        return;
    }
    // the SDK of this process runs its capture session already: only the settings can change
    if(formats != s.formats) SEEK_LOG(warn, "Seek Eagle: the Seek SDK streams formats {:#x} already, {:#x} not set", s.formats, formats);
    s.publishTemperature = temperature;
    s.publishRaw = raw && (s.formats & kFormatPreAgc);
    s.pendingConfigs.push_back(initialConfig);
}

void configure(CameraBoardSocket socket, const dai::ThermalConfig& config) {
    const int eagle = eagleOf(socket);
    if(eagle < 0) return;
    AsEagle e(eagle);
    std::lock_guard<std::mutex> l(st().m);
    st().pendingConfigs.push_back(config);
}

unsigned long clearLenientRx() {
    unsigned long phys = 0;
    std::ifstream(kLenientRxParam) >> phys;
    if(phys != 0) std::ofstream(kLenientRxParam) << 0 << "\n";
    return phys;
}

}  // namespace seek_eagle
}  // namespace gate
}  // namespace dai
