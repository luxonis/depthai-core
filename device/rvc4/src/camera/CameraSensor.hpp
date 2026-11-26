#pragma once
#include <camera/CameraMetadata.h>
#include <gstreamer-1.0/gst/app/gstappsink.h>
#include <gstreamer-1.0/gst/gst.h>
#include <hardware/camera3.h>
#include <luxonis_sidechannel_data.h>
#include <spdlog/async_logger.h>
#include <stdint.h>

#include <array>
#include <atomic>
#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/common/CameraImageOrientation.hpp>
#include <depthai/common/Timestamp.hpp>
#include <depthai/properties/CameraProperties.hpp>
#include <depthai/utility/Memory.hpp>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <tuple>

#include "camera/CameraPool.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "utilities/CameraDiscovery.hpp"

namespace dai {
namespace gate {

class CameraSensor : public camera3_callback_ops_t {
   public:
    struct FrameCallbackArgs {
        std::shared_ptr<dai::Memory> data;
        int width;
        int height;
        int size;
        int wstride;
        int hstride;
        std::string format;
        dai::Timestamp ts;
        std::string output;
        const dai::ImgFrame::CameraSettings& cam;
        uint64_t sequenceNumber;
    };
    using FrameCallback = std::function<void(FrameCallbackArgs)>;
    using SensorErrorStateCallback = std::function<void(int)>;

    // If FSYNC_MODE == SLAVE then the fps has to be exactly something that the stm can output!!! Especially on ov9282!!!
    CameraSensor(const CameraDescription cam,
                 const size_t sensorModeIndex,
                 float wantedFps,
                 lux_sc_camera_fsync_t fsyncMode,
                 const dai::CameraImageOrientation orientation,
                 dai::CameraProperties properties,
                 std::optional<EepromData> eepromData,
                 FrameCallback frameCb,
                 std::shared_ptr<spdlog::async_logger> pLogger);

    ~CameraSensor();

    // Has to be called before build()
    void setupDynamicOutputs(std::vector<std::tuple<uint32_t, uint32_t, float, size_t>>& specs);

    // Has to be called before build()
    void setupRawOutput();

    // Sets a custom camera tuning file to be applied instead of the default one.
    // Has to be called before build()
    void setTuningOverride(const std::string& tuningPath);

    // Has to be called before run()
    void build(dai::CameraSensorType sensorType);

    // This blocks the calling thread until stop() is called.
    void run();

    // Stops all streams and tears down the resources.
    // Can only be called once.
    // This makes the run() function return / unblocks the calling thread.
    void stop();

    void applyCameraControl(dai::CameraControl control);
    void* sensorErrorCbAdd(SensorErrorStateCallback cb);
    void sensorErrorCbRemove(void* opaqueHandle);

   private:
    struct CallbackArgs {
        CameraSensor* instance;
        int outputSpecsIndex;
    };
    struct MetaCacheItem {
        uint64_t sequenceNumber;
        uint64_t timestamp;
        uint64_t exposureTime;
        float lensPosition;
        int32_t correlatedColorTemperature;
        std::optional<float> sensorTemperatureC{std::nullopt};
        int triggeredCount{0};
        int32_t iso;
        bool hasSettings{false};  // Exposure, ISO, lens position and colour temperature are known.
    };

    // Create a mutex to serialize start commands
    static std::mutex buildMtx;
    static void processCaptureResultCb(const struct camera3_callback_ops* ops, const camera3_capture_result_t* result);
    static void notifyCb(const struct camera3_callback_ops* ops, const camera3_notify_msg_t* msg);
    static camera3_buffer_request_status_t requestStreamBuffersCb(const struct camera3_callback_ops* ops,
                                                                  uint32_t num_buffer_reqs,
                                                                  const camera3_buffer_request_t* buffer_reqs,
                                                                  /*out*/ uint32_t* num_returned_buf_reqs,
                                                                  /*out*/ camera3_stream_buffer_ret_t* returned_buf_reqs);
    static void returnStreamBuffersCb(const struct camera3_callback_ops* ops, uint32_t num_buffers, const camera3_stream_buffer_t* const* buffers);

    bool onlyOnce{true};
    int cameraId = -1;
    bool hasRaw{false};
    bool isToFSensor{false};
    std::shared_ptr<spdlog::async_logger> logger;
    size_t sensorMode;
    // AF and AE regions are clamped to the active array, read once the camera is opened.
    std::optional<std::pair<int32_t, int32_t>> activeArraySize;
    mutable bool warnedRegionClamped{false};
    bool warnedBrightnessUnsupported{false};
    std::optional<std::pair<int32_t, int32_t>> readActiveArraySize() const;
    std::array<int32_t, 5> toCamxRegion(const dai::CameraControl::RegionParams& region, const char* name) const;
    int stride_align = 64;
    int scanline_align = 64;
    bool isEvenFocusApply{false};
    std::vector<std::tuple<uint32_t, uint32_t, float, size_t>> outputSpecs;
    std::mutex metaCacheMutex;
    std::deque<MetaCacheItem> metaCache;
    std::map<std::string, uint64_t> metaCacheLast;
    // Frames per CamX batch in a high-frame-rate session, 1 otherwise. Only the batch's last frame carries the settings.
    uint32_t hfrBatchSize{1};
    uint32_t readHfrBatchSize(uint32_t width, uint32_t height) const;
    std::vector<std::unique_ptr<CallbackArgs>> callbackArgs;
    dai::CameraControl initialControl;
    std::vector<std::string> queues;
    FrameCallback handleBuffer;
    struct SensorErrorCallbackEntry {
        SensorErrorStateCallback cb;
    };
    std::mutex sensorErrorCallbacksLock;
    std::vector<std::shared_ptr<SensorErrorCallbackEntry>> sensorErrorCallbacks;
    std::shared_ptr<CameraPool> cameraPool;
    std::optional<int> numFramesPoolOutputs;
    std::optional<int> maxSizePoolOutputs;
    std::optional<int32_t> explicitIspRoute;
    std::optional<EepromData> eepromData;
    std::optional<std::string> tuningOverride;
    CameraDescription cameraDescription;
    std::atomic<bool> streamEnabled{true};
    std::atomic<bool> sensorTemperatureRead{false};

    std::optional<MetaCacheItem> popMetaCache(uint64_t sequenceNumber, const std::string& output);
    void processCaptureResult(const camera3_capture_result_t* result);
    void handleNotify(const camera3_notify_msg_t* msg);
    std::mutex streamsToCallbacksLock;
    std::map<camera3_stream_t*, std::function<void(const camera3_stream_buffer_t*, const camera3_capture_result_t*)>> streamsToCallbacks;
    camera3_device_t* mDevice{nullptr};
    camera3_stream_configuration_t curStreamsConfig{};
    std::vector<camera3_stream_t> curStreams;
    android::hardware::camera::common::V1_0::helper::CameraMetadata sessionParameters{};
    android::hardware::camera::common::V1_0::helper::CameraMetadata curSettings{};
    std::mutex curSettingsNextLock;
    bool curSettingsNextChanged{false};
    android::hardware::camera::common::V1_0::helper::CameraMetadata curSettingsNext{};
    const camera_metadata_t* curSettingsLastLocked{nullptr};
    camera3_capture_request_t curRequest{};
    bool isStereoSensor;
    bool firstControlApply{true};
    bool hfrModeEnabled{false};
    bool highFpsMode{false};
    std::thread camThread;
    std::thread sensorErrorMonitorThread;
    float curFps{0.0f};
    lux_sc_camera_fsync_t fsyncMode{LUX_SC_CAMERA_FSYNC_NONE};
    std::atomic<bool> sensorErrorMonitorRunning{false};
    std::atomic<int> sensorErrorGpioState{0};
    void* sensorErrorCbHandle{nullptr};
    static int shouldSigAbort(int32_t depthaiSocket, void* userData);
    void startSensorErrorMonitor();
    void stopSensorErrorMonitor();
    void registerSensorErrorCallback();
    void unregisterSensorErrorCallback();
    void notifySensorErrorStateChanged(int state);
    void sensorErrorMonitorLoop();
    void cameraLoop();
    void addStream(std::vector<camera3_stream_t*>& curStreamsPtrs, const std::string& outputName, int format, uint32_t width, uint32_t height);
    void applyTuningFile(const std::string& tuningPath);
    void loadCorrectTuning();
};

}  // namespace gate
}  // namespace dai
