#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "CameraSensor.hpp"
#include "depthai/pipeline/datatype/ThermalConfig.hpp"

namespace dai {
namespace gate {

// Seek Eagle thermal core: the module only sends raw wire frames over MIPI (328x248 YUYV transport, 656-byte rows);
// calibration, NUC/FFC, AGC and the color palette are done by the Seek SDK. The SDK (libseekcamera.so.5.2 from the
// firmware package) runs inside this process: after dlopen() its import table is patched so that the devices it
// expects on a Raspberry Pi 5 (media graph "sefpa <bus>-0010" -> "csi2" -> "rp1-cfe-csi2_ch0", the V4L2 capture node,
// the seekfpa reset device, the I2C bus symlink) are emulated here, fed with the camx raw frames. I2C goes to the real
// CCI bus, the reset line is driven on the TLMM GPIO directly. Each Eagle (depthai socket) runs an SDK copy of its own.
namespace seek_eagle {

// The outputs of the socket's Eagle (temperature, color, raw) and the settings it starts with; called before start()
void setup(CameraBoardSocket socket, bool temperature, bool color, bool raw, const dai::ThermalConfig& initialConfig);

// Loads and patches the SDK copy of the socket's Eagle (once per process) and starts it: reset, identification, imaging
// start. Blocks until the SDK streams from the emulated capture device. A core the SDK has not calibrated before is
// calibrated first: the SDK reads its tables as flash passthrough frames, captured by a camx session of their own on
// camera cameraId (seconds), and over I2C where that fails (about 15 minutes). The wait goes on while the SDK talks to the
// core and ends after timeoutMs without that traffic (30 minutes at most). false if the SDK is missing or does not start.
bool start(std::shared_ptr<spdlog::async_logger> logger, int timeoutMs, int cameraId, CameraBoardSocket socket);

// The SDK's frames are published through cb (nullptr detaches): output "temperature" (GRAYF16, degrees Celsius), "color"
// (YUV422i) and "raw" (RAW16 counts before the AGC), 320x240, all with the sequence number and timestamp of the wire frame
void attach(CameraBoardSocket socket, CameraSensor::FrameCallback cb);

// camx raw wire frame -> emulated V4L2 capture device of the socket's SDK
void submitWireFrame(CameraBoardSocket socket,
                     const uint8_t* data,
                     uint32_t width,
                     uint32_t height,
                     int stride,
                     const dai::Timestamp& ts,
                     uint64_t seq,
                     const dai::ImgFrame::CameraSettings& cam,
                     const std::string& output);

// The set fields of config, applied before the next SDK frame
void configure(CameraBoardSocket socket, const dai::ThermalConfig& config);

// Clears the camera driver's lenient CSIPHY mask (lux_lenient_rx_phys), which a firmware that died during a flash capture
// session can leave set; called before camx loads, when no flash session is open. Returns the mask it cleared, 0 if none.
unsigned long clearLenientRx();

}  // namespace seek_eagle
}  // namespace gate
}  // namespace dai
