#pragma once
// Public helpers for the C interface of the vendor module (dai_device_*.h): readable errors, the log of the vendor
// module, and the conversion of depthai types to the types of the interface. Not part of the interface itself.

#include <cstdint>
#include <stdexcept>
#include <string>

#include "depthai/common/Interpolation.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "vendor/dai_device_common.h"

namespace dai {

class AcceleratorMemory;

namespace device {

/// A failed call of the C interface of the vendor module
class DeviceError : public std::runtime_error {
   public:
    DeviceError(dai_status status, std::string operation, const std::string& message);
    dai_status status() const noexcept {
        return statusValue;
    }
    /// The C function, for example "dai_device_stereo_init"
    const std::string& operation() const noexcept {
        return operationName;
    }

   private:
    dai_status statusValue;
    std::string operationName;
};

/// Text of a status: "invalid argument", "driver status 7"
std::string describeStatus(dai_status status);

/**
 * Throws a DeviceError for a status that is not DAI_STATUS_OK, with the text
 * "<unit>: <function> failed: <status>: <text of dai_device_last_error()>". An internal error also gets the commit
 * and the version, as DAI_CHECK_IN. `call` is the call as text: the function name is the part before '('. `detail`
 * replaces the text of dai_device_last_error() when it is not NULL (the DSP offload has no error text).
 * DAI_CHECK_DEVICE (utilities/ErrorMacros.hpp) calls it.
 */
[[noreturn]] void throwDeviceError(dai_status status, const char* unit, const char* call, const char* detail);

/// For the methods that return the driver status, as before the C interface: throws for status < 0 and returns a
/// driver status (> 0) or DAI_STATUS_OK
dai_status returnDriverStatus(dai_status status, const char* unit, const char* call);

/// Sends the log of the vendor module and of the DSP offload to the system logger and checks the interface version.
/// Each wrapper calls it; only the first call does the work.
void installDeviceLog();

/// Opens a session of the DSP offload (open is dai_device_dsp_open_stereo or dai_device_dsp_open_align; call is its name)
/// after installDeviceLog(). Throws a DeviceError "DSP offload: <call> failed: ...: DSP open failed" when it fails. The
/// C library keeps the session, so more calls do nothing; the nodes call it early, before the hardware units start.
void openDspSession(dai_status (*open)(void), const char* call);

/// DAI_FRAME_FORMAT_* of a frame type; DAI_FRAME_FORMAT_UNKNOWN for the types that the accelerators do not take
uint32_t toDeviceFormat(ImgFrame::Type type);

/// DAI_INTERPOLATION_* of an interpolation; throws "Unsupported interpolation type: <n>" for the others (the hardware
/// has only bilinear and bicubic)
uint32_t toDeviceInterpolation(Interpolation interpolation);

/// The image of a frame in `memory` (the accelerator buffer of the frame data)
dai_device_image toDeviceImage(const ImgFrame& frame, const AcceleratorMemory& memory);

/// The buffer view for the DSP offload: data() and getMaxSize() of the buffer, as before the C interface
dai_device_buf toDeviceBuf(const AcceleratorMemory& memory);

}  // namespace device
}  // namespace dai
