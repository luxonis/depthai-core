#include "vendor/DeviceError.hpp"

#include <spdlog/spdlog.h>

#include <mutex>

#include "build/version.hpp"
#include "pipeline/PipelineBuilder.hpp"
#include "utilities/EvaDataMemory.hpp"
#include "vendor/dai_device_dsp.h"

namespace dai {
namespace device {

namespace {

// A reference to the version symbol of the vendor module: a vendor module of another interface version gives the
// link error "undefined reference to dai_device_abi_<version>"
[[maybe_unused]] const volatile uint32_t* const abiSymbol = &DAI_DEVICE_ABI_SYMBOL;

std::string functionName(const char* call) {
    std::string name = call != nullptr ? call : "";
    const auto paren = name.find('(');
    if(paren != std::string::npos) name.resize(paren);
    while(!name.empty() && name.back() == ' ') name.pop_back();
    return name;
}

spdlog::level::level_enum toSpdlogLevel(int32_t level) {
    switch(level) {
        case DAI_LOG_LEVEL_TRACE:
            return spdlog::level::trace;
        case DAI_LOG_LEVEL_DEBUG:
            return spdlog::level::debug;
        case DAI_LOG_LEVEL_INFO:
            return spdlog::level::info;
        case DAI_LOG_LEVEL_WARN:
            return spdlog::level::warn;
        default:
            return spdlog::level::err;
    }
}

// The log callback of the vendor module and of the DSP offload; user is the name of the source
void forwardLog(void* user, int32_t level, const char* message) noexcept {
    try {
        auto logger = gate::PipelineBuilder::getInstance().getSystemLogger();
        if(logger) {
            logger->log(toSpdlogLevel(level), "[{}] {}", static_cast<const char*>(user), message);
        }
    } catch(...) {
        // The log must not throw into the vendor module
    }
}

}  // namespace

DeviceError::DeviceError(dai_status status, std::string operation, const std::string& message)
    : std::runtime_error(message), statusValue(status), operationName(std::move(operation)) {}

std::string describeStatus(dai_status status) {
    if(status > 0) {
        return "driver status " + std::to_string(status);
    }
    return dai_status_name(status);
}

void throwDeviceError(dai_status status, const char* unit, const char* call, const char* detail) {
    const std::string operation = functionName(call);
    const char* text = detail != nullptr ? detail : dai_device_last_error();
    std::string message = std::string(unit != nullptr ? unit : "Device") + ": " + operation + " failed: " + describeStatus(status);
    if(text != nullptr && text[0] != '\0') {
        message += ": ";
        message += text;
    }
    if(status == DAI_STATUS_INTERNAL) {
        message += ". Internal error occured. Please report. commit: ";
        message += dai::device::build::COMMIT;
        message += " | version: ";
        message += dai::device::build::VERSION;
    }
    throw DeviceError(status, operation, message);
}

dai_status returnDriverStatus(dai_status status, const char* unit, const char* call) {
    if(status < 0) {
        throwDeviceError(status, unit, call, nullptr);
    }
    return status;
}

void installDeviceLog() {
    static std::once_flag once;
    std::call_once(once, [] {
        if(dai_device_abi_version() != DAI_DEVICE_ABI_VERSION) {
            throw std::runtime_error("The vendor module has the interface version " + std::to_string(dai_device_abi_version()) + ", this firmware needs "
                                     + std::to_string(DAI_DEVICE_ABI_VERSION));
        }
        dai_device_set_log(&forwardLog, const_cast<char*>("vendor"), DAI_LOG_LEVEL_TRACE);
        dai_device_dsp_set_log(&forwardLog, const_cast<char*>("dsp"), DAI_LOG_LEVEL_TRACE);
    });
}

void openDspSession(dai_status (*open)(void), const char* call) {
    installDeviceLog();
    const dai_status status = open();
    if(status != DAI_STATUS_OK) {
        throwDeviceError(status, "DSP offload", call, "DSP open failed");
    }
}

uint32_t toDeviceFormat(ImgFrame::Type type) {
    switch(type) {
        case ImgFrame::Type::GRAY8:
            return DAI_FRAME_FORMAT_GRAY8;
        case ImgFrame::Type::RAW8:
            return DAI_FRAME_FORMAT_RAW8;
        case ImgFrame::Type::NV12:
            return DAI_FRAME_FORMAT_NV12;
        case ImgFrame::Type::YUV420p:
            return DAI_FRAME_FORMAT_YUV420P;
        default:
            return DAI_FRAME_FORMAT_UNKNOWN;
    }
}

uint32_t toDeviceInterpolation(Interpolation interpolation) {
    switch(interpolation) {
        case Interpolation::BILINEAR:
            return DAI_INTERPOLATION_BILINEAR;
        case Interpolation::BICUBIC:
            return DAI_INTERPOLATION_BICUBIC;
        default:
            throw std::runtime_error("Unsupported interpolation type: " + std::to_string(static_cast<int>(interpolation)));
    }
}

dai_device_image toDeviceImage(const ImgFrame& frame, const EvaDataMemory& memory) {
    dai_device_image image{};
    image.mem = memory.handle();
    image.width = frame.getWidth();
    image.height = frame.getHeight();
    image.stride = frame.getStride();
    image.format = toDeviceFormat(frame.getType());
    image.p1_offset = frame.fb.p1Offset;
    image.p2_offset = frame.fb.p2Offset;
    image.p3_offset = frame.fb.p3Offset;
    return image;
}

dai_device_buf toDeviceBuf(const EvaDataMemory& memory) {
    dai_device_buf buf{};
    buf.fd = memory.fd();
    buf.data = const_cast<uint8_t*>(memory.data());
    buf.size = memory.getMaxSize();
    return buf;
}

}  // namespace device
}  // namespace dai
