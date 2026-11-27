#include "utilities/AuxConnector.hpp"

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iterator>
#include <system_error>

namespace dai {
namespace aux {

namespace {

std::optional<std::string> readDeviceTreeString(const std::filesystem::path& property) {
    std::ifstream file(property, std::ios::binary);
    if(!file) {
        return std::nullopt;
    }
    std::string value{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
    // Device tree strings end with a NUL
    return std::string(value.c_str());
}

// A kernel string that fills its whole array has no NUL
template <std::size_t N>
std::string fixedString(const char (&chars)[N]) {
    return std::string(chars, strnlen(chars, N));
}

struct FileDescriptor {
    int fd;
    explicit FileDescriptor(int fd) : fd(fd) {}
    FileDescriptor(const FileDescriptor&) = delete;
    FileDescriptor& operator=(const FileDescriptor&) = delete;
    ~FileDescriptor() {
        if(fd >= 0) {
            close(fd);
        }
    }
};

}  // namespace

bool fsyncLinkedToConnector(const std::filesystem::path& deviceTree) {
    auto node = readDeviceTreeString(deviceTree / "aliases/stm_fsync");
    if(!node || node->empty()) {
        return false;
    }
    std::error_code ec;
    if(!std::filesystem::exists(deviceTree / std::filesystem::path(*node).relative_path() / "luxonis,aux-connector", ec)) {
        return false;
    }
    auto status = readDeviceTreeString(deviceTree / "aux-connector/status");
    return !status || *status == "okay" || *status == "ok";
}

bool connectorBound(const std::filesystem::path& device) {
    int fd = open(device.c_str(), O_RDONLY | O_CLOEXEC);
    if(fd < 0) {
        return false;
    }
    close(fd);
    return true;
}

Pin parsePin(const aux_pin_info& info, const aux_pin_state& state) {
    Pin pin{info.pin, fixedString(info.label), {}, std::nullopt, {NoOwner{}, (state.flags & AUX_PIN_FLAG_LINE) != 0}};
    for(uint32_t i = 0; i < std::min<uint32_t>(info.nfuncs, AUX_MAX_PIN_FUNCS); i++) {
        pin.functions.push_back(info.funcs[i].function);
    }
    if(info.gpio.chip[0] != '\0') {
        pin.gpio = GpioLineRef{fixedString(info.gpio.chip), info.gpio.offset};
    }
    switch(state.owner.type) {
        case AUX_OWNER_ME:
        case AUX_OWNER_OTHER:
            pin.state.owner = ProcessOwner{static_cast<pid_t>(state.owner.pid)};
            break;
        case AUX_OWNER_KERNEL:
            pin.state.owner = KernelOwner{fixedString(state.owner.name)};
            break;
        default:
            break;
    }
    return pin;
}

std::optional<Pin> findPinWithFunction(const std::vector<Pin>& pins, uint32_t function) {
    auto pin = std::find_if(pins.begin(), pins.end(), [function](const Pin& candidate) {
        return std::find(candidate.functions.begin(), candidate.functions.end(), function) != candidate.functions.end();
    });
    if(pin == pins.end()) {
        return std::nullopt;
    }
    return *pin;
}

std::variant<std::vector<Pin>, int> readConnector(const std::filesystem::path& device) {
    FileDescriptor connector(open(device.c_str(), O_RDONLY | O_CLOEXEC));
    if(connector.fd < 0) {
        return errno;
    }
    aux_info info{};
    info.size = sizeof(info);
    if(ioctl(connector.fd, AUX_IOC_INFO, &info) != 0) {
        return errno;
    }
    aux_state state{};
    state.size = sizeof(state);
    if(ioctl(connector.fd, AUX_IOC_STATE, &state) != 0) {
        return errno;
    }
    std::vector<Pin> pins;
    for(uint32_t index = 0; index < std::min<uint32_t>(info.npins, AUX_MAX_PINS); index++) {
        aux_pin_info pinInfo{};
        pinInfo.size = sizeof(pinInfo);
        pinInfo.pin = index;
        if(ioctl(connector.fd, AUX_IOC_PIN_INFO, &pinInfo) != 0) {
            return errno;
        }
        pins.push_back(parsePin(pinInfo, state.pins[index]));
    }
    return pins;
}

std::string readComm(pid_t pid) {
    std::ifstream file("/proc/" + std::to_string(pid) + "/comm");
    std::string comm;
    std::getline(file, comm);
    return comm;
}

std::string lineConsumer(const GpioLineRef& line) {
    std::error_code ec;
    for(std::filesystem::directory_iterator entry("/dev", ec), end; !ec && entry != end; entry.increment(ec)) {
        if(entry->path().filename().string().rfind("gpiochip", 0) != 0) {
            continue;
        }
        FileDescriptor chipFd(open(entry->path().c_str(), O_RDONLY | O_CLOEXEC));
        gpiochip_info chip{};
        if(chipFd.fd < 0 || ioctl(chipFd.fd, GPIO_GET_CHIPINFO_IOCTL, &chip) != 0 || fixedString(chip.label) != line.chip) {
            continue;
        }
        gpio_v2_line_info lineInfo{};
        lineInfo.offset = line.offset;
        if(ioctl(chipFd.fd, GPIO_V2_GET_LINEINFO_IOCTL, &lineInfo) != 0) {
            return "";
        }
        std::string consumer = fixedString(lineInfo.consumer);
        // gpiolib names the consumer of a request that gave none "?"
        return consumer == "?" ? "" : consumer;
    }
    return "";
}

gate::PinUnavailable busyFsyncPin(const std::vector<Pin>& pins) {
    gate::PinUnavailable busy{gate::PinProblem::Busy, "", std::nullopt};
    auto pin = findPinWithFunction(pins, AUX_FUNC_FSYNC);
    if(!pin) {
        return busy;
    }
    busy.pinLabel = pin->label;
    if(const auto* process = std::get_if<ProcessOwner>(&pin->state.owner)) {
        busy.holder = gate::ProcessHolder{process->pid, readComm(process->pid)};
    } else if(const auto* kernel = std::get_if<KernelOwner>(&pin->state.owner)) {
        busy.holder = gate::KernelHolder{kernel->name};
    } else if(pin->state.lineRequested) {
        // The connector's gpiochip names each line aux-<label>
        busy.holder = gate::LineHolder{"aux-" + pin->label, pin->gpio ? lineConsumer(*pin->gpio) : ""};
    }
    return busy;
}

}  // namespace aux
}  // namespace dai
