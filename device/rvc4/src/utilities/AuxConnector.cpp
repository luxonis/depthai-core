#include "utilities/AuxConnector.hpp"

#include <fcntl.h>
#include <linux/aux-connector.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
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

bool hasFunction(const Pin& pin, uint32_t function) {
    return std::find(pin.functions.begin(), pin.functions.end(), function) != pin.functions.end();
}

std::optional<std::filesystem::path> findChip(const std::string& label) {
    std::error_code ec;
    for(std::filesystem::directory_iterator entry("/dev", ec), end; !ec && entry != end; entry.increment(ec)) {
        if(entry->path().filename().string().rfind("gpiochip", 0) != 0) {
            continue;
        }
        FileDescriptor chipFd(open(entry->path().c_str(), O_RDONLY | O_CLOEXEC));
        gpiochip_info chip{};
        if(chipFd.fd >= 0 && ioctl(chipFd.fd, GPIO_GET_CHIPINFO_IOCTL, &chip) == 0 && fixedString(chip.label) == label) {
            return entry->path();
        }
    }
    return std::nullopt;
}

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
    Pin pin{info.pin, fixedString(info.label), info.fixed, {}, 0, std::nullopt, {NoOwner{}, (state.flags & AUX_PIN_FLAG_LINE) != 0}};
    for(uint32_t i = 0; i < std::min<uint32_t>(info.nfuncs, AUX_MAX_PIN_FUNCS); i++) {
        pin.functions.push_back(info.funcs[i].function);
        if(info.funcs[i].function == AUX_FUNC_GPIO) {
            pin.gpioPullCaps = info.funcs[i].pulls;
        }
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
    auto pin = std::find_if(pins.begin(), pins.end(), [function](const Pin& candidate) { return hasFunction(candidate, function); });
    if(pin == pins.end()) {
        return std::nullopt;
    }
    return *pin;
}

std::variant<Connector, int> readConnector(const std::filesystem::path& device) {
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
    Connector read{{}, state.detect};
    for(uint32_t index = 0; index < std::min<uint32_t>(info.npins, AUX_MAX_PINS); index++) {
        aux_pin_info pinInfo{};
        pinInfo.size = sizeof(pinInfo);
        pinInfo.pin = index;
        if(ioctl(connector.fd, AUX_IOC_PIN_INFO, &pinInfo) != 0) {
            return errno;
        }
        read.pins.push_back(parsePin(pinInfo, state.pins[index]));
    }
    return read;
}

std::string readComm(pid_t pid) {
    std::ifstream file("/proc/" + std::to_string(pid) + "/comm");
    std::string comm;
    std::getline(file, comm);
    return comm;
}

std::string lineConsumer(const std::filesystem::path& chip, uint32_t offset) {
    FileDescriptor chipFd(open(chip.c_str(), O_RDONLY | O_CLOEXEC));
    gpio_v2_line_info lineInfo{};
    lineInfo.offset = offset;
    if(chipFd.fd < 0 || ioctl(chipFd.fd, GPIO_V2_GET_LINEINFO_IOCTL, &lineInfo) != 0) {
        return "";
    }
    std::string consumer = fixedString(lineInfo.consumer);
    // gpiolib names the consumer of a request that gave none "?"
    return consumer == "?" ? "" : consumer;
}

namespace {

const std::string linePrefix = "aux-";

std::optional<gate::Holder> holderOf(const Pin& pin) {
    if(const auto* process = std::get_if<ProcessOwner>(&pin.state.owner)) {
        return gate::ProcessHolder{process->pid, readComm(process->pid)};
    }
    if(const auto* kernel = std::get_if<KernelOwner>(&pin.state.owner)) {
        return gate::KernelHolder{kernel->name};
    }
    if(pin.state.lineRequested) {
        auto chip = pin.gpio ? findChip(pin.gpio->chip) : std::nullopt;
        return gate::LineHolder{linePrefix + pin.label, chip ? lineConsumer(*chip, pin.gpio->offset) : ""};
    }
    return std::nullopt;
}

const std::map<uint32_t, std::string> functionNames = {
    {AUX_FUNC_GPIO, "gpio"},
    {AUX_FUNC_UART_RX, "uart-rx"},
    {AUX_FUNC_UART_TX, "uart-tx"},
    {AUX_FUNC_I2C_SDA, "i2c-sda"},
    {AUX_FUNC_I2C_SCL, "i2c-scl"},
    {AUX_FUNC_FSYNC, "fsync"},
    {AUX_FUNC_STROBE, "strobe"},
    {AUX_FUNC_DETECT, "detect"},
};

const std::map<uint32_t, std::string> fixedRoleNames = {
    {AUX_FIXED_USB_DP, "usb-dp"},
    {AUX_FIXED_USB_DN, "usb-dn"},
    {AUX_FIXED_POWER_OUT, "power-out"},
    {AUX_FIXED_GND, "gnd"},
    {AUX_FIXED_DC_IN, "dc-in"},
    {AUX_FIXED_CAN_H, "can-h"},
    {AUX_FIXED_CAN_L, "can-l"},
    {AUX_FIXED_IGNITION, "ignition"},
    {AUX_FIXED_NC, "nc"},
};

std::string nameOf(const std::map<uint32_t, std::string>& names, uint32_t id) {
    auto name = names.find(id);
    return name == names.end() ? std::to_string(id) : name->second;
}

}  // namespace

std::optional<std::string> connectorHintForTlmm(int tlmm, const std::string& consumer) {
    const std::string tlmmName = "TLMM " + std::to_string(tlmm);
    if(consumer == "aux-connector") {
        return tlmmName + " is a control line of the aux connector";
    }
    if(consumer.size() > linePrefix.size() && consumer.compare(0, linePrefix.size(), linePrefix) == 0) {
        return tlmmName + " is connector pin " + consumer.substr(linePrefix.size()) + ", use GPIO.setup(\"" + consumer + "\")";
    }
    return std::nullopt;
}

gate::PinUnavailable busyFsyncPin(const std::vector<Pin>& pins) {
    auto pin = findPinWithFunction(pins, AUX_FUNC_FSYNC);
    if(!pin) {
        return {gate::PinProblem::Busy, "", std::nullopt};
    }
    return {gate::PinProblem::Busy, pin->label, holderOf(*pin)};
}

std::string gpioSetupError(const std::string& name, const std::string& reason) {
    return "Cannot set up \"" + name + "\": " + reason;
}

std::variant<GpioPlan, std::string> planGpio(const std::string& name, gpio::Bias bias, const std::variant<Connector, int>& connector) {
    auto refuse = [&name](const std::string& reason) { return gpioSetupError(name, reason); };
    if(name.size() <= linePrefix.size() || name.compare(0, linePrefix.size(), linePrefix) != 0) {
        return refuse("a connector pin is named \"aux-<label>\", such as \"aux-1\"");
    }
    const std::string label = name.substr(linePrefix.size());
    if(bias == gpio::Bias::PullDown) {
        return refuse("connector pins have no pull-down");
    }
    if(const int* err = std::get_if<int>(&connector)) {
        return refuse(*err == ENOENT ? "this device has no aux connector driver" : "cannot read /dev/aux-connector: " + std::string(std::strerror(*err)));
    }

    const Connector& read = std::get<Connector>(connector);
    auto pin = std::find_if(read.pins.begin(), read.pins.end(), [&label](const Pin& candidate) { return candidate.label == label; });
    if(pin == read.pins.end()) {
        return refuse("no pin " + label + " on this board");
    }
    if(pin->fixed != AUX_FIXED_NONE) {
        return refuse("pin " + label + " is a fixed pin (" + nameOf(fixedRoleNames, pin->fixed) + ")");
    }
    if(!pin->gpio) {
        std::string functions;
        for(uint32_t function : pin->functions) {
            functions += (functions.empty() ? "" : ", ") + nameOf(functionNames, function);
        }
        return refuse("pin " + label + " has no function gpio, its functions are " + functions);
    }

    // Pins without a choice keep the board's pull, which PIN_INFO does not always list
    uint32_t pull = AUX_PULL_DEFAULT;
    if(bias == gpio::Bias::PullUp && pin->gpioPullCaps != 0) {
        if(!(pin->gpioPullCaps & AUX_PULL_CAP_UP)) {
            return refuse("pin " + label + " has no pull-up as gpio");
        }
        pull = AUX_PULL_UP;
    } else if(bias == gpio::Bias::Disabled && (pin->gpioPullCaps & AUX_PULL_CAP_OFF)) {
        pull = AUX_PULL_OFF;
    }

    if(auto holder = holderOf(*pin)) {
        std::string reason = "pin " + label + " " + gate::holderText(*holder);
        if(std::holds_alternative<gate::KernelHolder>(*holder) && hasFunction(*pin, AUX_FUNC_FSYNC)) {
            reason += ". Set the external frame sync role to STANDALONE to use pin " + label + " as GPIO";
        }
        return refuse(reason);
    }
    if(read.detect == AUX_DETECT_0 && hasFunction(*pin, AUX_FUNC_DETECT)) {
        return refuse("pin " + label + " is busy: something is attached to DET. Unplug it to use pin " + label + " as GPIO");
    }
    return GpioPlan{pin->index, pull, *pin->gpio};
}

GpioPins::~GpioPins() {
    if(fd >= 0) {
        close(fd);
    }
}

std::variant<AcquiredLine, std::string> GpioPins::acquire(const std::string& name, gpio::Bias bias) {
    auto plan = planGpio(name, bias, readConnector());
    if(const auto* error = std::get_if<std::string>(&plan)) {
        return *error;
    }
    const auto& request = std::get<GpioPlan>(plan);
    const std::string pin = "pin " + name.substr(linePrefix.size());
    auto chip = findChip(request.line.chip);
    if(!chip) {
        return gpioSetupError(name, "no gpiochip " + request.line.chip + " for " + pin);
    }
    if(fd < 0) {
        fd = open("/dev/aux-connector", O_RDWR | O_CLOEXEC);
        if(fd < 0) {
            return gpioSetupError(name, "cannot open /dev/aux-connector: " + std::string(std::strerror(errno)));
        }
    }

    aux_request acquire{};
    acquire.size = sizeof(acquire);
    acquire.pin = request.pin;
    acquire.function = AUX_FUNC_GPIO;
    acquire.direction = AUX_DIR_DEFAULT;
    acquire.pull = request.pull;
    if(ioctl(fd, AUX_IOC_ACQUIRE, &acquire) != 0) {
        int err = errno;
        if(err == EBUSY) {
            auto whyBusy = planGpio(name, bias, readConnector());
            if(const auto* error = std::get_if<std::string>(&whyBusy)) {
                return *error;
            }
            return gpioSetupError(name, pin + " is busy");
        }
        return gpioSetupError(name, pin + ": " + std::strerror(err));
    }
    return AcquiredLine{request.pin, std::move(*chip), request.line.offset};
}

std::error_code GpioPins::release(uint32_t pin) {
    aux_request release{};
    release.size = sizeof(release);
    release.pin = pin;
    return ioctl(fd, AUX_IOC_RELEASE, &release) == 0 ? std::error_code() : std::error_code(errno, std::generic_category());
}

}  // namespace aux
}  // namespace dai
