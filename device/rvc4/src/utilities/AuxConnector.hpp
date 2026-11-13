#pragma once

#include <sys/types.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <variant>
#include <vector>

#include "utilities/FsyncRole.hpp"
#include "utilities/GpioLine.hpp"

struct aux_pin_info;
struct aux_pin_state;

namespace dai {
namespace aux {

/**
 * Whether the fsync-stm node links to an enabled aux connector. fsync-stm looks the connector up only then,
 * so without the link a failed SET_INFO is a controller error, as on OS images without the connector driver.
 */
bool fsyncLinkedToConnector(const std::filesystem::path& deviceTree = "/proc/device-tree");

/**
 * Whether the aux connector driver is bound. Its probe creates the device node last.
 */
bool connectorBound(const std::filesystem::path& device = "/dev/aux-connector");

struct GpioLineRef {
    std::string chip;
    uint32_t offset;
};

struct NoOwner {};
struct ProcessOwner {
    pid_t pid;
};
struct KernelOwner {
    std::string name;
};
using Owner = std::variant<NoOwner, ProcessOwner, KernelOwner>;

struct PinState {
    Owner owner;
    bool lineRequested;
};

/**
 * One connector pin from PIN_INFO and its entry in STATE, read together.
 */
struct Pin {
    uint32_t index;
    std::string label;
    uint32_t fixed;
    std::vector<uint32_t> functions;  // AUX_FUNC_*, empty on a fixed pin
    uint32_t gpioPullCaps;
    std::optional<GpioLineRef> gpio;
    PinState state;
};

struct Connector {
    std::vector<Pin> pins;
    uint32_t detect;
};

Pin parsePin(const aux_pin_info& info, const aux_pin_state& state);
std::optional<Pin> findPinWithFunction(const std::vector<Pin>& pins, uint32_t function);

/**
 * Read every pin of the connector once, without taking a lock.
 * @return The connector, or the errno of the first call that failed.
 */
std::variant<Connector, int> readConnector(const std::filesystem::path& device = "/dev/aux-connector");

/**
 * @return The name in /proc/<pid>/comm, "" when it cannot be read.
 */
std::string readComm(pid_t pid);

/**
 * @return The consumer of a requested line, "" when the line has none or cannot be read.
 */
std::string lineConsumer(const std::filesystem::path& chip, uint32_t offset);

std::optional<std::string> connectorHintForTlmm(int tlmm, const std::string& consumer);

/**
 * Describe the FSYNC pin of a connector whose FSYNC pin fsync-stm could not acquire.
 * @return Busy with the pin's label and its holder, as far as the connector and /proc show them.
 */
gate::PinUnavailable busyFsyncPin(const std::vector<Pin>& pins);

struct GpioPlan {
    uint32_t pin;
    uint32_t pull;
    GpioLineRef line;
};

std::variant<GpioPlan, std::string> planGpio(const std::string& name, gpio::Bias bias, const std::variant<Connector, int>& connector);

std::string gpioSetupError(const std::string& name, const std::string& reason);

struct AcquiredLine {
    uint32_t pin;
    std::filesystem::path chip;
    uint32_t offset;
};

class GpioPins {
   public:
    GpioPins() = default;
    GpioPins(const GpioPins&) = delete;
    GpioPins& operator=(const GpioPins&) = delete;
    ~GpioPins();

    std::variant<AcquiredLine, std::string> acquire(const std::string& name, gpio::Bias bias);

    std::error_code release(uint32_t pin);

   private:
    int fd = -1;
};

}  // namespace aux
}  // namespace dai
