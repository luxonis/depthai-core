#pragma once

#include <linux/aux-connector.h>
#include <sys/types.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "utilities/FsyncRole.hpp"

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
    std::vector<uint32_t> functions;  // AUX_FUNC_*, empty on a fixed pin
    std::optional<GpioLineRef> gpio;
    PinState state;
};

Pin parsePin(const aux_pin_info& info, const aux_pin_state& state);
std::optional<Pin> findPinWithFunction(const std::vector<Pin>& pins, uint32_t function);

/**
 * Read every pin of the connector once, without taking a lock.
 * @return The pins in index order, or the errno of the first call that failed.
 */
std::variant<std::vector<Pin>, int> readConnector(const std::filesystem::path& device = "/dev/aux-connector");

/**
 * @return The name in /proc/<pid>/comm, "" when it cannot be read.
 */
std::string readComm(pid_t pid);

/**
 * @return The consumer of a requested line, "" when the line has none or cannot be read.
 */
std::string lineConsumer(const GpioLineRef& line);

/**
 * Describe the FSYNC pin of a connector whose FSYNC pin fsync-stm could not acquire.
 * @return Busy with the pin's label and its holder, as far as the connector and /proc show them.
 */
gate::PinUnavailable busyFsyncPin(const std::vector<Pin>& pins);

}  // namespace aux
}  // namespace dai
