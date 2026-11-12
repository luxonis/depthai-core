#pragma once

#include <filesystem>

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

}  // namespace aux
}  // namespace dai
