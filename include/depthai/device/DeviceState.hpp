#pragma once

namespace dai {

/**
 * State of a device that is part of a running pipeline.
 */
enum class DeviceState : int {
    /// Device is connected and streaming
    RUNNING = 0,
    /// Device connection was lost; reconnection not started yet
    DISCONNECTED = 1,
    /// Device is being reconnected
    RECONNECTING = 2,
    /// Device is gone for good (closed, or reconnection exhausted)
    FAILED = 3,
};

}  // namespace dai
