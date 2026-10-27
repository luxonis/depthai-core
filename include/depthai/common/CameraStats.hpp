#pragma once

#include <cstdint>
#include <string>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/optional.hpp"

namespace dai {

/**
 * Cumulative, non-destructive camera-side health snapshot for one physical sensor.
 *
 * Empty counters mean unsupported or unavailable; a measured zero is 0. AVAILABLE
 * permits partial counter availability. Host queue discards are excluded.
 * CSL causes may overlap each other and HAL notifications; use cslErrorEvents as
 * the CSL event total, without adding its cause counters or HAL notifications.
 *
 * Counters are unsigned 64-bit values and wrap modulo 2^64. Subtract only valid
 * samples with the same socket and sessionId, advancing timestamps, and fewer
 * than 2^64 events between samples. Establish a new baseline after any missing
 * reading. A session changes whenever a counter source resets, including runtime
 * restart or CamX stream restart; its identifier is opaque.
 *
 * INACTIVE retains the last session's counters and stop timestamp where possible.
 * Unverified physical socket mappings return AUTO and UNAVAILABLE.
 */
struct CameraStats {
    /// Availability of a sensor snapshot; individual counters can still be empty.
    enum class Status {
        AVAILABLE,    ///< Active sensor with at least one available counter.
        INACTIVE,     ///< Sensor has no active capture session; retained counters are historical.
        UNSUPPORTED,  ///< Platform does not implement camera statistics.
        UNAVAILABLE   ///< Statistics cannot currently be attributed or read reliably.
    };

    /// Physical camera socket. AUTO means the mapping could not be verified.
    CameraBoardSocket socket = CameraBoardSocket::AUTO;
    /// Internal camera ID for diagnostics; never use vector position as identity.
    std::int32_t cameraId = -1;
    /// Overall lifecycle/availability state.
    Status status = Status::UNAVAILABLE;
    /// Opaque capture/counter-source session identifier; empty before any session.
    std::string sessionId{};
    /// Device CLOCK_MONOTONIC snapshot time in nanoseconds, or retained stop time.
    std::uint64_t timestampNs = 0;
    /// Separate device-monotonic CSL sampling time; empty if that source is unavailable.
    std::optional<std::uint64_t> cslTimestampNs = std::nullopt;

    /// Successful HAL capture requests: final metadata and every requested output buffer returned with OK status, with no request/result/buffer error. Counted
    /// once across all streams; not an ISP-stage completion counter.
    std::optional<std::uint64_t> completedCaptures = std::nullopt;
    /// HAL DEVICE error notifications. These invalidate capture tracking until a new capture session.
    std::optional<std::uint64_t> halDeviceErrors = std::nullopt;
    /// HAL REQUEST error notifications, counted per notification.
    std::optional<std::uint64_t> halRequestErrors = std::nullopt;
    /// HAL RESULT error notifications, counted per notification.
    std::optional<std::uint64_t> halResultErrors = std::nullopt;
    /// HAL BUFFER error notifications. An error-status buffer return does not increment this again.
    std::optional<std::uint64_t> halBufferErrors = std::nullopt;
    /// Link-scoped CSL error/page-fault messages delivered by the kernel. One event per message; not a unique failed-frame count. Session/global events without
    /// sensor attribution are excluded.
    std::optional<std::uint64_t> cslErrorEvents = std::nullopt;
    /// CSL error messages with the link-stalled cause bit.
    std::optional<std::uint64_t> linkStalledEvents = std::nullopt;
    /// CSL error messages with the CSID fatal-error cause bit.
    std::optional<std::uint64_t> csidFatalEvents = std::nullopt;
    /// CSL error messages with the CSID output-FIFO overflow cause bit.
    std::optional<std::uint64_t> csidFifoOverflowEvents = std::nullopt;
    /// CSL error messages with the CSID recovery-overflow cause bit.
    std::optional<std::uint64_t> csidRecoveryOverflowEvents = std::nullopt;
    /// CSL error messages with the CSID lane-FIFO overflow cause bit.
    std::optional<std::uint64_t> csidLaneFifoOverflowEvents = std::nullopt;
    /// CSL error messages with the CSID pixel-count mismatch cause bit.
    std::optional<std::uint64_t> csidPixelCountMismatchEvents = std::nullopt;
    /// CSL error messages with the received packet-header corruption cause bit.
    std::optional<std::uint64_t> csidPacketHeaderCorruptionEvents = std::nullopt;
    /// CSL error messages with the missing packet-header data cause bit.
    std::optional<std::uint64_t> csidMissingPacketHeaderEvents = std::nullopt;
    /// CSL error messages with the sensor-switching error cause bit.
    std::optional<std::uint64_t> csidSensorSwitchingEvents = std::nullopt;
    /// CSL error messages with the missing/invalid EOF cause bit.
    std::optional<std::uint64_t> csidUnboundedFrameEvents = std::nullopt;
    /// CSL error messages with the ICP no-memory cause bit.
    std::optional<std::uint64_t> icpNoMemoryEvents = std::nullopt;
    /// CSL error messages with the ICP system-failure cause bit.
    std::optional<std::uint64_t> icpSystemFailureEvents = std::nullopt;
    /// CSL error messages with the missing end-of-transmission cause bit.
    std::optional<std::uint64_t> csidMissingEotEvents = std::nullopt;
    /// CSL error messages with the packet-payload CRC mismatch cause bit.
    std::optional<std::uint64_t> csidPayloadCorruptionEvents = std::nullopt;
    /// CSL error messages with the sensor stream-off failure cause bit.
    std::optional<std::uint64_t> sensorStreamOffFailureEvents = std::nullopt;
};

DEPTHAI_SERIALIZE_EXT(CameraStats,
                      socket,
                      cameraId,
                      status,
                      sessionId,
                      timestampNs,
                      cslTimestampNs,
                      completedCaptures,
                      halDeviceErrors,
                      halRequestErrors,
                      halResultErrors,
                      halBufferErrors,
                      cslErrorEvents,
                      linkStalledEvents,
                      csidFatalEvents,
                      csidFifoOverflowEvents,
                      csidRecoveryOverflowEvents,
                      csidLaneFifoOverflowEvents,
                      csidPixelCountMismatchEvents,
                      csidPacketHeaderCorruptionEvents,
                      csidMissingPacketHeaderEvents,
                      csidSensorSwitchingEvents,
                      csidUnboundedFrameEvents,
                      icpNoMemoryEvents,
                      icpSystemFailureEvents,
                      csidMissingEotEvents,
                      csidPayloadCorruptionEvents,
                      sensorStreamOffFailureEvents);

}  // namespace dai
