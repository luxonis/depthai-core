#include "CameraStatsBindings.hpp"

#include <pybind11/stl_bind.h>

void bind_camera_stats(pybind11::module& m, void* pCallstack) {
    using CameraStats = dai::CameraStats;
    py::class_<CameraStats> stats(m, "CameraStats", DOC(dai, CameraStats));
    py::enum_<CameraStats::Status> status(stats, "Status", DOC(dai, CameraStats, Status));
    py::bind_vector<std::vector<CameraStats>>(m, "VectorCameraStats");

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    status.value("AVAILABLE", CameraStats::Status::AVAILABLE, DOC(dai, CameraStats, Status, AVAILABLE))
        .value("INACTIVE", CameraStats::Status::INACTIVE, DOC(dai, CameraStats, Status, INACTIVE))
        .value("UNSUPPORTED", CameraStats::Status::UNSUPPORTED, DOC(dai, CameraStats, Status, UNSUPPORTED))
        .value("UNAVAILABLE", CameraStats::Status::UNAVAILABLE, DOC(dai, CameraStats, Status, UNAVAILABLE));

    stats.def(py::init<>())
        .def_readwrite("socket", &CameraStats::socket, DOC(dai, CameraStats, socket))
        .def_readwrite("cameraId", &CameraStats::cameraId, DOC(dai, CameraStats, cameraId))
        .def_readwrite("status", &CameraStats::status, DOC(dai, CameraStats, status))
        .def_readwrite("sessionId", &CameraStats::sessionId, DOC(dai, CameraStats, sessionId))
        .def_readwrite("timestampNs", &CameraStats::timestampNs, DOC(dai, CameraStats, timestampNs))
        .def_readwrite("cslTimestampNs", &CameraStats::cslTimestampNs, DOC(dai, CameraStats, cslTimestampNs))
        .def_readwrite("completedCaptures", &CameraStats::completedCaptures, DOC(dai, CameraStats, completedCaptures))
        .def_readwrite("halDeviceErrors", &CameraStats::halDeviceErrors, DOC(dai, CameraStats, halDeviceErrors))
        .def_readwrite("halRequestErrors", &CameraStats::halRequestErrors, DOC(dai, CameraStats, halRequestErrors))
        .def_readwrite("halResultErrors", &CameraStats::halResultErrors, DOC(dai, CameraStats, halResultErrors))
        .def_readwrite("halBufferErrors", &CameraStats::halBufferErrors, DOC(dai, CameraStats, halBufferErrors))
        .def_readwrite("cslErrorEvents", &CameraStats::cslErrorEvents, DOC(dai, CameraStats, cslErrorEvents))
        .def_readwrite("linkStalledEvents", &CameraStats::linkStalledEvents, DOC(dai, CameraStats, linkStalledEvents))
        .def_readwrite("csidFatalEvents", &CameraStats::csidFatalEvents, DOC(dai, CameraStats, csidFatalEvents))
        .def_readwrite("csidFifoOverflowEvents", &CameraStats::csidFifoOverflowEvents, DOC(dai, CameraStats, csidFifoOverflowEvents))
        .def_readwrite("csidRecoveryOverflowEvents", &CameraStats::csidRecoveryOverflowEvents, DOC(dai, CameraStats, csidRecoveryOverflowEvents))
        .def_readwrite("csidLaneFifoOverflowEvents", &CameraStats::csidLaneFifoOverflowEvents, DOC(dai, CameraStats, csidLaneFifoOverflowEvents))
        .def_readwrite("csidPixelCountMismatchEvents", &CameraStats::csidPixelCountMismatchEvents, DOC(dai, CameraStats, csidPixelCountMismatchEvents))
        .def_readwrite(
            "csidPacketHeaderCorruptionEvents", &CameraStats::csidPacketHeaderCorruptionEvents, DOC(dai, CameraStats, csidPacketHeaderCorruptionEvents))
        .def_readwrite("csidMissingPacketHeaderEvents", &CameraStats::csidMissingPacketHeaderEvents, DOC(dai, CameraStats, csidMissingPacketHeaderEvents))
        .def_readwrite("csidSensorSwitchingEvents", &CameraStats::csidSensorSwitchingEvents, DOC(dai, CameraStats, csidSensorSwitchingEvents))
        .def_readwrite("csidUnboundedFrameEvents", &CameraStats::csidUnboundedFrameEvents, DOC(dai, CameraStats, csidUnboundedFrameEvents))
        .def_readwrite("icpNoMemoryEvents", &CameraStats::icpNoMemoryEvents, DOC(dai, CameraStats, icpNoMemoryEvents))
        .def_readwrite("icpSystemFailureEvents", &CameraStats::icpSystemFailureEvents, DOC(dai, CameraStats, icpSystemFailureEvents))
        .def_readwrite("csidMissingEotEvents", &CameraStats::csidMissingEotEvents, DOC(dai, CameraStats, csidMissingEotEvents))
        .def_readwrite("csidPayloadCorruptionEvents", &CameraStats::csidPayloadCorruptionEvents, DOC(dai, CameraStats, csidPayloadCorruptionEvents))
        .def_readwrite("sensorStreamOffFailureEvents", &CameraStats::sensorStreamOffFailureEvents, DOC(dai, CameraStats, sensorStreamOffFailureEvents));

    py::implicitly_convertible<py::list, std::vector<CameraStats>>();
}
