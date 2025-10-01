#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/PipelineState.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_pipelinestate(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NodeState> nodeState(m, "NodeState", DOC(dai, NodeState));
    py::class_<NodeState::DurationEvent> durationEvent(nodeState, "DurationEvent", DOC(dai, NodeState, DurationEvent));
    py::class_<NodeState::TimingStats> nodeStateTimingStats(nodeState, "TimingStats", DOC(dai, NodeState, TimingStats));
    py::class_<NodeState::QueueStats> nodeStateQueueStats(nodeState, "QueueStats", DOC(dai, NodeState, QueueStats));
    py::class_<NodeState::QueueState> nodeStateQueueState(nodeState, "QueueState", DOC(dai, NodeState, QueueState));
    py::class_<PipelineState, Py<PipelineState>, Buffer, std::shared_ptr<PipelineState>> pipelineState(m, "PipelineState", DOC(dai, PipelineState));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    durationEvent.def(py::init<>())
        .def("__repr__", &NodeState::DurationEvent::str)
        .def_readwrite("startEvent", &NodeState::DurationEvent::startEvent, DOC(dai, NodeState, DurationEvent, startEvent))
        .def_readwrite("durationUs", &NodeState::DurationEvent::durationUs, DOC(dai, NodeState, TimingStats, durationUs));

    nodeStateTimingStats.def(py::init<>())
        .def("__repr__", &NodeState::TimingStats::str)
        .def_readwrite("minMicros", &NodeState::TimingStats::minMicros, DOC(dai, NodeState, TimingStats, minMicros))
        .def_readwrite("maxMicros", &NodeState::TimingStats::maxMicros, DOC(dai, NodeState, TimingStats, maxMicros))
        .def_readwrite("averageMicrosRecent", &NodeState::TimingStats::averageMicrosRecent, DOC(dai, NodeState, TimingStats, averageMicrosRecent))
        .def_readwrite("stdDevMicrosRecent", &NodeState::TimingStats::stdDevMicrosRecent, DOC(dai, NodeState, TimingStats, stdDevMicrosRecent))
        .def_readwrite("minMicrosRecent", &NodeState::TimingStats::minMicrosRecent, DOC(dai, NodeState, TimingStats, minMicrosRecent))
        .def_readwrite("maxMicrosRecent", &NodeState::TimingStats::maxMicrosRecent, DOC(dai, NodeState, TimingStats, maxMicrosRecent))
        .def_readwrite("medianMicrosRecent", &NodeState::TimingStats::medianMicrosRecent, DOC(dai, NodeState, TimingStats, medianMicrosRecent));

    nodeStateQueueStats.def(py::init<>())
        .def("__repr__", &NodeState::QueueStats::str)
        .def_readwrite("maxQueued", &NodeState::QueueStats::maxQueued, DOC(dai, NodeState, QueueStats, maxQueued))
        .def_readwrite("minQueuedRecent", &NodeState::QueueStats::minQueuedRecent, DOC(dai, NodeState, QueueStats, minQueuedRecent))
        .def_readwrite("maxQueuedRecent", &NodeState::QueueStats::maxQueuedRecent, DOC(dai, NodeState, QueueStats, maxQueuedRecent))
        .def_readwrite("medianQueuedRecent", &NodeState::QueueStats::medianQueuedRecent, DOC(dai, NodeState, QueueStats, medianQueuedRecent));

    nodeStateQueueState.def(py::init<>())
        .def("__repr__", &NodeState::QueueState::str)
        .def_readwrite("waiting", &NodeState::QueueState::waiting, DOC(dai, NodeState, QueueState, waiting))
        .def_readwrite("numQueued", &NodeState::QueueState::numQueued, DOC(dai, NodeState, QueueState, numQueued))
        .def_readwrite("timingStats", &NodeState::QueueState::timingStats, DOC(dai, NodeState, QueueState, timingStats))
        .def_readwrite("queueStats", &NodeState::QueueState::queueStats, DOC(dai, NodeState, QueueState, queueStats));

    nodeState.def(py::init<>())
        .def("__repr__", &NodeState::str)
        .def_readwrite("events", &NodeState::events, DOC(dai, NodeState, events))
        .def_readwrite("timingsByType", &NodeState::timingsByType, DOC(dai, NodeState, timingsByType))
        .def_readwrite("inputStates", &NodeState::inputStates, DOC(dai, NodeState, inputStates))
        .def_readwrite("outputStates", &NodeState::outputStates, DOC(dai, NodeState, outputStates))
        .def_readwrite("mainLoopStats", &NodeState::mainLoopStats, DOC(dai, NodeState, mainLoopStats))
        .def_readwrite("otherStats", &NodeState::otherStats, DOC(dai, NodeState, otherStats));

    // Message
    pipelineState.def(py::init<>())
        .def("__repr__", &PipelineState::str)
        .def_readwrite("nodeStates", &PipelineState::nodeStates, DOC(dai, PipelineState, nodeStates))
        .def("getTimestamp", &PipelineState::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &PipelineState::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &PipelineState::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("setTimestamp", &PipelineState::setTimestamp, DOC(dai, PipelineState, setTimestamp))
        .def("setTimestampDevice", &PipelineState::setTimestampDevice, DOC(dai, PipelineState, setTimestampDevice))
        .def("setSequenceNum", &PipelineState::setSequenceNum, DOC(dai, PipelineState, setSequenceNum));
}
