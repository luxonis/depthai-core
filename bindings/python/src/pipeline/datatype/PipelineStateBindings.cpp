#include <memory>

#include "DatatypeBindings.hpp"

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
    py::class_<NodeState::Timing> nodeStateTiming(nodeState, "Timing", DOC(dai, NodeState, Timing));
    py::class_<NodeState::QueueStats> nodeStateQueueStats(nodeState, "QueueStats", DOC(dai, NodeState, QueueStats));
    py::class_<NodeState::InputQueueState> nodeStateInputQueueState(nodeState, "InputQueueState", DOC(dai, NodeState, InputQueueState));
    py::class_<NodeState::OutputQueueState> nodeStateOutputQueueState(nodeState, "OutputQueueState", DOC(dai, NodeState, OutputQueueState));
    py::class_<PipelineState, Py<PipelineState>, Buffer, std::shared_ptr<PipelineState>> pipelineState(m, "PipelineState", DOC(dai, PipelineState));

    py::enum_<NodeState::InputQueueState::State>(nodeStateInputQueueState, "State", DOC(dai, NodeState, InputQueueState, State))
        .value("IDLE", NodeState::InputQueueState::State::IDLE)
        .value("WAITING", NodeState::InputQueueState::State::WAITING)
        .value("BLOCKED", NodeState::InputQueueState::State::BLOCKED);
    py::enum_<NodeState::OutputQueueState::State>(nodeStateOutputQueueState, "State", DOC(dai, NodeState, OutputQueueState, State))
        .value("IDLE", NodeState::OutputQueueState::State::IDLE)
        .value("SENDING", NodeState::OutputQueueState::State::SENDING);
    py::enum_<NodeState::State>(nodeState, "State", DOC(dai, NodeState, State))
        .value("IDLE", NodeState::State::IDLE)
        .value("GETTING_INPUTS", NodeState::State::GETTING_INPUTS)
        .value("PROCESSING", NodeState::State::PROCESSING)
        .value("SENDING_OUTPUTS", NodeState::State::SENDING_OUTPUTS);

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

    nodeStateTiming.def(py::init<>())
        .def("__repr__", &NodeState::Timing::str)
        .def_readwrite("fps", &NodeState::Timing::fps, DOC(dai, NodeState, Timing, fps))
        .def_readwrite("durationStats", &NodeState::Timing::durationStats, DOC(dai, NodeState, Timing, durationStats));

    nodeStateQueueStats.def(py::init<>())
        .def("__repr__", &NodeState::QueueStats::str)
        .def_readwrite("maxQueued", &NodeState::QueueStats::maxQueued, DOC(dai, NodeState, QueueStats, maxQueued))
        .def_readwrite("minQueuedRecent", &NodeState::QueueStats::minQueuedRecent, DOC(dai, NodeState, QueueStats, minQueuedRecent))
        .def_readwrite("maxQueuedRecent", &NodeState::QueueStats::maxQueuedRecent, DOC(dai, NodeState, QueueStats, maxQueuedRecent))
        .def_readwrite("medianQueuedRecent", &NodeState::QueueStats::medianQueuedRecent, DOC(dai, NodeState, QueueStats, medianQueuedRecent));

    nodeStateInputQueueState.def(py::init<>())
        .def("__repr__", &NodeState::InputQueueState::str)
        .def_readwrite("state", &NodeState::InputQueueState::state, DOC(dai, NodeState, InputQueueState, state))
        .def_readwrite("numQueued", &NodeState::InputQueueState::numQueued, DOC(dai, NodeState, InputQueueState, numQueued))
        .def_readwrite("timing", &NodeState::InputQueueState::timing, DOC(dai, NodeState, InputQueueState, timing))
        .def_readwrite("queueStats", &NodeState::InputQueueState::queueStats, DOC(dai, NodeState, InputQueueState, queueStats));

    nodeStateOutputQueueState.def(py::init<>())
        .def("__repr__", &NodeState::OutputQueueState::str)
        .def_readwrite("state", &NodeState::OutputQueueState::state, DOC(dai, NodeState, OutputQueueState, state))
        .def_readwrite("timing", &NodeState::OutputQueueState::timing, DOC(dai, NodeState, OutputQueueState, timing));

    nodeState.def(py::init<>())
        .def("__repr__", &NodeState::str)
        .def_readwrite("state", &NodeState::state, DOC(dai, NodeState, state))
        .def_readwrite("events", &NodeState::events, DOC(dai, NodeState, events))
        .def_readwrite("inputStates", &NodeState::inputStates, DOC(dai, NodeState, inputStates))
        .def_readwrite("outputStates", &NodeState::outputStates, DOC(dai, NodeState, outputStates))
        .def_readwrite("inputsGetTiming", &NodeState::inputsGetTiming, DOC(dai, NodeState, inputsGetTiming))
        .def_readwrite("outputsSendTiming", &NodeState::outputsSendTiming, DOC(dai, NodeState, outputsSendTiming))
        .def_readwrite("mainLoopTiming", &NodeState::mainLoopTiming, DOC(dai, NodeState, mainLoopTiming))
        .def_readwrite("otherTimings", &NodeState::otherTimings, DOC(dai, NodeState, otherTimings));

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
