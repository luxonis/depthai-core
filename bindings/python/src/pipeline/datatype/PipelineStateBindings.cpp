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
    py::class_<NodeState::Timing> nodeStateTiming(nodeState, "Timing", DOC(dai, NodeState, Timing));
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

    nodeStateTiming.def(py::init<>())
        .def("__repr__", &NodeState::Timing::str)
        .def_readwrite("averageMicros", &NodeState::Timing::averageMicros, DOC(dai, NodeState, Timing, averageMicros))
        .def_readwrite("stdDevMicros", &NodeState::Timing::stdDevMicros, DOC(dai, NodeState, Timing, stdDevMicros));

    nodeState.def(py::init<>())
        .def("__repr__", &NodeState::str)
        .def_readwrite("events", &NodeState::events, DOC(dai, NodeState, events))
        .def_readwrite("timingsByType", &NodeState::timingsByType, DOC(dai, NodeState, timingsByType))
        .def_readwrite("timingsByInstance", &NodeState::timingsByInstance, DOC(dai, NodeState, timingsByInstance));

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
