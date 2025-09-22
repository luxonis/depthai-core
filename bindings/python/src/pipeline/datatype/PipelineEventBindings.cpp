#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/PipelineEvent.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_pipelineevent(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<PipelineEvent, Py<PipelineEvent>, Buffer, std::shared_ptr<PipelineEvent>> pipelineEvent(m, "PipelineEvent", DOC(dai, PipelineEvent));
    py::enum_<PipelineEvent::EventType> pipelineEventType(pipelineEvent, "EventType", DOC(dai, PipelineEvent, EventType));

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

    pipelineEventType.value("CUSTOM", PipelineEvent::EventType::CUSTOM)
        .value("LOOP", PipelineEvent::EventType::LOOP)
        .value("INPUT", PipelineEvent::EventType::INPUT)
        .value("OUTPUT", PipelineEvent::EventType::OUTPUT)
        .value("FUNC_CALL", PipelineEvent::EventType::FUNC_CALL);

    // Message
    pipelineEvent.def(py::init<>())
        .def("__repr__", &PipelineEvent::str)
        .def_readwrite("nodeId", &PipelineEvent::nodeId, DOC(dai, PipelineEvent, nodeId))
        .def_readwrite("metadata", &PipelineEvent::metadata, DOC(dai, PipelineEvent, metadata))
        .def_readwrite("duration", &PipelineEvent::duration, DOC(dai, PipelineEvent, duration))
        .def_readwrite("type", &PipelineEvent::type, DOC(dai, PipelineEvent, type))
        .def_readwrite("source", &PipelineEvent::source, DOC(dai, PipelineEvent, source))
        .def("getTimestamp", &PipelineEvent::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &PipelineEvent::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &PipelineEvent::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("setTimestamp", &PipelineEvent::setTimestamp, DOC(dai, PipelineEvent, setTimestamp))
        .def("setTimestampDevice", &PipelineEvent::setTimestampDevice, DOC(dai, PipelineEvent, setTimestampDevice))
        .def("setSequenceNum", &PipelineEvent::setSequenceNum, DOC(dai, PipelineEvent, setSequenceNum));
}
