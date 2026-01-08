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
    py::enum_<PipelineEvent::Type> pipelineEventType(pipelineEvent, "Type", DOC(dai, PipelineEvent, Type));
    py::enum_<PipelineEvent::Interval> pipelineEventInterval(pipelineEvent, "Interval", DOC(dai, PipelineEvent, Interval));
    py::enum_<PipelineEvent::Status> pipelineEventStatus(pipelineEvent, "Status", DOC(dai, PipelineEvent, Status));

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

    pipelineEventType.value("CUSTOM", PipelineEvent::Type::CUSTOM)
        .value("LOOP", PipelineEvent::Type::LOOP)
        .value("INPUT", PipelineEvent::Type::INPUT)
        .value("OUTPUT", PipelineEvent::Type::OUTPUT)
        .value("INPUT_BLOCK", PipelineEvent::Type::INPUT_BLOCK)
        .value("OUTPUT_BLOCK", PipelineEvent::Type::OUTPUT_BLOCK);
    pipelineEventInterval.value("NONE", PipelineEvent::Interval::NONE)
        .value("START", PipelineEvent::Interval::START)
        .value("END", PipelineEvent::Interval::END);
    pipelineEventStatus.value("SUCCESS", PipelineEvent::Status::SUCCESS)
        .value("BLOCKED", PipelineEvent::Status::BLOCKED)
        .value("CANCELLED", PipelineEvent::Status::CANCELLED);

    // Message
    pipelineEvent.def(py::init<>())
        .def("__repr__", &PipelineEvent::str)
        .def_readwrite("nodeId", &PipelineEvent::nodeId, DOC(dai, PipelineEvent, nodeId))
        .def_readwrite("status", &PipelineEvent::status, DOC(dai, PipelineEvent, status))
        .def_readwrite("queueSize", &PipelineEvent::queueSize, DOC(dai, PipelineEvent, queueSize))
        .def_readwrite("interval", &PipelineEvent::interval, DOC(dai, PipelineEvent, interval))
        .def_readwrite("type", &PipelineEvent::type, DOC(dai, PipelineEvent, type))
        .def_readwrite("source", &PipelineEvent::source, DOC(dai, PipelineEvent, source))
        .def("getTimestamp", &PipelineEvent::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &PipelineEvent::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &PipelineEvent::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("setTimestamp", &PipelineEvent::setTimestamp, DOC(dai, Buffer, setTimestamp))
        .def("setTimestampDevice", &PipelineEvent::setTimestampDevice, DOC(dai, Buffer, setTimestampDevice))
        .def("setSequenceNum", &PipelineEvent::setSequenceNum, DOC(dai, Buffer, setSequenceNum));
}
