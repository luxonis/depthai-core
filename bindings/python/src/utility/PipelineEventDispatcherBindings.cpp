#include "PipelineEventDispatcherBindings.hpp"
#include "depthai/utility/PipelineEventDispatcher.hpp"

void PipelineEventDispatcherBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    using namespace dai::utility;
    auto pipelineEventDispatcher = py::class_<PipelineEventDispatcher>(m, "PipelineEventDispatcher");

    pipelineEventDispatcher
        .def(py::init<Node::Output*>(), py::arg("output"))
        .def("setNodeId", &PipelineEventDispatcher::setNodeId, py::arg("id"), DOC(dai, utility, PipelineEventDispatcher, setNodeId))
        .def("addEvent", &PipelineEventDispatcher::addEvent, py::arg("source"), py::arg("type"), DOC(dai, utility, PipelineEventDispatcher, addEvent))
        .def("startEvent", &PipelineEventDispatcher::startEvent, py::arg("source"), py::arg("queueSize") = std::nullopt, py::arg("metadata") = std::nullopt, DOC(dai, utility, PipelineEventDispatcher, startEvent))
        .def("endEvent", &PipelineEventDispatcher::endEvent, py::arg("source"), py::arg("queueSize") = std::nullopt, py::arg("metadata") = std::nullopt, DOC(dai, utility, PipelineEventDispatcher, endEvent))
        .def("pingEvent", &PipelineEventDispatcher::pingEvent, py::arg("source"), DOC(dai, utility, PipelineEventDispatcher, pingEvent));
}
