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
        .def("startCustomEvent", &PipelineEventDispatcher::startCustomEvent, py::arg("source"), DOC(dai, utility, PipelineEventDispatcher, startCustomEvent))
        .def("endCustomEvent", &PipelineEventDispatcher::endCustomEvent, py::arg("source"), DOC(dai, utility, PipelineEventDispatcher, endCustomEvent))
        .def("pingCustomEvent", &PipelineEventDispatcher::pingCustomEvent, py::arg("source"), DOC(dai, utility, PipelineEventDispatcher, pingCustomEvent))
        .def("inputBlockEvent", &PipelineEventDispatcher::inputBlockEvent, DOC(dai, utility, PipelineEventDispatcher, inputBlockEvent))
        .def("outputBlockEvent", &PipelineEventDispatcher::outputBlockEvent, DOC(dai, utility, PipelineEventDispatcher, outputBlockEvent))
        .def("customBlockEvent", &PipelineEventDispatcher::customBlockEvent, py::arg("source"), DOC(dai, utility, PipelineEventDispatcher, customBlockEvent));
}
