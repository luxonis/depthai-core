#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/GatherData.hpp"

void bind_gatherdata(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<GatherDataProperties> gatherDataProperties(m, "GatherDataProperties", DOC(dai, GatherDataProperties));
    auto gatherData = ADD_NODE(GatherData);

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

    gatherData.def_readonly("referenceInput", &GatherData::referenceInput, DOC(dai, node, GatherData, referenceInput))
        .def_readonly("collectingInput", &GatherData::collectingInput, DOC(dai, node, GatherData, collectingInput))
        .def_readonly("output", &GatherData::output, DOC(dai, node, GatherData, output))
        .def_readonly("passthroughCollectingInput", &GatherData::passthroughCollectingInput)
        .def("setRunOnHost", &GatherData::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, GatherData, setRunOnHost))
        .def("runOnHost", &GatherData::runOnHost, DOC(dai, node, GatherData, runOnHost));

    daiNodeModule.attr("GatherData").attr("Properties") = gatherDataProperties;
}
