#include "Common.hpp"
#include "depthai/pipeline/node/internal/PipelineEventAggregation.hpp"
#include "depthai/properties/internal/PipelineEventAggregationProperties.hpp"

void bind_pipelineeventaggregation(pybind11::module& m, void* pCallstack) {
    // using namespace dai;
    // using namespace dai::node::internal;
    //
    // // Node and Properties declare upfront
    // py::class_<PipelineEventAggregationProperties> pipelineEventAggregationProperties(
    //     m, "PipelineEventAggregationProperties", DOC(dai, PipelineEventAggregationProperties));
    // auto pipelineEventAggregation = ADD_NODE(PipelineEventAggregation);
    //
    // ///////////////////////////////////////////////////////////////////////
    // ///////////////////////////////////////////////////////////////////////
    // ///////////////////////////////////////////////////////////////////////
    // // Call the rest of the type defines, then perform the actual bindings
    // Callstack* callstack = (Callstack*)pCallstack;
    // auto cb = callstack->top();
    // callstack->pop();
    // cb(m, pCallstack);
    // // Actual bindings
    // ///////////////////////////////////////////////////////////////////////
    // ///////////////////////////////////////////////////////////////////////
    // ///////////////////////////////////////////////////////////////////////
    //
    // // Properties
    // pipelineEventAggregationProperties.def_readwrite("aggregationWindowSize", &PipelineEventAggregationProperties::aggregationWindowSize)
    //     .def_readwrite("eventBatchSize", &PipelineEventAggregationProperties::eventBatchSize)
    //     .def_readwrite("sendEvents", &PipelineEventAggregationProperties::sendEvents);
    //
    // // Node
    // pipelineEventAggregation.def_readonly("out", &PipelineEventAggregation::out, DOC(dai, node, PipelineEventAggregation, out))
    //     .def_readonly("inputs", &PipelineEventAggregation::inputs, DOC(dai, node, PipelineEventAggregation, inputs))
    //     .def("setRunOnHost", &PipelineEventAggregation::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, PipelineEventAggregation, setRunOnHost))
    //     .def("runOnHost", &PipelineEventAggregation::runOnHost, DOC(dai, node, PipelineEventAggregation, runOnHost));
    // daiNodeModule.attr("PipelineEventAggregation").attr("Properties") = pipelineEventAggregationProperties;
}
