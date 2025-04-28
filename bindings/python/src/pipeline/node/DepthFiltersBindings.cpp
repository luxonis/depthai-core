#include <pybind11/stl.h>
#include "Common.hpp"
#include "depthai/pipeline/node/host/DepthFilters.hpp"

void bind_depthfilters(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace node;

    // bind properties
    py::class_<SequentialDepthFiltersProperties> sequentialDepthFiltersProperties(m, "SequentialDepthFiltersProperties", DOC(dai, SequentialDepthFiltersProperties));
    py::class_<DepthConfidenceFilterProperties> depthConfidenceFilterProperties(m, "DepthConfidenceFilterProperties", DOC(dai, DepthConfidenceFilterProperties));

    // Add node bindings
    auto sequentialDepthFilters = ADD_NODE(SequentialDepthFilters);
    auto depthConfidenceFilter = ADD_NODE(DepthConfidenceFilter);

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

    // StereoDepthFilterPipeline bindings
    sequentialDepthFilters
        .def_readonly("input", &SequentialDepthFilters::input, DOC(dai, node, SequentialDepthFilters, input))
        .def_readonly("output", &SequentialDepthFilters::output, DOC(dai, node, SequentialDepthFilters, output))
        .def("setRunOnHost", &SequentialDepthFilters::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, SequentialDepthFilters, setRunOnHost))
        .def("runOnHost", &SequentialDepthFilters::runOnHost, DOC(dai, node, SequentialDepthFilters, runOnHost))
        .def("addFilter", &SequentialDepthFilters::addFilter, py::arg("filter"), DOC(dai, node, SequentialDepthFilters, addFilter));

    // Just an alias for the filter stereo depth config parameters
    sequentialDepthFilters.attr("MedianFilterParams") = m.attr("MedianFilter");
    sequentialDepthFilters.attr("SpatialFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpatialFilter");
    sequentialDepthFilters.attr("SpeckleFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpeckleFilter");
    sequentialDepthFilters.attr("TemporalFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("TemporalFilter");

    // DepthConfidenceFilter bindings
    depthConfidenceFilter
        .def_readonly("depth", &DepthConfidenceFilter::depth, DOC(dai, node, DepthConfidenceFilter, depth))
        .def_readonly("amplitude", &DepthConfidenceFilter::amplitude, DOC(dai, node, DepthConfidenceFilter, amplitude))
        .def_readonly("filtered_depth", &DepthConfidenceFilter::filtered_depth, DOC(dai, node, DepthConfidenceFilter, filtered_depth))
        .def_readonly("confidence", &DepthConfidenceFilter::confidence, DOC(dai, node, DepthConfidenceFilter, confidence))
        .def("setConfidenceThreshold", &DepthConfidenceFilter::setConfidenceThreshold, py::arg("threshold"), DOC(dai, node, DepthConfidenceFilter, setConfidenceThreshold))
        .def("getConfidenceThreshold", &DepthConfidenceFilter::getConfidenceThreshold, DOC(dai, node, DepthConfidenceFilter, getConfidenceThreshold));
}
