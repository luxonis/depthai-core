#include <pybind11/stl.h>

#include "Common.hpp"
#include "DatatypeBindings.hpp"
#include "depthai/pipeline/node/DepthFilters.hpp"

void bind_depthfilters(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace node;

    // bind properties
    py::class_<SequentialDepthFiltersProperties> sequentialDepthFiltersProperties(
        m, "SequentialDepthFiltersProperties", DOC(dai, SequentialDepthFiltersProperties));
    py::class_<DepthConfidenceFilterProperties> depthConfidenceFilterProperties(
        m, "DepthConfidenceFilterProperties", DOC(dai, DepthConfidenceFilterProperties));

    // bind config
    py::class_<SequentialDepthFiltersConfig, Py<SequentialDepthFiltersConfig>, Buffer, std::shared_ptr<SequentialDepthFiltersConfig>>
        sequentialDepthFiltersConfig(m, "SequentialDepthFiltersConfig", DOC(dai, SequentialDepthFiltersConfig));
    sequentialDepthFiltersConfig.def(py::init<>());
    sequentialDepthFiltersConfig.def_readwrite("filterIndex", &SequentialDepthFiltersConfig::filterIndex, DOC(dai, SequentialDepthFiltersConfig, filterIndex));
    sequentialDepthFiltersConfig.def_readwrite(
        "filterParams", &SequentialDepthFiltersConfig::filterParams, DOC(dai, SequentialDepthFiltersConfig, filterParams));

    py::class_<DepthConfidenceFilterConfig, Py<DepthConfidenceFilterConfig>, Buffer, std::shared_ptr<DepthConfidenceFilterConfig>>
        depthConfidenceFilterConfig(m, "DepthConfidenceFilterConfig", DOC(dai, DepthConfidenceFilterConfig));
    depthConfidenceFilterConfig.def(py::init<>());
    depthConfidenceFilterConfig.def_readwrite("confidenceThreshold", &DepthConfidenceFilterConfig::confidenceThreshold, DOC(dai, DepthConfidenceFilterConfig, confidenceThreshold));

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
    sequentialDepthFilters.def_readonly("input", &SequentialDepthFilters::input, DOC(dai, node, SequentialDepthFilters, input))
        .def_readonly("output", &SequentialDepthFilters::output, DOC(dai, node, SequentialDepthFilters, output))
        .def_readonly("config", &SequentialDepthFilters::config, DOC(dai, node, SequentialDepthFilters, config))
        .def("setRunOnHost", &SequentialDepthFilters::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, SequentialDepthFilters, setRunOnHost))
        .def("runOnHost", &SequentialDepthFilters::runOnHost, DOC(dai, node, SequentialDepthFilters, runOnHost))
        .def("addFilter", &SequentialDepthFilters::addFilter, py::arg("filter"), DOC(dai, node, SequentialDepthFilters, addFilter));

    // Add MedianFilterParams
    py::class_<MedianFilterParams> medianFilterParams(m, "MedianFilterParams", DOC(dai, MedianFilterParams));
    medianFilterParams.def(py::init<>());
    medianFilterParams.def_readwrite("enable", &MedianFilterParams::enable, DOC(dai, MedianFilterParams, enable));
    medianFilterParams.def_readwrite("median", &MedianFilterParams::median, DOC(dai, MedianFilterParams, median));

    // Just an alias for the filter stereo depth config parameters
    sequentialDepthFilters.attr("MedianFilterParams") = medianFilterParams;
    sequentialDepthFilters.attr("SpatialFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpatialFilter");
    sequentialDepthFilters.attr("SpeckleFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpeckleFilter");
    sequentialDepthFilters.attr("TemporalFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("TemporalFilter");

    // DepthConfidenceFilter bindings
    depthConfidenceFilter.def_readonly("depth", &DepthConfidenceFilter::depth, DOC(dai, node, DepthConfidenceFilter, depth))
        .def_readonly("amplitude", &DepthConfidenceFilter::amplitude, DOC(dai, node, DepthConfidenceFilter, amplitude))
        .def_readonly("filteredDepth", &DepthConfidenceFilter::filteredDepth, DOC(dai, node, DepthConfidenceFilter, filteredDepth))
        .def_readonly("confidence", &DepthConfidenceFilter::confidence, DOC(dai, node, DepthConfidenceFilter, confidence))
        .def_readonly("config", &DepthConfidenceFilter::config, DOC(dai, node, DepthConfidenceFilter, config))
        .def("setConfidenceThreshold",
             &DepthConfidenceFilter::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, node, DepthConfidenceFilter, setConfidenceThreshold))
        .def("getConfidenceThreshold", &DepthConfidenceFilter::getConfidenceThreshold, DOC(dai, node, DepthConfidenceFilter, getConfidenceThreshold));
}
