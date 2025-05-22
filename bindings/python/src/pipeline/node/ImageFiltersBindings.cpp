#include <pybind11/stl.h>

#include "Common.hpp"
#include "DatatypeBindings.hpp"
#include "depthai/pipeline/node/ImageFilters.hpp"

void bind_imagefilters(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace node;

    // bind properties
    py::class_<ImageFiltersProperties> imageFiltersProperties(
        m, "ImageFiltersProperties", DOC(dai, ImageFiltersProperties));
    py::class_<DepthConfidenceFilterProperties> depthConfidenceFilterProperties(
        m, "DepthConfidenceFilterProperties", DOC(dai, DepthConfidenceFilterProperties));

    // bind config
    py::class_<ImageFiltersConfig, Py<ImageFiltersConfig>, Buffer, std::shared_ptr<ImageFiltersConfig>>
        imageFiltersConfig(m, "ImageFiltersConfig", DOC(dai, ImageFiltersConfig));
    imageFiltersConfig.def(py::init<>());
    imageFiltersConfig.def_readwrite("filterIndex", &ImageFiltersConfig::filterIndex, DOC(dai, ImageFiltersConfig, filterIndex));
    imageFiltersConfig.def_readwrite(
        "filterParams", &ImageFiltersConfig::filterParams, DOC(dai, ImageFiltersConfig, filterParams));

    py::class_<DepthConfidenceFilterConfig, Py<DepthConfidenceFilterConfig>, Buffer, std::shared_ptr<DepthConfidenceFilterConfig>>
        depthConfidenceFilterConfig(m, "DepthConfidenceFilterConfig", DOC(dai, DepthConfidenceFilterConfig));
    depthConfidenceFilterConfig.def(py::init<>());
    depthConfidenceFilterConfig.def_readwrite("confidenceThreshold", &DepthConfidenceFilterConfig::confidenceThreshold, DOC(dai, DepthConfidenceFilterConfig, confidenceThreshold));

    // Add node bindings
    auto imageFilters = ADD_NODE(ImageFilters);
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
    imageFilters.def_readonly("input", &ImageFilters::input, DOC(dai, node, ImageFilters, input))
        .def_readonly("output", &ImageFilters::output, DOC(dai, node, ImageFilters, output))
        .def_readonly("config", &ImageFilters::config, DOC(dai, node, ImageFilters, config))
        .def("setRunOnHost", &ImageFilters::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, ImageFilters, setRunOnHost))
        .def("runOnHost", &ImageFilters::runOnHost, DOC(dai, node, ImageFilters, runOnHost))
        .def("addFilter", &ImageFilters::addFilter, py::arg("filter"), DOC(dai, node, ImageFilters, addFilter));

    // Just an alias for the filter stereo depth config parameters
    imageFilters.attr("MedianFilterParams") = m.attr("StereoDepthConfig").attr("MedianFilter");
    imageFilters.attr("SpatialFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpatialFilter");
    imageFilters.attr("SpeckleFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpeckleFilter");
    imageFilters.attr("TemporalFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("TemporalFilter");

    // DepthConfidenceFilter bindings
    depthConfidenceFilter.def_readonly("depth", &DepthConfidenceFilter::depth, DOC(dai, node, DepthConfidenceFilter, depth))
        .def_readonly("amplitude", &DepthConfidenceFilter::amplitude, DOC(dai, node, DepthConfidenceFilter, amplitude))
        .def_readonly("filteredDepth", &DepthConfidenceFilter::filteredDepth, DOC(dai, node, DepthConfidenceFilter, filteredDepth))
        .def_readonly("confidence", &DepthConfidenceFilter::confidence, DOC(dai, node, DepthConfidenceFilter, confidence))
        .def_readonly("config", &DepthConfidenceFilter::config, DOC(dai, node, DepthConfidenceFilter, config))
        .def("setRunOnHost", &DepthConfidenceFilter::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, DepthConfidenceFilter, setRunOnHost))
        .def("runOnHost", &DepthConfidenceFilter::runOnHost, DOC(dai, node, DepthConfidenceFilter, runOnHost))
        .def("setConfidenceThreshold",
             &DepthConfidenceFilter::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, node, DepthConfidenceFilter, setConfidenceThreshold))
        .def("getConfidenceThreshold", &DepthConfidenceFilter::getConfidenceThreshold, DOC(dai, node, DepthConfidenceFilter, getConfidenceThreshold));
}
