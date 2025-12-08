#include "Common.hpp"
#include "DatatypeBindings.hpp"
#include "depthai/pipeline/node/ImageFilters.hpp"

void bind_imagefilters(py::module& m, void* pCallstack) {
    using namespace dai;
    using namespace node;

    // bind properties
    py::class_<ImageFiltersProperties> imageFiltersProperties(m, "ImageFiltersProperties", DOC(dai, ImageFiltersProperties));
    py::class_<ToFDepthConfidenceFilterProperties> depthConfidenceFilterProperties(
        m, "ToFDepthConfidenceFilterProperties", DOC(dai, ToFDepthConfidenceFilterProperties));

    // bind config
    py::class_<ImageFiltersConfig, Py<ImageFiltersConfig>, Buffer, std::shared_ptr<ImageFiltersConfig>> imageFiltersConfig(
        m, "ImageFiltersConfig", DOC(dai, ImageFiltersConfig));
    imageFiltersConfig.def(py::init<>());
    imageFiltersConfig.def_readwrite("filterIndices", &ImageFiltersConfig::filterIndices, DOC(dai, ImageFiltersConfig, filterIndices));
    imageFiltersConfig.def_readwrite("filterParams", &ImageFiltersConfig::filterParams, DOC(dai, ImageFiltersConfig, filterParams));
    imageFiltersConfig.def("insertFilter", &ImageFiltersConfig::insertFilter, py::arg("params"), DOC(dai, ImageFiltersConfig, insertFilter));
    imageFiltersConfig
        .def("updateFilterAtIndex",
             &ImageFiltersConfig::updateFilterAtIndex,
             py::arg("index"),
             py::arg("params"),
             DOC(dai, ImageFiltersConfig, updateFilterAtIndex))
        .def("setProfilePreset", &ImageFiltersConfig::setProfilePreset, DOC(dai, ImageFiltersConfig, setProfilePreset));

    py::class_<ToFDepthConfidenceFilterConfig, Py<ToFDepthConfidenceFilterConfig>, Buffer, std::shared_ptr<ToFDepthConfidenceFilterConfig>>
        depthConfidenceFilterConfig(m, "ToFDepthConfidenceFilterConfig", DOC(dai, ToFDepthConfidenceFilterConfig));
    depthConfidenceFilterConfig.def(py::init<>());
    depthConfidenceFilterConfig
        .def_readwrite(
            "confidenceThreshold", &ToFDepthConfidenceFilterConfig::confidenceThreshold, DOC(dai, ToFDepthConfidenceFilterConfig, confidenceThreshold))
        .def("setProfilePreset", &ToFDepthConfidenceFilterConfig::setProfilePreset, DOC(dai, ToFDepthConfidenceFilterConfig, setProfilePreset));

    // Add node bindings
    auto imageFilters = ADD_NODE(ImageFilters);
    auto depthConfidenceFilter = ADD_NODE(ToFDepthConfidenceFilter);

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

    // Add enum for preset mode
    py::enum_<ImageFiltersPresetMode>(m, "ImageFiltersPresetMode")
        .value("TOF_LOW_RANGE", ImageFiltersPresetMode::TOF_LOW_RANGE)
        .value("TOF_MID_RANGE", ImageFiltersPresetMode::TOF_MID_RANGE)
        .value("TOF_HIGH_RANGE", ImageFiltersPresetMode::TOF_HIGH_RANGE);

    // StereoDepthFilterPipeline bindings
    imageFilters.def_readonly("input", &ImageFilters::input, DOC(dai, node, ImageFilters, input))
        .def_readonly("output", &ImageFilters::output, DOC(dai, node, ImageFilters, output))
        .def_readonly("inputConfig", &ImageFilters::inputConfig, DOC(dai, node, ImageFilters, inputConfig))
        .def_readonly("initialConfig", &ImageFilters::initialConfig, DOC(dai, node, ImageFilters, initialConfig))
        .def("setRunOnHost", &ImageFilters::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, ImageFilters, setRunOnHost))
        .def("runOnHost", &ImageFilters::runOnHost, DOC(dai, node, ImageFilters, runOnHost))
        .def("build",
             py::overload_cast<Node::Output&, ImageFiltersPresetMode>(&ImageFilters::build),
             py::arg("input"),
             py::arg("presetMode") = ImageFiltersPresetMode::TOF_MID_RANGE,
             DOC(dai, node, ImageFilters, build))
        .def("build",
             py::overload_cast<ImageFiltersPresetMode>(&ImageFilters::build),
             py::arg("presetMode") = ImageFiltersPresetMode::TOF_MID_RANGE,
             DOC(dai, node, ImageFilters, build));

    // Just an alias for the filter stereo depth config parameters
    imageFilters.attr("MedianFilterParams") = m.attr("StereoDepthConfig").attr("MedianFilter");
    imageFilters.attr("SpatialFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpatialFilter");
    imageFilters.attr("SpeckleFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpeckleFilter");
    imageFilters.attr("TemporalFilterParams") = m.attr("StereoDepthConfig").attr("PostProcessing").attr("TemporalFilter");

    // DepthConfidenceFilter bindings
    depthConfidenceFilter.def_readonly("depth", &ToFDepthConfidenceFilter::depth, DOC(dai, node, ToFDepthConfidenceFilter, depth))
        .def_readonly("amplitude", &ToFDepthConfidenceFilter::amplitude, DOC(dai, node, ToFDepthConfidenceFilter, amplitude))
        .def_readonly("filteredDepth", &ToFDepthConfidenceFilter::filteredDepth, DOC(dai, node, ToFDepthConfidenceFilter, filteredDepth))
        .def_readonly("confidence", &ToFDepthConfidenceFilter::confidence, DOC(dai, node, ToFDepthConfidenceFilter, confidence))
        .def_readonly("inputConfig", &ToFDepthConfidenceFilter::inputConfig, DOC(dai, node, ToFDepthConfidenceFilter, inputConfig))
        .def_readonly("initialConfig", &ToFDepthConfidenceFilter::initialConfig, DOC(dai, node, ToFDepthConfidenceFilter, initialConfig))
        .def("setRunOnHost", &ToFDepthConfidenceFilter::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, ToFDepthConfidenceFilter, setRunOnHost))
        .def("runOnHost", &ToFDepthConfidenceFilter::runOnHost, DOC(dai, node, ToFDepthConfidenceFilter, runOnHost))
        .def("build",
             py::overload_cast<Node::Output&, Node::Output&, ImageFiltersPresetMode>(&ToFDepthConfidenceFilter::build),
             py::arg("depth"),
             py::arg("amplitude"),
             py::arg("presetMode") = ImageFiltersPresetMode::TOF_MID_RANGE,
             DOC(dai, node, ToFDepthConfidenceFilter, build))
        .def("build",
             py::overload_cast<ImageFiltersPresetMode>(&ToFDepthConfidenceFilter::build),
             py::arg("presetMode") = ImageFiltersPresetMode::TOF_MID_RANGE,
             DOC(dai, node, ToFDepthConfidenceFilter, build));
}
