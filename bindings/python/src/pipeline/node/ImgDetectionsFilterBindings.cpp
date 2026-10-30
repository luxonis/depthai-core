#include "DatatypeBindings.hpp"
#include "depthai/pipeline/node/ImgDetectionsFilter.hpp"
#include "pipeline/node/Common.hpp"

void bind_imgdetectionsfilter(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    py::class_<ImgDetectionsFilterProperties> properties(m, "ImgDetectionsFilterProperties", DOC(dai, ImgDetectionsFilterProperties));
    auto node = ADD_NODE(ImgDetectionsFilter);
    Callstack* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    properties.def_readwrite("initialConfig", &ImgDetectionsFilterProperties::initialConfig, DOC(dai, ImgDetectionsFilterProperties, initialConfig));
    node.def_readonly("inputs", &ImgDetectionsFilter::inputs, DOC(dai, node, ImgDetectionsFilter, inputs))
        .def_readonly("inputSourceMasks", &ImgDetectionsFilter::inputSourceMasks, DOC(dai, node, ImgDetectionsFilter, inputSourceMasks))
        .def_readonly("inputReference", &ImgDetectionsFilter::inputReference, DOC(dai, node, ImgDetectionsFilter, inputReference))
        .def_readonly("inputConfig", &ImgDetectionsFilter::inputConfig, DOC(dai, node, ImgDetectionsFilter, inputConfig))
        .def_readonly("out", &ImgDetectionsFilter::out, DOC(dai, node, ImgDetectionsFilter, out))
        .def_readonly("initialConfig", &ImgDetectionsFilter::initialConfig, DOC(dai, node, ImgDetectionsFilter, initialConfig))
        .def("setRunOnHost",
             &ImgDetectionsFilter::setRunOnHost,
             py::arg("runOnHost"),
             py::return_value_policy::reference_internal,
             DOC(dai, node, ImgDetectionsFilter, setRunOnHost))
        .def("runOnHost", &ImgDetectionsFilter::runOnHost, DOC(dai, node, ImgDetectionsFilter, runOnHost));
    node.attr("Properties") = properties;
    node.attr("Config") = m.attr("ImgDetectionsFilterConfig");
}
