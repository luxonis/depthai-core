#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"

void bind_imgdetectionsfilterconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    py::class_<ImgDetectionsFilterConfig, Py<ImgDetectionsFilterConfig>, Buffer, std::shared_ptr<ImgDetectionsFilterConfig>> config(
        m, "ImgDetectionsFilterConfig", DOC(dai, ImgDetectionsFilterConfig));
    py::enum_<ImgDetectionsFilterConfig::OverlapMode> overlap(config, "OverlapMode", DOC(dai, ImgDetectionsFilterConfig, OverlapMode));
    Callstack* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    overlap.value("OFF", ImgDetectionsFilterConfig::OverlapMode::OFF)
        .value("NMS", ImgDetectionsFilterConfig::OverlapMode::NMS)
        .value("AVERAGE", ImgDetectionsFilterConfig::OverlapMode::AVERAGE);
    config.def(py::init<>())
        .def("__repr__", &ImgDetectionsFilterConfig::str)
        .def_readwrite("labelsToKeep", &ImgDetectionsFilterConfig::labelsToKeep, DOC(dai, ImgDetectionsFilterConfig, labelsToKeep))
        .def_readwrite("labelsToReject", &ImgDetectionsFilterConfig::labelsToReject, DOC(dai, ImgDetectionsFilterConfig, labelsToReject))
        .def_readwrite("minConfidence", &ImgDetectionsFilterConfig::minConfidence, DOC(dai, ImgDetectionsFilterConfig, minConfidence))
        .def_readwrite("maxConfidence", &ImgDetectionsFilterConfig::maxConfidence, DOC(dai, ImgDetectionsFilterConfig, maxConfidence))
        .def_readwrite("minArea", &ImgDetectionsFilterConfig::minArea, DOC(dai, ImgDetectionsFilterConfig, minArea))
        .def_readwrite("maxArea", &ImgDetectionsFilterConfig::maxArea, DOC(dai, ImgDetectionsFilterConfig, maxArea))
        .def_readwrite("minWidth", &ImgDetectionsFilterConfig::minWidth, DOC(dai, ImgDetectionsFilterConfig, minWidth))
        .def_readwrite("maxWidth", &ImgDetectionsFilterConfig::maxWidth, DOC(dai, ImgDetectionsFilterConfig, maxWidth))
        .def_readwrite("minHeight", &ImgDetectionsFilterConfig::minHeight, DOC(dai, ImgDetectionsFilterConfig, minHeight))
        .def_readwrite("maxHeight", &ImgDetectionsFilterConfig::maxHeight, DOC(dai, ImgDetectionsFilterConfig, maxHeight))
        .def_readwrite("regionOfInterest", &ImgDetectionsFilterConfig::regionOfInterest, DOC(dai, ImgDetectionsFilterConfig, regionOfInterest))
        .def_readwrite("maxDetections", &ImgDetectionsFilterConfig::maxDetections, DOC(dai, ImgDetectionsFilterConfig, maxDetections))
        .def_readwrite("sortByConfidence", &ImgDetectionsFilterConfig::sortByConfidence, DOC(dai, ImgDetectionsFilterConfig, sortByConfidence))
        .def_readwrite("overlapMode", &ImgDetectionsFilterConfig::overlapMode, DOC(dai, ImgDetectionsFilterConfig, overlapMode))
        .def_readwrite("overlapIouThreshold", &ImgDetectionsFilterConfig::overlapIouThreshold, DOC(dai, ImgDetectionsFilterConfig, overlapIouThreshold))
        .def_readwrite("reference", &ImgDetectionsFilterConfig::reference, DOC(dai, ImgDetectionsFilterConfig, reference))
        .def("setConfidenceRange",
             &ImgDetectionsFilterConfig::setConfidenceRange,
             py::arg("min") = 0.0f,
             py::arg("max") = 1.0f,
             py::return_value_policy::reference_internal,
             DOC(dai, ImgDetectionsFilterConfig, setConfidenceRange))
        .def("setSizeRange",
             &ImgDetectionsFilterConfig::setSizeRange,
             py::arg("minArea") = 0.0f,
             py::arg("maxArea") = std::numeric_limits<float>::max(),
             py::return_value_policy::reference_internal,
             DOC(dai, ImgDetectionsFilterConfig, setSizeRange))
        .def("setWidthRange",
             &ImgDetectionsFilterConfig::setWidthRange,
             py::arg("min") = 0.0f,
             py::arg("max") = std::numeric_limits<float>::max(),
             py::return_value_policy::reference_internal,
             DOC(dai, ImgDetectionsFilterConfig, setWidthRange))
        .def("setHeightRange",
             &ImgDetectionsFilterConfig::setHeightRange,
             py::arg("min") = 0.0f,
             py::arg("max") = std::numeric_limits<float>::max(),
             py::return_value_policy::reference_internal,
             DOC(dai, ImgDetectionsFilterConfig, setHeightRange))
        .def("validate", &ImgDetectionsFilterConfig::validate, DOC(dai, ImgDetectionsFilterConfig, validate))
        .def("hasGeometryFilters", &ImgDetectionsFilterConfig::hasGeometryFilters, DOC(dai, ImgDetectionsFilterConfig, hasGeometryFilters));
}
