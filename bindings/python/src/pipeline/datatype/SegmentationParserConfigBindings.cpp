#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_segmentationparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<SegmentationParserConfig, Py<SegmentationParserConfig>, Buffer, py::smart_holder> segmentationParserConfig(
        m, "SegmentationParserConfig", DOC(dai, SegmentationParserConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    segmentationParserConfig.def(py::init<>())
        .def("__repr__", &SegmentationParserConfig::str)
        .def_readwrite("confidenceThreshold", &SegmentationParserConfig::confidenceThreshold, DOC(dai, SegmentationParserConfig, confidenceThreshold))
        .def_readwrite("stepSize", &SegmentationParserConfig::stepSize, DOC(dai, SegmentationParserConfig, stepSize))
        .def("setConfidenceThreshold",
             &SegmentationParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, SegmentationParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &SegmentationParserConfig::getConfidenceThreshold, DOC(dai, SegmentationParserConfig, getConfidenceThreshold))
        .def("setStepSize", &SegmentationParserConfig::setStepSize, py::arg("stepSize"), DOC(dai, SegmentationParserConfig, setStepSize))
        .def("getStepSize", &SegmentationParserConfig::getStepSize, DOC(dai, SegmentationParserConfig, getStepSize));
}
