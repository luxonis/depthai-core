#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/DetectionParserConfig.hpp"

void bind_detectionparserconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<DetectionParserConfig, Py<DetectionParserConfig>, Buffer, std::shared_ptr<DetectionParserConfig>> detectionParserConfig(
        m, "DetectionParserConfig", DOC(dai, DetectionParserConfig));

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

    // Message
    detectionParserConfig.def(py::init<>())
        .def("__repr__", &DetectionParserConfig::str)
        .def_readwrite("confidenceThreshold", &DetectionParserConfig::confidenceThreshold, DOC(dai, DetectionParserConfig, confidenceThreshold))
        .def_readwrite("iouThreshold", &DetectionParserConfig::iouThreshold, DOC(dai, DetectionParserConfig, iouThreshold))
        .def("setConfidenceThreshold",
             &DetectionParserConfig::setConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, DetectionParserConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &DetectionParserConfig::getConfidenceThreshold, DOC(dai, DetectionParserConfig, getConfidenceThreshold))
        .def("setIouThreshold", &DetectionParserConfig::setIouThreshold, py::arg("threshold"), DOC(dai, DetectionParserConfig, setIouThreshold))
        .def("getIouThreshold", &DetectionParserConfig::getIouThreshold, DOC(dai, DetectionParserConfig, getIouThreshold));
}
