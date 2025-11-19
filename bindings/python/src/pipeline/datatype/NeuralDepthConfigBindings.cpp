#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_neuraldepthconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NeuralDepthConfig, Py<NeuralDepthConfig>, Buffer, std::shared_ptr<NeuralDepthConfig>> neuralDepthConfig(
        m, "NeuralDepthConfig", DOC(dai, NeuralDepthConfig));

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
    neuralDepthConfig.def(py::init<>())
        .def("__repr__", &NeuralDepthConfig::str)
        .def("setConfidenceThreshold", &NeuralDepthConfig::setConfidenceThreshold, DOC(dai, NeuralDepthConfig, setConfidenceThreshold))
        .def("getConfidenceThreshold", &NeuralDepthConfig::getConfidenceThreshold, DOC(dai, NeuralDepthConfig, getConfidenceThreshold))
        .def("setEdgeThreshold", &NeuralDepthConfig::setEdgeThreshold, DOC(dai, NeuralDepthConfig, setEdgeThreshold))
        .def("getEdgeThreshold", &NeuralDepthConfig::getEdgeThreshold, DOC(dai, NeuralDepthConfig, getEdgeThreshold))
        .def("setDepthUnit", &NeuralDepthConfig::setDepthUnit, DOC(dai, NeuralDepthConfig, setDepthUnit))
        .def("getDepthUnit", &NeuralDepthConfig::getDepthUnit, DOC(dai, NeuralDepthConfig, getDepthUnit))
        .def("setCustomDepthUnitMultiplier", &NeuralDepthConfig::setCustomDepthUnitMultiplier, DOC(dai, NeuralDepthConfig, setCustomDepthUnitMultiplier))
        .def("getCustomDepthUnitMultiplier", &NeuralDepthConfig::getCustomDepthUnitMultiplier, DOC(dai, NeuralDepthConfig, getCustomDepthUnitMultiplier));
}
