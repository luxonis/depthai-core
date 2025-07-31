#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/VppConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_vppconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    // VppConfig enum
    py::enum_<VppConfig::Method>(m, "VppMethod", DOC(dai, VppConfig, Method))
        .value("RANDOM", VppConfig::Method::RANDOM, DOC(dai, VppConfig, Method, RANDOM))
        .value("MAX_DISTANCE", VppConfig::Method::MAX_DISTANCE, DOC(dai, VppConfig, Method, MAX_DISTANCE));

    py::class_<VppConfig, Py<VppConfig>, Buffer, std::shared_ptr<VppConfig>> vppConfig(m, "VppConfig", DOC(dai, VppConfig));

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
    vppConfig.def(py::init<>())
        .def("__repr__", &VppConfig::str)

        .def_readwrite("method", &VppConfig::method, DOC(dai, VppConfig, method))
        .def_readwrite("patchSize", &VppConfig::patchSize, DOC(dai, VppConfig, patchSize))
        .def_readwrite("windowSizeAggX", &VppConfig::windowSizeAggX, DOC(dai, VppConfig, windowSizeAggX))
        .def_readwrite("windowSizeAggY", &VppConfig::windowSizeAggY, DOC(dai, VppConfig, windowSizeAggY))
        .def_readwrite("left2right", &VppConfig::left2right, DOC(dai, VppConfig, left2right))
        .def_readwrite("blending", &VppConfig::blending, DOC(dai, VppConfig, blending))
        .def_readwrite("blendingOcclusion", &VppConfig::blendingOcclusion, DOC(dai, VppConfig, blendingOcclusion))
        .def_readwrite("useDistancePatch", &VppConfig::useDistancePatch, DOC(dai, VppConfig, useDistancePatch))
        .def_readwrite("useBilateralPatch", &VppConfig::useBilateralPatch, DOC(dai, VppConfig, useBilateralPatch))
        .def_readwrite("distanceGamma", &VppConfig::distanceGamma, DOC(dai, VppConfig, distanceGamma))
        .def_readwrite("bilateralSpatialSigma", &VppConfig::bilateralSpatialSigma, DOC(dai, VppConfig, bilateralSpatialSigma))
        .def_readwrite("bilateralIntensitySigma", &VppConfig::bilateralIntensitySigma, DOC(dai, VppConfig, bilateralIntensitySigma))
        .def_readwrite("bilateralThreshold", &VppConfig::bilateralThreshold, DOC(dai, VppConfig, bilateralThreshold))
        .def_readwrite("uniformColor", &VppConfig::uniformColor, DOC(dai, VppConfig, uniformColor))
        .def_readwrite("discardOcclusion", &VppConfig::discardOcclusion, DOC(dai, VppConfig, discardOcclusion))
        .def_readwrite("interpolate", &VppConfig::interpolate, DOC(dai, VppConfig, interpolate))
        .def_readwrite("disparityMinThreshold", &VppConfig::disparityMinThreshold, DOC(dai, VppConfig, disparityMinThreshold))
        .def_readwrite("disparityMaxThreshold", &VppConfig::disparityMaxThreshold, DOC(dai, VppConfig, disparityMaxThreshold))

        .def("setMethod", &VppConfig::setMethod, "method"_a, DOC(dai, VppConfig, setMethod))
        .def("setPatchSize", &VppConfig::setPatchSize, "size"_a, DOC(dai, VppConfig, setPatchSize))
        .def("setAggregationWindowSize", &VppConfig::setAggregationWindowSize, "sizeX"_a, "sizeY"_a, DOC(dai, VppConfig, setAggregationWindowSize))
        .def("setProjectionDirection", &VppConfig::setProjectionDirection, "left2right"_a, DOC(dai, VppConfig, setProjectionDirection))
        .def("setBlending", &VppConfig::setBlending, "blending"_a, "blendingOcclusion"_a, DOC(dai, VppConfig, setBlending))
        .def("setDistancePatch", &VppConfig::setDistancePatch, "enable"_a, "gamma"_a = 0.3f, DOC(dai, VppConfig, setDistancePatch))
        .def("setBilateralPatch", &VppConfig::setBilateralPatch, "enable"_a, "spatialSigma"_a = 2.0f, "intensitySigma"_a = 1.0f, "threshold"_a = 0.001f, DOC(dai, VppConfig, setBilateralPatch))
        .def("setDisparityRange", &VppConfig::setDisparityRange, "minThreshold"_a, "maxThreshold"_a, DOC(dai, VppConfig, setDisparityRange))
        ;
}