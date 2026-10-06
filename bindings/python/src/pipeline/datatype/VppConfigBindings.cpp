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

    // 1. First, define the VppConfig class object so we can attach things to it
    py::class_<VppConfig, Py<VppConfig>, Buffer, std::shared_ptr<VppConfig>> vppConfig(m, "VppConfig", DOC(dai, VppConfig));

    py::enum_<VppConfig::PatchColoringType> patchColoringType(vppConfig, "PatchColoringType", DOC(dai, VppConfig, PatchColoringType));
    py::class_<VppConfig::InjectionParameters> injectionParameters(vppConfig, "InjectionParameters", DOC(dai, VppConfig, InjectionParameters));

    // 4. Handle DepthAI callstack logic
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    patchColoringType.value("RANDOM", VppConfig::PatchColoringType::RANDOM, "Random patch coloring")
        .value("MAXDIST", VppConfig::PatchColoringType::MAXDIST, "Color with most distant color")
        .export_values();

    injectionParameters.def("getUseInjection", &dai::VppConfig::InjectionParameters::getUseInjection, DOC(dai, VppConfig, InjectionParameters, getUseInjection))
        .def("setUseInjection",
             &dai::VppConfig::InjectionParameters::setUseInjection,
             py::arg("value"),
             DOC(dai, VppConfig, InjectionParameters, setUseInjection))
        .def("getKernelSize", &dai::VppConfig::InjectionParameters::getKernelSize, DOC(dai, VppConfig, InjectionParameters, getKernelSize))
        .def("setKernelSize", &dai::VppConfig::InjectionParameters::setKernelSize, py::arg("value"), DOC(dai, VppConfig, InjectionParameters, setKernelSize))
        .def("getTextureThreshold", &dai::VppConfig::InjectionParameters::getTextureThreshold, DOC(dai, VppConfig, InjectionParameters, getTextureThreshold))
        .def("setTextureThreshold",
             &dai::VppConfig::InjectionParameters::setTextureThreshold,
             py::arg("value"),
             DOC(dai, VppConfig, InjectionParameters, setTextureThreshold))
        .def("getConfidenceThreshold",
             &dai::VppConfig::InjectionParameters::getConfidenceThreshold,
             DOC(dai, VppConfig, InjectionParameters, getConfidenceThreshold))
        .def("setConfidenceThreshold",
             &dai::VppConfig::InjectionParameters::setConfidenceThreshold,
             py::arg("value"),
             DOC(dai, VppConfig, InjectionParameters, setConfidenceThreshold))
        .def("getMorphologyIterations",
             &dai::VppConfig::InjectionParameters::getMorphologyIterations,
             DOC(dai, VppConfig, InjectionParameters, getMorphologyIterations))
        .def("setMorphologyIterations",
             &dai::VppConfig::InjectionParameters::setMorphologyIterations,
             py::arg("value"),
             DOC(dai, VppConfig, InjectionParameters, setMorphologyIterations))
        .def("isUseMorphology", &dai::VppConfig::InjectionParameters::isUseMorphology, DOC(dai, VppConfig, InjectionParameters, isUseMorphology))
        .def("setUseMorphology",
             &dai::VppConfig::InjectionParameters::setUseMorphology,
             py::arg("value"),
             DOC(dai, VppConfig, InjectionParameters, setUseMorphology))
        .def(py::init<>())
        .def_readwrite("useInjection", &VppConfig::InjectionParameters::useInjection, DOC(dai, VppConfig, InjectionParameters, useInjection))
        .def_readwrite("kernelSize", &VppConfig::InjectionParameters::kernelSize, DOC(dai, VppConfig, InjectionParameters, kernelSize))
        .def_readwrite("textureThreshold", &VppConfig::InjectionParameters::textureThreshold, DOC(dai, VppConfig, InjectionParameters, textureThreshold))
        .def_readwrite(
            "confidenceThreshold", &VppConfig::InjectionParameters::confidenceThreshold, DOC(dai, VppConfig, InjectionParameters, confidenceThreshold))
        .def_readwrite(
            "morphologyIterations", &VppConfig::InjectionParameters::morphologyIterations, DOC(dai, VppConfig, InjectionParameters, morphologyIterations))
        .def_readwrite("useMorphology", &VppConfig::InjectionParameters::useMorphology, DOC(dai, VppConfig, InjectionParameters, useMorphology))
        .def("__repr__", [](const VppConfig::InjectionParameters& p) {
            return "<VppInjectionParameters useInjection=" + std::to_string(p.useInjection) + "...>";  // Simplified for brevity
        });

    // VppConfig methods and properties
    vppConfig.def("getBlending", &dai::VppConfig::getBlending, DOC(dai, VppConfig, getBlending))
        .def("setBlending", &dai::VppConfig::setBlending, py::arg("value"), DOC(dai, VppConfig, setBlending))
        .def("getDistanceGamma", &dai::VppConfig::getDistanceGamma, DOC(dai, VppConfig, getDistanceGamma))
        .def("setDistanceGamma", &dai::VppConfig::setDistanceGamma, py::arg("value"), DOC(dai, VppConfig, setDistanceGamma))
        .def("getMaxPatchSize", &dai::VppConfig::getMaxPatchSize, DOC(dai, VppConfig, getMaxPatchSize))
        .def("setMaxPatchSize", &dai::VppConfig::setMaxPatchSize, py::arg("value"), DOC(dai, VppConfig, setMaxPatchSize))
        .def("getPatchColoringType", &dai::VppConfig::getPatchColoringType, DOC(dai, VppConfig, getPatchColoringType))
        .def("setPatchColoringType", &dai::VppConfig::setPatchColoringType, py::arg("type"), DOC(dai, VppConfig, setPatchColoringType))
        .def("getUniformPatch", &dai::VppConfig::getUniformPatch, DOC(dai, VppConfig, getUniformPatch))
        .def("setUniformPatch", &dai::VppConfig::setUniformPatch, py::arg("value"), DOC(dai, VppConfig, setUniformPatch))
        .def("getInjectionParameters", &dai::VppConfig::getInjectionParameters, DOC(dai, VppConfig, getInjectionParameters))
        .def("setInjectionParameters", &dai::VppConfig::setInjectionParameters, py::arg("params"), DOC(dai, VppConfig, setInjectionParameters))
        .def("getMaxNumThreads", &dai::VppConfig::getMaxNumThreads, DOC(dai, VppConfig, getMaxNumThreads))
        .def("setMaxNumThreads", &dai::VppConfig::setMaxNumThreads, py::arg("value"), DOC(dai, VppConfig, setMaxNumThreads))
        .def("getMaxFPS", &dai::VppConfig::getMaxFPS, DOC(dai, VppConfig, getMaxFPS))
        .def("setMaxFPS", &dai::VppConfig::setMaxFPS, py::arg("value"), DOC(dai, VppConfig, setMaxFPS))
        .def(py::init<>())
        .def("__repr__", &VppConfig::str)
        .def_readwrite("blending", &VppConfig::blending, DOC(dai, VppConfig, blending))
        .def_readwrite("distanceGamma", &VppConfig::distanceGamma, DOC(dai, VppConfig, distanceGamma))
        .def_readwrite("maxPatchSize", &VppConfig::maxPatchSize, DOC(dai, VppConfig, maxPatchSize))
        .def_readwrite("patchColoringType", &VppConfig::patchColoringType, DOC(dai, VppConfig, patchColoringType))
        .def_readwrite("uniformPatch", &VppConfig::uniformPatch, DOC(dai, VppConfig, uniformPatch))
        .def_readwrite("injectionParameters", &VppConfig::injectionParameters, DOC(dai, VppConfig, injectionParameters))
        .def_readwrite("maxNumThreads", &VppConfig::maxNumThreads, DOC(dai, VppConfig, maxNumThreads))
        .def_readwrite("maxFPS", &VppConfig::maxFPS, DOC(dai, VppConfig, maxFPS));
}
