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

    // ---------------------------------------------------------------------
    // ENUM: PatchColoringType
    // ---------------------------------------------------------------------
    py::enum_<VppConfig::PatchColoringType>(m, "VppPatchColoringType", DOC(dai, VppConfig, PatchColoringType))
        .value("RANDOM", VppConfig::PatchColoringType::RANDOM, "Random patch coloring")
        .value("MAXDIST", VppConfig::PatchColoringType::MAXDIST, "Color with most distant color");

    // ---------------------------------------------------------------------
    // STRUCT: InjectionParameters
    // ---------------------------------------------------------------------
    py::class_<VppConfig::InjectionParameters>(m, "VppInjectionParameters", DOC(dai, VppConfig, InjectionParameters))
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
            return "<VppInjectionParameters useInjection=" + std::to_string(p.useInjection) + ", kernelSize=" + std::to_string(p.kernelSize)
                   + ", textureThreshold=" + std::to_string(p.textureThreshold) + ", confidenceThreshold=" + std::to_string(p.confidenceThreshold)
                   + ", morphologyIterations=" + std::to_string(p.morphologyIterations) + ", useMorphology=" + std::to_string(p.useMorphology) + ">";
        });

    // ---------------------------------------------------------------------
    // CLASS: VppConfig
    // ---------------------------------------------------------------------
    py::class_<VppConfig, Py<VppConfig>, Buffer, std::shared_ptr<VppConfig>> vppConfig(m, "VppConfig", DOC(dai, VppConfig));

    // Bindings setup continuation (DepthAI callstack pattern)
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // ---------------------------------------------------------------------
    // Actual bindings
    // ---------------------------------------------------------------------
    vppConfig.def(py::init<>())
        .def("__repr__", &VppConfig::str)
        .def_readwrite("blending", &VppConfig::blending, DOC(dai, VppConfig, blending))
        .def_readwrite("distanceGamma", &VppConfig::distanceGamma, DOC(dai, VppConfig, distanceGamma))
        .def_readwrite("maxPatchSize", &VppConfig::maxPatchSize, DOC(dai, VppConfig, maxPatchSize))
        .def_readwrite("patchColoringType", &VppConfig::patchColoringType, DOC(dai, VppConfig, patchColoringType))
        .def_readwrite("uniformPatch", &VppConfig::uniformPatch, DOC(dai, VppConfig, uniformPatch))
        .def_readwrite("injectionParameters", &VppConfig::injectionParameters, DOC(dai, VppConfig, injectionParameters));
}
