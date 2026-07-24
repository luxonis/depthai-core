#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ToFConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_tofconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ToFConfig, Py<ToFConfig>, Buffer, std::shared_ptr<ToFConfig>> toFConfig(m, "ToFConfig", DOC(dai, ToFConfig));
    py::enum_<ToFConfig::Profile> toFConfigProfile(toFConfig, "Profile", DOC(dai, ToFConfig, Profile));
    py::enum_<ToFConfig::PipeType> toFConfigPipeType(toFConfig, "PipeType", DOC(dai, ToFConfig, PipeType));

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
    toFConfigProfile.value("LOW_RANGE", ToFConfig::Profile::LOW_RANGE)
        .value("MID_RANGE", ToFConfig::Profile::MID_RANGE)
        .value("HIGH_RANGE", ToFConfig::Profile::HIGH_RANGE)
        .export_values();

    toFConfigPipeType.value("AUTO", ToFConfig::PipeType::AUTO)
        .value("FLOOD", ToFConfig::PipeType::FLOOD)
        .value("DOT", ToFConfig::PipeType::DOT)
        .export_values();

    toFConfig.def(py::init<>())
        .def("__repr__", &ToFConfig::str)
        // .def(py::init<std::shared_ptr<ToFConfig>>())
        .def_property(
            "profile",
            [](const ToFConfig& self) { return self.profile; },
            [](ToFConfig& self, ToFConfig::Profile profile) { self.setProfilePreset(profile); },
            DOC(dai, ToFConfig, profile))
        .def_readwrite("median", &ToFConfig::median, DOC(dai, ToFConfig, median))
        .def_readwrite("enablePhaseShuffleTemporalFilter", &ToFConfig::enablePhaseShuffleTemporalFilter, DOC(dai, ToFConfig, enablePhaseShuffleTemporalFilter))
        .def_readwrite("enableBurstMode", &ToFConfig::enableBurstMode, DOC(dai, ToFConfig, enableBurstMode))
        .def_readwrite("enableDistortionCorrection", &ToFConfig::enableDistortionCorrection, DOC(dai, ToFConfig, enableDistortionCorrection))
        .def_readwrite("phaseUnwrappingLevel", &ToFConfig::phaseUnwrappingLevel, DOC(dai, ToFConfig, phaseUnwrappingLevel))
        .def_readwrite("enableFPPNCorrection", &ToFConfig::enableFPPNCorrection, DOC(dai, ToFConfig, enableFPPNCorrection))
        .def_readwrite("enableOpticalCorrection", &ToFConfig::enableOpticalCorrection, DOC(dai, ToFConfig, enableOpticalCorrection))
        .def_readwrite("enableTemperatureCorrection", &ToFConfig::enableTemperatureCorrection, DOC(dai, ToFConfig, enableTemperatureCorrection))
        .def_readwrite("enableWiggleCorrection", &ToFConfig::enableWiggleCorrection, DOC(dai, ToFConfig, enableWiggleCorrection))
        .def_readwrite("enablePhaseUnwrapping", &ToFConfig::enablePhaseUnwrapping, DOC(dai, ToFConfig, enablePhaseUnwrapping))
        .def_readwrite("phaseUnwrapErrorThreshold", &ToFConfig::phaseUnwrapErrorThreshold, DOC(dai, ToFConfig, phaseUnwrapErrorThreshold))

        // RVC4 / VD55H1 depth post-processing tuning (only honored on RVC4)
        .def_readwrite("enableBilateralFilter", &ToFConfig::enableBilateralFilter, DOC(dai, ToFConfig, enableBilateralFilter))
        .def_readwrite("bilateralStdFactor", &ToFConfig::bilateralStdFactor, DOC(dai, ToFConfig, bilateralStdFactor))
        .def_readwrite("bilateralKernelSize", &ToFConfig::bilateralKernelSize, DOC(dai, ToFConfig, bilateralKernelSize))
        .def_readwrite("enableTemporalNoiseReduction", &ToFConfig::enableTemporalNoiseReduction, DOC(dai, ToFConfig, enableTemporalNoiseReduction))
        .def_readwrite("tnrMaxGain", &ToFConfig::tnrMaxGain, DOC(dai, ToFConfig, tnrMaxGain))
        .def_readwrite("tnrStdFactor", &ToFConfig::tnrStdFactor, DOC(dai, ToFConfig, tnrStdFactor))
        .def_readwrite("enableFlyingPixelFilter", &ToFConfig::enableFlyingPixelFilter, DOC(dai, ToFConfig, enableFlyingPixelFilter))
        .def_readwrite("flyingPixelDepthThreshold", &ToFConfig::flyingPixelDepthThreshold, DOC(dai, ToFConfig, flyingPixelDepthThreshold))
        .def_readwrite("flyingPixelMinDepthOccurrence", &ToFConfig::flyingPixelMinDepthOccurrence, DOC(dai, ToFConfig, flyingPixelMinDepthOccurrence))
        .def_readwrite("pipeType", &ToFConfig::pipeType, DOC(dai, ToFConfig, pipeType))

        .def("setMedianFilter", &ToFConfig::setMedianFilter, DOC(dai, ToFConfig, setMedianFilter))
        .def("setProfilePreset", &ToFConfig::setProfilePreset, DOC(dai, ToFConfig, setProfilePreset))

        // .def("set", &ToFConfig::set, py::arg("config"), DOC(dai, ToFConfig, set))
        // .def("get", &ToFConfig::get, DOC(dai, ToFConfig, get))
        ;

    // add aliases
    // m.attr("ToFConfig").attr("DepthParams") = m.attr("ToFConfig").attr("DepthParams");
}
