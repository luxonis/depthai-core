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
    py::class_<ToFConfig::S5K33D> s5k33d(toFConfig, "S5K33D", DOC(dai, ToFConfig, S5K33D));
    toFConfig.attr("S5K63D") = s5k33d;
    py::class_<ToFConfig::VD55H1> vd55h1(toFConfig, "VD55H1", DOC(dai, ToFConfig, VD55H1));

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

    toFConfig.def(py::init<>())
        .def("__repr__", &ToFConfig::str)
        // .def(py::init<std::shared_ptr<ToFConfig>>())
        .def_property(
            "profile",
            [](const ToFConfig& self) { return self.profile; },
            [](ToFConfig& self, ToFConfig::Profile profile) { self.setProfilePreset(profile); },
            DOC(dai, ToFConfig, profile))
        .def_readwrite("median", &ToFConfig::median, DOC(dai, ToFConfig, median))
        .def_readwrite("enableDistortionCorrection", &ToFConfig::enableDistortionCorrection, DOC(dai, ToFConfig, enableDistortionCorrection))
        .def_readwrite("s5k33d", &ToFConfig::s5k33d, DOC(dai, ToFConfig, s5k33d))
        .def_readwrite("s5k63d", &ToFConfig::s5k33d, DOC(dai, ToFConfig, s5k63d))
        .def_readwrite("vd55h1", &ToFConfig::vd55h1, DOC(dai, ToFConfig, vd55h1))

        .def("setMedianFilter", &ToFConfig::setMedianFilter, DOC(dai, ToFConfig, setMedianFilter))
        .def("setProfilePreset", &ToFConfig::setProfilePreset, DOC(dai, ToFConfig, setProfilePreset))

        // .def("set", &ToFConfig::set, py::arg("config"), DOC(dai, ToFConfig, set))
        // .def("get", &ToFConfig::get, DOC(dai, ToFConfig, get))
        ;

    auto bindLegacyField = [&](const char* name, auto member, const char* message, const char* doc) {
        using Value = std::decay_t<decltype(std::declval<ToFConfig>().*member)>;
        toFConfig.def_property(
            name,
            [member](const ToFConfig& self) { return self.*member; },
            [member, message](ToFConfig& self, Value value) {
                if(value.has_value() && PyErr_WarnEx(PyExc_DeprecationWarning, message, 1) < 0) {
                    throw py::error_already_set();
                }
                self.*member = value;
            },
            doc);
    };
    bindLegacyField("phaseUnwrappingLevel",
                    &ToFConfig::phaseUnwrappingLevel,
                    "ToFConfig.phaseUnwrappingLevel is deprecated; use ToFConfig.s5k33d.phaseUnwrappingLevel (or s5k63d) instead.",
                    DOC(dai, ToFConfig, phaseUnwrappingLevel));
    bindLegacyField("phaseUnwrapErrorThreshold",
                    &ToFConfig::phaseUnwrapErrorThreshold,
                    "ToFConfig.phaseUnwrapErrorThreshold is deprecated; use ToFConfig.s5k33d.phaseUnwrapErrorThreshold (or s5k63d) instead.",
                    DOC(dai, ToFConfig, phaseUnwrapErrorThreshold));
    bindLegacyField("enablePhaseShuffleTemporalFilter",
                    &ToFConfig::enablePhaseShuffleTemporalFilter,
                    "ToFConfig.enablePhaseShuffleTemporalFilter is deprecated; use ToFConfig.s5k33d.enablePhaseShuffleTemporalFilter (or s5k63d) instead.",
                    DOC(dai, ToFConfig, enablePhaseShuffleTemporalFilter));
    bindLegacyField("enableBurstMode",
                    &ToFConfig::enableBurstMode,
                    "ToFConfig.enableBurstMode is deprecated; use ToFConfig.s5k33d.enableBurstMode (or s5k63d) instead.",
                    DOC(dai, ToFConfig, enableBurstMode));
    bindLegacyField("enableFPPNCorrection",
                    &ToFConfig::enableFPPNCorrection,
                    "ToFConfig.enableFPPNCorrection is deprecated: this control was removed and its value is ignored.",
                    DOC(dai, ToFConfig, enableFPPNCorrection));
    bindLegacyField("enableOpticalCorrection",
                    &ToFConfig::enableOpticalCorrection,
                    "ToFConfig.enableOpticalCorrection is deprecated: this control was removed and its value is ignored.",
                    DOC(dai, ToFConfig, enableOpticalCorrection));
    bindLegacyField("enableTemperatureCorrection",
                    &ToFConfig::enableTemperatureCorrection,
                    "ToFConfig.enableTemperatureCorrection is deprecated: this control was removed and its value is ignored.",
                    DOC(dai, ToFConfig, enableTemperatureCorrection));
    bindLegacyField("enableWiggleCorrection",
                    &ToFConfig::enableWiggleCorrection,
                    "ToFConfig.enableWiggleCorrection is deprecated: this control was removed and its value is ignored.",
                    DOC(dai, ToFConfig, enableWiggleCorrection));
    bindLegacyField("enablePhaseUnwrapping",
                    &ToFConfig::enablePhaseUnwrapping,
                    "ToFConfig.enablePhaseUnwrapping is deprecated: this control was removed and its value is ignored.",
                    DOC(dai, ToFConfig, enablePhaseUnwrapping));

    s5k33d.def(py::init<>())
        .def_readwrite("enablePhaseShuffleTemporalFilter",
                       &ToFConfig::S5K33D::enablePhaseShuffleTemporalFilter,
                       DOC(dai, ToFConfig, S5K33D, enablePhaseShuffleTemporalFilter))
        .def_readwrite("enableBurstMode", &ToFConfig::S5K33D::enableBurstMode, DOC(dai, ToFConfig, S5K33D, enableBurstMode))
        .def_readwrite("phaseUnwrappingLevel", &ToFConfig::S5K33D::phaseUnwrappingLevel, DOC(dai, ToFConfig, S5K33D, phaseUnwrappingLevel))
        .def_readwrite("phaseUnwrapErrorThreshold", &ToFConfig::S5K33D::phaseUnwrapErrorThreshold, DOC(dai, ToFConfig, S5K33D, phaseUnwrapErrorThreshold));

    vd55h1.def(py::init<>())
        .def_readwrite("phaseUnwrapErrorThreshold", &ToFConfig::VD55H1::phaseUnwrapErrorThreshold, DOC(dai, ToFConfig, VD55H1, phaseUnwrapErrorThreshold))
        .def_readwrite("enableBilateralFilter", &ToFConfig::VD55H1::enableBilateralFilter, DOC(dai, ToFConfig, VD55H1, enableBilateralFilter))
        .def_readwrite("bilateralStdFactor", &ToFConfig::VD55H1::bilateralStdFactor, DOC(dai, ToFConfig, VD55H1, bilateralStdFactor))
        .def_readwrite(
            "enableTemporalNoiseReduction", &ToFConfig::VD55H1::enableTemporalNoiseReduction, DOC(dai, ToFConfig, VD55H1, enableTemporalNoiseReduction))
        .def_readwrite(
            "temporalNoiseReductionMaxGain", &ToFConfig::VD55H1::temporalNoiseReductionMaxGain, DOC(dai, ToFConfig, VD55H1, temporalNoiseReductionMaxGain))
        .def_readwrite("temporalNoiseReductionStdFactor",
                       &ToFConfig::VD55H1::temporalNoiseReductionStdFactor,
                       DOC(dai, ToFConfig, VD55H1, temporalNoiseReductionStdFactor))
        .def_readwrite("enableFlyingPixelFilter", &ToFConfig::VD55H1::enableFlyingPixelFilter, DOC(dai, ToFConfig, VD55H1, enableFlyingPixelFilter))
        .def_readwrite("flyingPixelDepthThreshold", &ToFConfig::VD55H1::flyingPixelDepthThreshold, DOC(dai, ToFConfig, VD55H1, flyingPixelDepthThreshold))
        .def_readwrite(
            "flyingPixelMinDepthOccurrence", &ToFConfig::VD55H1::flyingPixelMinDepthOccurrence, DOC(dai, ToFConfig, VD55H1, flyingPixelMinDepthOccurrence));

    // add aliases
    // m.attr("ToFConfig").attr("DepthParams") = m.attr("ToFConfig").attr("DepthParams");
}
