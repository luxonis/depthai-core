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
    py::class_<ToFIppConfig> tofIppConfig(m, "ToFIppConfig");
    py::class_<ToFDecoderConfig> tofDecoderConfig(m, "ToFDecoderConfig");

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
    toFConfig.def(py::init<>())
        .def("__repr__", &ToFConfig::str)
        // .def(py::init<std::shared_ptr<ToFConfig>>())

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
        .def_readwrite("enableBilateralFilter", &ToFConfig::enableBilateralFilter, DOC(dai, ToFConfig, enableBilateralFilter))
        .def_readwrite("bilateralStdFactor", &ToFConfig::bilateralStdFactor, DOC(dai, ToFConfig, bilateralStdFactor))
        .def_readwrite("bilateralFilterKernelSize", &ToFConfig::bilateralFilterKernelSize, DOC(dai, ToFConfig, bilateralFilterKernelSize))
        .def_readwrite("enableTemporalNoiseReduction", &ToFConfig::enableTemporalNoiseReduction, DOC(dai, ToFConfig, enableTemporalNoiseReduction))
        .def_readwrite("tnrMaxGain", &ToFConfig::tnrMaxGain, DOC(dai, ToFConfig, tnrMaxGain))
        .def_readwrite("tnrStdFactor", &ToFConfig::tnrStdFactor, DOC(dai, ToFConfig, tnrStdFactor))
        .def_readwrite("enableFlyingPixelCorrection", &ToFConfig::enableFlyingPixelCorrection, DOC(dai, ToFConfig, enableFlyingPixelCorrection))
        .def_readwrite("fpDepthThreshold", &ToFConfig::fpDepthThreshold, DOC(dai, ToFConfig, fpDepthThreshold))
        .def_readwrite("fpMinDepthOccurrence", &ToFConfig::fpMinDepthOccurrence, DOC(dai, ToFConfig, fpMinDepthOccurrence))
        .def_readwrite("enableRadialToPerp", &ToFConfig::enableRadialToPerp, DOC(dai, ToFConfig, enableRadialToPerp))

        .def("setMedianFilter", &ToFConfig::setMedianFilter, DOC(dai, ToFConfig, setMedianFilter))
        .def("setProfilePreset", &ToFConfig::setProfilePreset, DOC(dai, ToFConfig, setProfilePreset))
        .def("setToFPreset", &ToFConfig::setToFPreset, "Set RVC4 IPP preset");

    tofIppConfig.def(py::init<>())
        .def_readwrite("phaseUnwrapErrorThreshold", &ToFIppConfig::phaseUnwrapErrorThreshold)
        .def_readwrite("enableBilateralFilter", &ToFIppConfig::enableBilateralFilter)
        .def_readwrite("bilateralStdFactor", &ToFIppConfig::bilateralStdFactor)
        .def_readwrite("bilateralFilterKernelSize", &ToFIppConfig::bilateralFilterKernelSize)
        .def_readwrite("enableTemporalNoiseReduction", &ToFIppConfig::enableTemporalNoiseReduction)
        .def_readwrite("tnrMaxGain", &ToFIppConfig::tnrMaxGain)
        .def_readwrite("tnrStdFactor", &ToFIppConfig::tnrStdFactor)
        .def_readwrite("enableFlyingPixelCorrection", &ToFIppConfig::enableFlyingPixelCorrection)
        .def_readwrite("fpDepthThreshold", &ToFIppConfig::fpDepthThreshold)
        .def_readwrite("fpMinDepthOccurrence", &ToFIppConfig::fpMinDepthOccurrence)
        .def_readwrite("enableRadialToPerp", &ToFIppConfig::enableRadialToPerp)
        .def_static("fromToFConfig", &ToFIppConfig::fromToFConfig)
        .def("applyTo", &ToFIppConfig::applyTo)
        .def("applyPreset", &ToFIppConfig::applyPreset);

    tofDecoderConfig.def(py::init<>())
        .def_readwrite("median", &ToFDecoderConfig::median)
        .def_readwrite("phaseUnwrappingLevel", &ToFDecoderConfig::phaseUnwrappingLevel)
        .def_readwrite("phaseUnwrapErrorThreshold", &ToFDecoderConfig::phaseUnwrapErrorThreshold)
        .def_readwrite("enablePhaseShuffleTemporalFilter", &ToFDecoderConfig::enablePhaseShuffleTemporalFilter)
        .def_readwrite("enableBurstMode", &ToFDecoderConfig::enableBurstMode)
        .def_readwrite("enableDistortionCorrection", &ToFDecoderConfig::enableDistortionCorrection)
        .def_readwrite("enableFPPNCorrection", &ToFDecoderConfig::enableFPPNCorrection)
        .def_readwrite("enableOpticalCorrection", &ToFDecoderConfig::enableOpticalCorrection)
        .def_readwrite("enableTemperatureCorrection", &ToFDecoderConfig::enableTemperatureCorrection)
        .def_readwrite("enableWiggleCorrection", &ToFDecoderConfig::enableWiggleCorrection)
        .def_readwrite("enablePhaseUnwrapping", &ToFDecoderConfig::enablePhaseUnwrapping)
        .def_static("fromToFConfig", &ToFDecoderConfig::fromToFConfig)
        .def("applyTo", &ToFDecoderConfig::applyTo)
        .def("applyPreset", &ToFDecoderConfig::applyPreset);

    // add aliases
    // m.attr("ToFConfig").attr("DepthParams") = m.attr("ToFConfig").attr("DepthParams");
}
