#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ThermalConfig.hpp"

// #include "spdlog/spdlog.h"

void bind_thermalconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ThermalConfig, Py<ThermalConfig>, Buffer, std::shared_ptr<ThermalConfig>> thermalConfig(m, "ThermalConfig", DOC(dai, ThermalConfig));
    py::class_<ThermalConfig::ThermalImageParams> thermalImageParams(m, "ThermalImageParams", DOC(dai, ThermalConfig, ThermalImageParams));
    py::class_<ThermalConfig::ThermalFFCParams> thermalFFCParams(m, "ThermalFFCParams", DOC(dai, ThermalConfig, ThermalFFCParams));
    py::class_<ThermalConfig::ThermalAmbientParams> thermalAmbientParams(m, "ThermalAmbientParams", DOC(dai, ThermalConfig, ThermalAmbientParams));

    py::enum_<ThermalConfig::ThermalGainMode> thermalGainMode(m, "ThermalGainMode", DOC(dai, ThermalConfig, ThermalGainMode));

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

    thermalAmbientParams.def(py::init<>())
        .def_readwrite("distance", &ThermalConfig::ThermalAmbientParams::distance, DOC(dai, ThermalConfig, ThermalAmbientParams, distance))
        .def_readwrite("reflectionTemperature",
                       &ThermalConfig::ThermalAmbientParams::reflectionTemperature,
                       DOC(dai, ThermalConfig, ThermalAmbientParams, reflectionTemperature))
        .def_readwrite("atmosphericTemperature",
                       &ThermalConfig::ThermalAmbientParams::atmosphericTemperature,
                       DOC(dai, ThermalConfig, ThermalAmbientParams, atmosphericTemperature))
        .def_readwrite(
            "targetEmissivity", &ThermalConfig::ThermalAmbientParams::targetEmissivity, DOC(dai, ThermalConfig, ThermalAmbientParams, targetEmissivity))
        .def_readwrite("atmosphericTransmittance",
                       &ThermalConfig::ThermalAmbientParams::atmosphericTransmittance,
                       DOC(dai, ThermalConfig, ThermalAmbientParams, atmosphericTransmittance))
        .def_readwrite("gainMode", &ThermalConfig::ThermalAmbientParams::gainMode, DOC(dai, ThermalConfig, ThermalAmbientParams, gainMode));

    thermalFFCParams.def(py::init<>())
        .def_readwrite("autoFFC", &ThermalConfig::ThermalFFCParams::autoFFC, DOC(dai, ThermalConfig, ThermalFFCParams, autoFFC))
        .def_readwrite("minFFCInterval", &ThermalConfig::ThermalFFCParams::minFFCInterval, DOC(dai, ThermalConfig, ThermalFFCParams, minFFCInterval))
        .def_readwrite("maxFFCInterval", &ThermalConfig::ThermalFFCParams::maxFFCInterval, DOC(dai, ThermalConfig, ThermalFFCParams, maxFFCInterval))
        .def_readwrite(
            "autoFFCTempThreshold", &ThermalConfig::ThermalFFCParams::autoFFCTempThreshold, DOC(dai, ThermalConfig, ThermalFFCParams, autoFFCTempThreshold))
        .def_readwrite("fallProtection", &ThermalConfig::ThermalFFCParams::fallProtection, DOC(dai, ThermalConfig, ThermalFFCParams, fallProtection))
        .def_readwrite(
            "minShutterInterval", &ThermalConfig::ThermalFFCParams::minShutterInterval, DOC(dai, ThermalConfig, ThermalFFCParams, minShutterInterval))
        .def_readwrite(
            "closeManualShutter", &ThermalConfig::ThermalFFCParams::closeManualShutter, DOC(dai, ThermalConfig, ThermalFFCParams, closeManualShutter))
        .def_readwrite("antiFallProtectionThresholdHighGainMode",
                       &ThermalConfig::ThermalFFCParams::antiFallProtectionThresholdHighGainMode,
                       DOC(dai, ThermalConfig, ThermalFFCParams, antiFallProtectionThresholdHighGainMode))
        .def_readwrite("antiFallProtectionThresholdLowGainMode",
                       &ThermalConfig::ThermalFFCParams::antiFallProtectionThresholdLowGainMode,
                       DOC(dai, ThermalConfig, ThermalFFCParams, antiFallProtectionThresholdLowGainMode));

    thermalImageParams.def(py::init<>())
        .def_readwrite(
            "timeNoiseFilterLevel", &ThermalConfig::ThermalImageParams::timeNoiseFilterLevel, DOC(dai, ThermalConfig, ThermalImageParams, timeNoiseFilterLevel))
        .def_readwrite("spatialNoiseFilterLevel",
                       &ThermalConfig::ThermalImageParams::spatialNoiseFilterLevel,
                       DOC(dai, ThermalConfig, ThermalImageParams, spatialNoiseFilterLevel))
        .def_readwrite("digitalDetailEnhanceLevel",
                       &ThermalConfig::ThermalImageParams::digitalDetailEnhanceLevel,
                       DOC(dai, ThermalConfig, ThermalImageParams, digitalDetailEnhanceLevel))
        .def_readwrite("brightnessLevel", &ThermalConfig::ThermalImageParams::brightnessLevel, DOC(dai, ThermalConfig, ThermalImageParams, brightnessLevel))
        .def_readwrite("contrastLevel", &ThermalConfig::ThermalImageParams::contrastLevel, DOC(dai, ThermalConfig, ThermalImageParams, contrastLevel))
        .def_readwrite("orientation", &ThermalConfig::ThermalImageParams::orientation, DOC(dai, ThermalConfig, ThermalImageParams, orientation));

    thermalGainMode.value("LOW", ThermalConfig::ThermalGainMode::LOW, DOC(dai, ThermalConfig, ThermalGainMode, LOW))
        .value("HIGH", ThermalConfig::ThermalGainMode::HIGH, DOC(dai, ThermalConfig, ThermalGainMode, HIGH));

    // ThermalConfig bindings
    thermalConfig.def(py::init<>())
        .def("__repr__", &ThermalConfig::str)
        .def_readwrite("ambientParams", &ThermalConfig::ambientParams, DOC(dai, ThermalConfig, ambientParams))
        .def_readwrite("ffcParams", &ThermalConfig::ffcParams, DOC(dai, ThermalConfig, ffcParams))
        .def_readwrite("imageParams", &ThermalConfig::imageParams, DOC(dai, ThermalConfig, imageParams));
}
