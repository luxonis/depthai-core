#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ThermalConfig.hpp"

// #include "spdlog/spdlog.h"

void bind_thermalconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ThermalConfig, Py<ThermalConfig>, Buffer, std::shared_ptr<ThermalConfig>> thermalConfig(m, "ThermalConfig", DOC(dai, ThermalConfig));
    py::class_<ThermalImageParams, Py<ThermalImageParams>, Buffer, std::shared_ptr<ThermalImageParams>> thermalImageParams(
        m, "ThermalImageParams", DOC(dai, ThermalImageParams));
    py::class_<ThermalFFCParams, Py<ThermalFFCParams>, Buffer, std::shared_ptr<ThermalFFCParams>> thermalFFCParams(
        m, "ThermalFFCParams", DOC(dai, ThermalFFCParams));
    py::class_<ThermalAmbientParams, Py<ThermalAmbientParams>, Buffer, std::shared_ptr<ThermalAmbientParams>> thermalAmbientParams(
        m, "ThermalAmbientParams", DOC(dai, ThermalAmbientParams));

    py::enum_<ThermalGainMode> thermalGainMode(m, "ThermalGainMode", DOC(dai, ThermalGainMode));

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
        .def_readwrite("distance", &ThermalAmbientParams::distance, DOC(dai, ThermalAmbientParams, distance))
        .def_readwrite("reflectionTemperature", &ThermalAmbientParams::reflectionTemperature, DOC(dai, ThermalAmbientParams, reflectionTemperature))
        .def_readwrite("atmosphericTemperature", &ThermalAmbientParams::atmosphericTemperature, DOC(dai, ThermalAmbientParams, atmosphericTemperature))
        .def_readwrite("targetEmissivity", &ThermalAmbientParams::targetEmissivity, DOC(dai, ThermalAmbientParams, targetEmissivity))
        .def_readwrite("atmosphericTransmittance", &ThermalAmbientParams::atmosphericTransmittance, DOC(dai, ThermalAmbientParams, atmosphericTransmittance))
        .def_readwrite("gainMode", &ThermalAmbientParams::gainMode, DOC(dai, ThermalAmbientParams, gainMode));

    thermalFFCParams.def(py::init<>())
        .def_readwrite("autoFFC", &ThermalFFCParams::autoFFC, DOC(dai, ThermalFFCParams, autoFFC))
        .def_readwrite("minFFCInterval", &ThermalFFCParams::minFFCInterval, DOC(dai, ThermalFFCParams, minFFCInterval))
        .def_readwrite("maxFFCInterval", &ThermalFFCParams::maxFFCInterval, DOC(dai, ThermalFFCParams, maxFFCInterval))
        .def_readwrite("autoFFCTempThreshold", &ThermalFFCParams::autoFFCTempThreshold, DOC(dai, ThermalFFCParams, autoFFCTempThreshold))
        .def_readwrite("fallProtection", &ThermalFFCParams::fallProtection, DOC(dai, ThermalFFCParams, fallProtection))
        .def_readwrite("minShutterInterval", &ThermalFFCParams::minShutterInterval, DOC(dai, ThermalFFCParams, minShutterInterval))
        .def_readwrite("closeManualShutter", &ThermalFFCParams::closeManualShutter, DOC(dai, ThermalFFCParams, closeManualShutter))
        .def_readwrite("antiFallProtectionThresholdHighGainMode",
                       &ThermalFFCParams::antiFallProtectionThresholdHighGainMode,
                       DOC(dai, ThermalFFCParams, antiFallProtectionThresholdHighGainMode))
        .def_readwrite("antiFallProtectionThresholdLowGainMode",
                       &ThermalFFCParams::antiFallProtectionThresholdLowGainMode,
                       DOC(dai, ThermalFFCParams, antiFallProtectionThresholdLowGainMode));

    thermalImageParams.def(py::init<>())
        .def_readwrite("timeNoiseFilterLevel", &ThermalImageParams::timeNoiseFilterLevel, DOC(dai, ThermalImageParams, timeNoiseFilterLevel))
        .def_readwrite("spatialNoiseFilterLevel", &ThermalImageParams::spatialNoiseFilterLevel, DOC(dai, ThermalImageParams, spatialNoiseFilterLevel))
        .def_readwrite("digitalDetailEnhanceLevel", &ThermalImageParams::digitalDetailEnhanceLevel, DOC(dai, ThermalImageParams, digitalDetailEnhanceLevel))
        .def_readwrite("brightnessLevel", &ThermalImageParams::brightnessLevel, DOC(dai, ThermalImageParams, brightnessLevel))
        .def_readwrite("contrastLevel", &ThermalImageParams::contrastLevel, DOC(dai, ThermalImageParams, contrastLevel))
        .def_readwrite("orientation", &ThermalImageParams::orientation, DOC(dai, ThermalImageParams, orientation));

    thermalGainMode.value("LOW", ThermalGainMode::LOW, DOC(dai, ThermalGainMode, LOW)).value("HIGH", ThermalGainMode::HIGH, DOC(dai, ThermalGainMode, HIGH));

    // ThermalConfig bindings
    thermalConfig.def(py::init<>())
        .def_readwrite("ambientParams", &ThermalConfig::ambientParams, DOC(dai, ThermalConfig, ambientParams))
        .def_readwrite("ffcParams", &ThermalConfig::ffcParams, DOC(dai, ThermalConfig, ffcParams))
        .def_readwrite("imageParams", &ThermalConfig::imageParams, DOC(dai, ThermalConfig, imageParams));
}
