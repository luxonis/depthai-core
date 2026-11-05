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
    py::class_<ThermalConfig::ThermalHistEqParams> thermalHistEqParams(m, "ThermalHistEqParams", DOC(dai, ThermalConfig, ThermalHistEqParams));
    py::class_<ThermalConfig::ThermalAgcParams> thermalAgcParams(m, "ThermalAgcParams", DOC(dai, ThermalConfig, ThermalAgcParams));
    py::class_<ThermalConfig::ThermalFSCParams> thermalFSCParams(m, "ThermalFSCParams", DOC(dai, ThermalConfig, ThermalFSCParams));

    py::enum_<ThermalConfig::ThermalGainMode> thermalGainMode(m, "ThermalGainMode", DOC(dai, ThermalConfig, ThermalGainMode));
    py::enum_<ThermalConfig::ThermalImageOrientation> thermalImageOrientation(
        m, "ThermalImageOrientation", DOC(dai, ThermalConfig, ThermalImageOrientation));
    py::enum_<ThermalConfig::ThermalColorPalette> thermalColorPalette(m, "ThermalColorPalette", DOC(dai, ThermalConfig, ThermalColorPalette));
    py::enum_<ThermalConfig::ThermalPipelineMode> thermalPipelineMode(m, "ThermalPipelineMode", DOC(dai, ThermalConfig, ThermalPipelineMode));
    py::enum_<ThermalConfig::ThermalAgcMode> thermalAgcMode(m, "ThermalAgcMode", DOC(dai, ThermalConfig, ThermalAgcMode));
    py::enum_<ThermalConfig::ThermalLinearAgcLockMode> thermalLinearAgcLockMode(
        m, "ThermalLinearAgcLockMode", DOC(dai, ThermalConfig, ThermalLinearAgcLockMode));
    py::enum_<ThermalConfig::ThermalHistEqGainLimitFactorMode> thermalHistEqGainLimitFactorMode(
        m, "ThermalHistEqGainLimitFactorMode", DOC(dai, ThermalConfig, ThermalHistEqGainLimitFactorMode));
    py::enum_<ThermalConfig::ThermalHistEqPlateauRedistributionMode> thermalHistEqPlateauRedistributionMode(
        m, "ThermalHistEqPlateauRedistributionMode", DOC(dai, ThermalConfig, ThermalHistEqPlateauRedistributionMode));
    py::enum_<ThermalConfig::ThermalFSCAction> thermalFSCAction(m, "ThermalFSCAction", DOC(dai, ThermalConfig, ThermalFSCAction));

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
        .def_readwrite("gainMode", &ThermalConfig::ThermalAmbientParams::gainMode, DOC(dai, ThermalConfig, ThermalAmbientParams, gainMode))
        .def_readwrite(
            "temperatureOffset", &ThermalConfig::ThermalAmbientParams::temperatureOffset, DOC(dai, ThermalConfig, ThermalAmbientParams, temperatureOffset));

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
                       DOC(dai, ThermalConfig, ThermalFFCParams, antiFallProtectionThresholdLowGainMode))
        .def_readwrite("triggerFFC", &ThermalConfig::ThermalFFCParams::triggerFFC, DOC(dai, ThermalConfig, ThermalFFCParams, triggerFFC));

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
        .def_readwrite("orientation", &ThermalConfig::ThermalImageParams::orientation, DOC(dai, ThermalConfig, ThermalImageParams, orientation))
        .def_readwrite("colorPalette", &ThermalConfig::ThermalImageParams::colorPalette, DOC(dai, ThermalConfig, ThermalImageParams, colorPalette))
        .def_readwrite(
            "gradientCorrection", &ThermalConfig::ThermalImageParams::gradientCorrection, DOC(dai, ThermalConfig, ThermalImageParams, gradientCorrection))
        .def_readwrite("pipelineMode", &ThermalConfig::ThermalImageParams::pipelineMode, DOC(dai, ThermalConfig, ThermalImageParams, pipelineMode));

    thermalHistEqParams.def(py::init<>())
        .def_readwrite("plateau", &ThermalConfig::ThermalHistEqParams::plateau, DOC(dai, ThermalConfig, ThermalHistEqParams, plateau))
        .def_readwrite("plateauRedistributionMode",
                       &ThermalConfig::ThermalHistEqParams::plateauRedistributionMode,
                       DOC(dai, ThermalConfig, ThermalHistEqParams, plateauRedistributionMode))
        .def_readwrite("gainLimit", &ThermalConfig::ThermalHistEqParams::gainLimit, DOC(dai, ThermalConfig, ThermalHistEqParams, gainLimit))
        .def_readwrite(
            "gainLimitFactorMode", &ThermalConfig::ThermalHistEqParams::gainLimitFactorMode, DOC(dai, ThermalConfig, ThermalHistEqParams, gainLimitFactorMode))
        .def_readwrite(
            "gainLimitFactorYMin", &ThermalConfig::ThermalHistEqParams::gainLimitFactorYMin, DOC(dai, ThermalConfig, ThermalHistEqParams, gainLimitFactorYMin))
        .def_readwrite(
            "gainLimitFactorXMax", &ThermalConfig::ThermalHistEqParams::gainLimitFactorXMax, DOC(dai, ThermalConfig, ThermalHistEqParams, gainLimitFactorXMax))
        .def_readwrite(
            "alphaTimeSeconds", &ThermalConfig::ThermalHistEqParams::alphaTimeSeconds, DOC(dai, ThermalConfig, ThermalHistEqParams, alphaTimeSeconds))
        .def_readwrite("trimLeft", &ThermalConfig::ThermalHistEqParams::trimLeft, DOC(dai, ThermalConfig, ThermalHistEqParams, trimLeft))
        .def_readwrite("trimRight", &ThermalConfig::ThermalHistEqParams::trimRight, DOC(dai, ThermalConfig, ThermalHistEqParams, trimRight))
        .def_readwrite("roiEnable", &ThermalConfig::ThermalHistEqParams::roiEnable, DOC(dai, ThermalConfig, ThermalHistEqParams, roiEnable))
        .def_readwrite("roi", &ThermalConfig::ThermalHistEqParams::roi, DOC(dai, ThermalConfig, ThermalHistEqParams, roi));

    thermalAgcParams.def(py::init<>())
        .def_readwrite("mode", &ThermalConfig::ThermalAgcParams::mode, DOC(dai, ThermalConfig, ThermalAgcParams, mode))
        .def_readwrite("linearLockMode", &ThermalConfig::ThermalAgcParams::linearLockMode, DOC(dai, ThermalConfig, ThermalAgcParams, linearLockMode))
        .def_readwrite("linearMin", &ThermalConfig::ThermalAgcParams::linearMin, DOC(dai, ThermalConfig, ThermalAgcParams, linearMin))
        .def_readwrite("linearMax", &ThermalConfig::ThermalAgcParams::linearMax, DOC(dai, ThermalConfig, ThermalAgcParams, linearMax))
        .def_readwrite("histEq", &ThermalConfig::ThermalAgcParams::histEq, DOC(dai, ThermalConfig, ThermalAgcParams, histEq));

    thermalFSCParams.def(py::init<>())
        .def_readwrite("enable", &ThermalConfig::ThermalFSCParams::enable, DOC(dai, ThermalConfig, ThermalFSCParams, enable))
        .def_readwrite("action", &ThermalConfig::ThermalFSCParams::action, DOC(dai, ThermalConfig, ThermalFSCParams, action));

    thermalGainMode.value("LOW", ThermalConfig::ThermalGainMode::LOW, DOC(dai, ThermalConfig, ThermalGainMode, LOW))
        .value("HIGH", ThermalConfig::ThermalGainMode::HIGH, DOC(dai, ThermalConfig, ThermalGainMode, HIGH));

    thermalImageOrientation.value("Normal", ThermalConfig::ThermalImageOrientation::Normal)
        .value("Mirror", ThermalConfig::ThermalImageOrientation::Mirror)
        .value("Flip", ThermalConfig::ThermalImageOrientation::Flip)
        .value("MirrorFlip", ThermalConfig::ThermalImageOrientation::MirrorFlip);

    thermalColorPalette.value("WHITE_HOT", ThermalConfig::ThermalColorPalette::WHITE_HOT)
        .value("BLACK_HOT", ThermalConfig::ThermalColorPalette::BLACK_HOT)
        .value("SPECTRA", ThermalConfig::ThermalColorPalette::SPECTRA)
        .value("PRISM", ThermalConfig::ThermalColorPalette::PRISM)
        .value("TYRIAN", ThermalConfig::ThermalColorPalette::TYRIAN)
        .value("IRON", ThermalConfig::ThermalColorPalette::IRON)
        .value("AMBER", ThermalConfig::ThermalColorPalette::AMBER)
        .value("HI", ThermalConfig::ThermalColorPalette::HI)
        .value("GREEN", ThermalConfig::ThermalColorPalette::GREEN);

    thermalPipelineMode.value("LITE", ThermalConfig::ThermalPipelineMode::LITE)
        .value("LEGACY", ThermalConfig::ThermalPipelineMode::LEGACY)
        .value("SEEKVISION", ThermalConfig::ThermalPipelineMode::SEEKVISION)
        .value("EAGLE", ThermalConfig::ThermalPipelineMode::EAGLE);

    thermalAgcMode.value("LINEAR", ThermalConfig::ThermalAgcMode::LINEAR)
        .value("HISTEQ", ThermalConfig::ThermalAgcMode::HISTEQ)
        .value("LEGACY_HISTEQ", ThermalConfig::ThermalAgcMode::LEGACY_HISTEQ);

    thermalLinearAgcLockMode.value("AUTO", ThermalConfig::ThermalLinearAgcLockMode::AUTO)
        .value("MANUAL", ThermalConfig::ThermalLinearAgcLockMode::MANUAL)
        .value("MANUAL_MIN", ThermalConfig::ThermalLinearAgcLockMode::MANUAL_MIN)
        .value("MANUAL_MAX", ThermalConfig::ThermalLinearAgcLockMode::MANUAL_MAX);

    thermalHistEqGainLimitFactorMode.value("AUTO", ThermalConfig::ThermalHistEqGainLimitFactorMode::AUTO)
        .value("MANUAL", ThermalConfig::ThermalHistEqGainLimitFactorMode::MANUAL);

    thermalHistEqPlateauRedistributionMode.value("DISABLED", ThermalConfig::ThermalHistEqPlateauRedistributionMode::DISABLED)
        .value("ALL_BINS", ThermalConfig::ThermalHistEqPlateauRedistributionMode::ALL_BINS)
        .value("ACTIVE_BINS_ONLY", ThermalConfig::ThermalHistEqPlateauRedistributionMode::ACTIVE_BINS_ONLY);

    thermalFSCAction.value("STORE", ThermalConfig::ThermalFSCAction::STORE).value("CLEAR", ThermalConfig::ThermalFSCAction::CLEAR);

    // ThermalConfig bindings
    thermalConfig.def(py::init<>())
        .def("__repr__", &ThermalConfig::str)
        .def_readwrite("ambientParams", &ThermalConfig::ambientParams, DOC(dai, ThermalConfig, ambientParams))
        .def_readwrite("ffcParams", &ThermalConfig::ffcParams, DOC(dai, ThermalConfig, ffcParams))
        .def_readwrite("imageParams", &ThermalConfig::imageParams, DOC(dai, ThermalConfig, imageParams))
        .def_readwrite("agcParams", &ThermalConfig::agcParams, DOC(dai, ThermalConfig, agcParams))
        .def_readwrite("fscParams", &ThermalConfig::fscParams, DOC(dai, ThermalConfig, fscParams));
}
