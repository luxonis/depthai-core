#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

void bind_dynamic_calibration_config(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<DynamicCalibrationConfig, Py<DynamicCalibrationConfig>, Buffer, std::shared_ptr<DynamicCalibrationConfig>> _DynamicCalibrationResults(
        m, "DynamicCalibrationConfig", DOC(dai, DynamicCalibrationConfig));

    py::class_<DynamicCalibrationConfig::AlgorithmControl> _AlgorithmControl(
        _DynamicCalibrationResults, "AlgorithmControl", DOC(dai, DynamicCalibrationConfig, AlgorithmControl));

    py::class_<DynamicCalibrationConfig::CoverageCheckThresholds> _CoverageCheckThresholds(
        _DynamicCalibrationResults, "CoverageCheckThresholds", DOC(dai, DynamicCalibrationConfig, CoverageCheckThresholds));

    py::class_<DynamicCalibrationConfig::CalibCheckThresholds> _CalibCheckThresholds(
        _DynamicCalibrationResults, "CalibCheckThresholds", DOC(dai, DynamicCalibrationConfig, CalibCheckThresholds));

    py::class_<DynamicCalibrationConfig::RecalibrationThresholds> _RecalibrationThresholds(
        _DynamicCalibrationResults, "RecalibrationThresholds", DOC(dai, DynamicCalibrationConfig, RecalibrationThresholds));

    py::enum_<DynamicCalibrationConfig::CalibrationCommand> _CalibrationCommand(_DynamicCalibrationResults, "CalibrationCommand", DOC(dai, DynamicCalibrationConfig, CalibrationCommand));


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

    // Metadata / raw
    _AlgorithmControl.def(py::init<>())
        .def_readwrite("recalibrationMode",
                       &DynamicCalibrationConfig::AlgorithmControl::recalibrationMode,
                       DOC(dai, DynamicCalibrationConfig, AlgorithmControl, recalibrationMode))
        .def_readwrite("maximumCalibCheckFrames",
                       &DynamicCalibrationConfig::AlgorithmControl::maximumCalibCheckFrames,
                       DOC(dai, DynamicCalibrationConfig, AlgorithmControl, maximumCalibCheckFrames))
        .def_readwrite("maximumRecalibrationFrames",
                       &DynamicCalibrationConfig::AlgorithmControl::maximumRecalibrationFrames,
                       DOC(dai, DynamicCalibrationConfig, AlgorithmControl, maximumRecalibrationFrames))
        .def_readwrite("enableCoverageCheck",
                       &DynamicCalibrationConfig::AlgorithmControl::enableCoverageCheck,
                       DOC(dai, DynamicCalibrationConfig, AlgorithmControl, enableCoverageCheck))
    ;

    _CoverageCheckThresholds.def(py::init<>())
        .def_readwrite("coverageCheckThreshold",
                       &DynamicCalibrationConfig::CoverageCheckThresholds::coverageCheckThreshold,
                       DOC(dai, DynamicCalibrationConfig, CoverageCheckThresholds, coverageCheckThreshold))
    ;


    _CalibCheckThresholds.def(py::init<>())
        .def_readwrite("epipolarErrorChangeThresholds",
                       &DynamicCalibrationConfig::CalibCheckThresholds::epipolarErrorChangeThresholds,
                       DOC(dai, DynamicCalibrationConfig, CalibCheckThresholds, epipolarErrorChangeThresholds))
        .def_readwrite("rotationChangeThresholds",
                       &DynamicCalibrationConfig::CalibCheckThresholds::rotationChangeThresholds,
                       DOC(dai, DynamicCalibrationConfig, CalibCheckThresholds, rotationChangeThresholds))
    ;

    _RecalibrationThresholds.def(py::init<>())
        .def_readwrite("flashNewCalibration",
                       &DynamicCalibrationConfig::RecalibrationThresholds::flashNewCalibration,
                       DOC(dai, DynamicCalibrationConfig, RecalibrationThresholds, flashNewCalibration))
    ;

    _CalibrationCommand
        .value("START_CALIBRATION_QUALITY_CHECK", DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK)
        .value("START_RECALIBRATION", DynamicCalibrationConfig::CalibrationCommand::START_RECALIBRATION)
    ;

    // Message
    _DynamicCalibrationResults.def(py::init<>())
        .def("__repr__", &DynamicCalibrationConfig::str)

        .def_readwrite("algorithmControl",
                       &DynamicCalibrationConfig::algorithmControl,
                       DOC(dai, DynamicCalibrationConfig, algorithmControl))
        .def_readwrite("coverageCheckThresholds",
                       &DynamicCalibrationConfig::coverageCheckThresholds,
                       DOC(dai, DynamicCalibrationConfig, coverageCheckThresholds))
        .def_readwrite("calibCheckThresholds",
                       &DynamicCalibrationConfig::calibCheckThresholds,
                       DOC(dai, DynamicCalibrationConfig, calibCheckThresholds))
        .def_readwrite("recalibrationThresholds",
                       &DynamicCalibrationConfig::recalibrationThresholds,
                       DOC(dai, DynamicCalibrationConfig, recalibrationThresholds))
        .def_readwrite("calibrationCommand",
                       &DynamicCalibrationConfig::calibrationCommand,
                       DOC(dai, DynamicCalibrationConfig, calibrationCommand))
        ;
}
