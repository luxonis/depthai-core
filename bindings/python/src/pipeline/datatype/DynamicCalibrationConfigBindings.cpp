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

    py::enum_<DynamicCalibrationConfig::CalibrationCommand> _CalibrationCommand(_DynamicCalibrationResults, "CalibrationCommand", DOC(dai, DynamicCalibrationConfig, CalibrationCommand));

    py::enum_<DynamicCalibrationConfig::AlgorithmControl::PerformanceMode>(_AlgorithmControl, "PerformanceMode")
        .value("SKIP_CHECKS", DynamicCalibrationConfig::AlgorithmControl::PerformanceMode::SKIP_CHECKS)
        .value("STATIC_SCENERY", DynamicCalibrationConfig::AlgorithmControl::PerformanceMode::STATIC_SCENERY)
        .value("OPTIMIZE_SPEED", DynamicCalibrationConfig::AlgorithmControl::PerformanceMode::OPTIMIZE_SPEED)
        .value("OPTIMIZE_PEFRORMACE", DynamicCalibrationConfig::AlgorithmControl::PerformanceMode::OPTIMIZE_PEFRORMACE)
        .value("DEFAULT", DynamicCalibrationConfig::AlgorithmControl::PerformanceMode::DEFAULT)
        .export_values();
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
        .def_readwrite("performanceMode",
                       &DynamicCalibrationConfig::AlgorithmControl::performanceMode,
                       DOC(dai, DynamicCalibrationConfig, AlgorithmControl, performanceMode ))
        .def_readwrite("timeFrequency",
                       &DynamicCalibrationConfig::AlgorithmControl::timeFrequency,
                       DOC(dai, DynamicCalibrationConfig, AlgorithmControl, timeFrequency))
    ;


    _CalibrationCommand
        .value("START_CALIBRATION_QUALITY_CHECK", DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK)
        .value("START_FORCE_CALIBRATION_QUALITY_CHECK", DynamicCalibrationConfig::CalibrationCommand::START_FORCE_CALIBRATION_QUALITY_CHECK)
        .value("START_RECALIBRATION", DynamicCalibrationConfig::CalibrationCommand::START_RECALIBRATION)
        .value("START_FORCE_RECALIBRATION", DynamicCalibrationConfig::CalibrationCommand::START_FORCE_RECALIBRATION)
    ;

    // Message
    _DynamicCalibrationResults.def(py::init<>())
        .def("__repr__", &DynamicCalibrationConfig::str)

        .def_readwrite("algorithmControl",
                       &DynamicCalibrationConfig::algorithmControl,
                       DOC(dai, DynamicCalibrationConfig, algorithmControl))
        .def_readwrite("calibrationCommand",
                       &DynamicCalibrationConfig::calibrationCommand,
                       DOC(dai, DynamicCalibrationConfig, calibrationCommand))
        ;
}
