#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

void bind_dynamic_calibration_config(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    // Bind enum Command
    py::enum_<DynamicCalibrationCommand::Command>(m, "Command")
        .value("START_CALIBRATION_QUALITY_CHECK", DynamicCalibrationCommand::Command::START_CALIBRATION_QUALITY_CHECK)
        .value("START_RECALIBRATION", DynamicCalibrationCommand::Command::START_RECALIBRATION)
        .value("START_FORCE_CALIBRATION_QUALITY_CHECK", DynamicCalibrationCommand::Command::START_FORCE_CALIBRATION_QUALITY_CHECK)
        .value("START_FORCE_RECALIBRATION", DynamicCalibrationCommand::Command::START_FORCE_RECALIBRATION)
        .value("START_LOADING_IMAGES", DynamicCalibrationCommand::Command::START_LOADING_IMAGES)
        .value("STOP_LOADING_IMAGES", DynamicCalibrationCommand::Command::STOP_LOADING_IMAGES)
        .value("APPLY_NEW_CALIBRATION", DynamicCalibrationCommand::Command::APPLY_NEW_CALIBRATION)
        .value("APPLY_PREVIOUS_CALIBRATION", DynamicCalibrationCommand::Command::APPLY_PREVIOUS_CALIBRATION)
        .value("APPLY_INITIAL_CALIBRATION", DynamicCalibrationCommand::Command::APPLY_INITIAL_CALIBRATION)
        .export_values();

    // Bind DynamicCalibrationCommand
    py::class_<DynamicCalibrationCommand, Buffer, std::shared_ptr<DynamicCalibrationCommand>>(m, "DynamicCalibrationCommand")
        .def(py::init<>())
        .def_readwrite("calibrationCommand", &DynamicCalibrationCommand::calibrationCommand);

    // Bind enum RecalibrationMode
    py::enum_<DynamicCalibrationConfig::RecalibrationMode>(m, "RecalibrationMode")
        .value("DEFAULT", DynamicCalibrationConfig::RecalibrationMode::DEFAULT)
        .value("CONTINUOUS", DynamicCalibrationConfig::RecalibrationMode::CONTINUOUS)
        .export_values();

    // Optionally bind dcl::PerformanceMode here if not already exposed elsewhere
    py::enum_<dcl::PerformanceMode>(m, "PerformanceMode")
        .value("DEFAULT", dcl::PerformanceMode::DEFAULT)
        .value("STATIC_SCENERY", dcl::PerformanceMode::STATIC_SCENERY)
        .value("OPTIMIZE_SPEED", dcl::PerformanceMode::OPTIMIZE_SPEED)
        .value("OPTIMIZE_PERFORMANCE", dcl::PerformanceMode::OPTIMIZE_PERFORMANCE)
        .value("SKIP_CHECKS", dcl::PerformanceMode::SKIP_CHECKS)
        .export_values();

    // Bind DynamicCalibrationConfig
    py::class_<DynamicCalibrationConfig, Buffer, std::shared_ptr<DynamicCalibrationConfig>>(m, "DynamicCalibrationConfig")
        .def(py::init<>())
        .def_readwrite("recalibrationMode", &DynamicCalibrationConfig::recalibrationMode)
        .def_readwrite("performanceMode", &DynamicCalibrationConfig::performanceMode)
        .def_readwrite("loadImageFrequency", &DynamicCalibrationConfig::loadImageFrequency)
        .def_readwrite("calibrationFrequency", &DynamicCalibrationConfig::calibrationFrequency);

    // Call remaining stack
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
