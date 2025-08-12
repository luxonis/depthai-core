#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

namespace py = pybind11;

void bind_dynamic_calibration_config(py::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    // ----- Enums -----

    // RecalibrationMode (in dai::DynamicCalibrationConfig)
    py::enum_<DynamicCalibrationConfig::RecalibrationMode>(m, "RecalibrationMode")
        .value("DEFAULT", DynamicCalibrationConfig::RecalibrationMode::DEFAULT)
        .value("CONTINUOUS", DynamicCalibrationConfig::RecalibrationMode::CONTINUOUS)
        .export_values();

    // dcl::PerformanceMode (bind here unless it’s already exposed elsewhere)
    // If it's already bound in another TU, remove this block to avoid duplicate definitions.
    py::enum_<dcl::PerformanceMode>(m, "PerformanceMode")
        .value("DEFAULT", dcl::PerformanceMode::DEFAULT)
        .value("STATIC_SCENERY", dcl::PerformanceMode::STATIC_SCENERY)
        .value("OPTIMIZE_SPEED", dcl::PerformanceMode::OPTIMIZE_SPEED)
        .value("OPTIMIZE_PERFORMANCE", dcl::PerformanceMode::OPTIMIZE_PERFORMANCE)
        .value("SKIP_CHECKS", dcl::PerformanceMode::SKIP_CHECKS)
        .export_values();

    // ----- Config -----

    py::class_<DynamicCalibrationConfig, Buffer, std::shared_ptr<DynamicCalibrationConfig>>(m, "DynamicCalibrationConfig")
        .def(py::init<>())
        .def_readwrite("recalibrationMode", &DynamicCalibrationConfig::recalibrationMode)
        .def_readwrite("performanceMode", &DynamicCalibrationConfig::performanceMode)
        .def_readwrite("loadImagePeriod", &DynamicCalibrationConfig::loadImagePeriod)
        .def_readwrite("calibrationPeriod", &DynamicCalibrationConfig::calibrationPeriod);

    // ----- Commands base -----

    py::class_<DynamicCalibrationCommand, Buffer, std::shared_ptr<DynamicCalibrationCommand>>(m, "DynamicCalibrationCommand").def(py::init<>());

    // ----- Concrete commands -----

    py::class_<RecalibrateCommand, DynamicCalibrationCommand, std::shared_ptr<RecalibrateCommand>>(m, "RecalibrateCommand")
        .def(py::init<bool, dcl::PerformanceMode>(), "force"_a = false, "performanceMode"_a = dcl::PerformanceMode::DEFAULT)
        .def_readwrite("performanceMode", &RecalibrateCommand::performanceMode)
        .def_readwrite("force", &RecalibrateCommand::force);

    py::class_<CalibrationQualityCommand, DynamicCalibrationCommand, std::shared_ptr<CalibrationQualityCommand>>(m, "CalibrationQualityCommand")
        .def(py::init<bool, dcl::PerformanceMode>(), "force"_a = false, "performanceMode"_a = dcl::PerformanceMode::DEFAULT)
        .def_readwrite("performanceMode", &CalibrationQualityCommand::performanceMode)
        .def_readwrite("force", &CalibrationQualityCommand::force);

    py::class_<StartRecalibrationCommand, DynamicCalibrationCommand, std::shared_ptr<StartRecalibrationCommand>>(m, "StartRecalibrationCommand")
        .def(py::init<dcl::PerformanceMode>(), "performanceMode"_a = dcl::PerformanceMode::DEFAULT)
        .def_readwrite("performanceMode", &StartRecalibrationCommand::performanceMode);

    py::class_<LoadImageCommand, DynamicCalibrationCommand, std::shared_ptr<LoadImageCommand>>(m, "LoadImageCommand").def(py::init<>());

    py::class_<ApplyCalibrationCommand, DynamicCalibrationCommand, std::shared_ptr<ApplyCalibrationCommand>>(m, "ApplyCalibrationCommand")
        .def(py::init<>())
        .def(py::init<const dai::CalibrationHandler&>(), py::arg("calibration"))
        .def_readwrite("calibration", &ApplyCalibrationCommand::calibration);

    // ----- Call remaining stack -----
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}