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

    // dcl::PerformanceMode (bind here unless it’s already exposed elsewhere)
    // If it's already bound in another TU, remove this block to avoid duplicate definitions.
    py::enum_<dcl::PerformanceMode>(m, "PerformanceMode")
        .value("DEFAULT", dcl::PerformanceMode::DEFAULT)
        .value("STATIC_SCENERY", dcl::PerformanceMode::STATIC_SCENERY)
        .value("OPTIMIZE_SPEED", dcl::PerformanceMode::OPTIMIZE_SPEED)
        .value("OPTIMIZE_PERFORMANCE", dcl::PerformanceMode::OPTIMIZE_PERFORMANCE)
        .value("SKIP_CHECKS", dcl::PerformanceMode::SKIP_CHECKS)
        .export_values();

    // ----- Commands base -----

    py::class_<DynamicCalibrationCommand, Buffer, std::shared_ptr<DynamicCalibrationCommand>>(m, "DynamicCalibrationCommand").def(py::init<>());

    // ----- Concrete commands -----

    py::class_<CalibrateCommand, DynamicCalibrationCommand, std::shared_ptr<CalibrateCommand>>(m, "CalibrateCommand")
        .def(py::init<bool>(), "force"_a = false)
        .def_readwrite("force", &CalibrateCommand::force);

    py::class_<CalibrationQualityCommand, DynamicCalibrationCommand, std::shared_ptr<CalibrationQualityCommand>>(m, "CalibrationQualityCommand")
        .def(py::init<bool>(), "force"_a = false)
        .def_readwrite("force", &CalibrationQualityCommand::force);

    py::class_<StartCalibrationCommand, DynamicCalibrationCommand, std::shared_ptr<StartCalibrationCommand>>(m, "StartCalibrationCommand")
        .def(py::init<float, float>(), "loadImagePeriod"_a = 0.5f, "calibrationPeriod"_a = 5.0f)
        .def_readwrite("loadImagePeriod", &StartCalibrationCommand::loadImagePeriod)
        .def_readwrite("calibrationPeriod", &StartCalibrationCommand::calibrationPeriod);

    py::class_<StopCalibrationCommand, DynamicCalibrationCommand, std::shared_ptr<StopCalibrationCommand>>(m, "StopCalibrationCommand").def(py::init<>());

    py::class_<LoadImageCommand, DynamicCalibrationCommand, std::shared_ptr<LoadImageCommand>>(m, "LoadImageCommand").def(py::init<>());

    py::class_<ApplyCalibrationCommand, DynamicCalibrationCommand, std::shared_ptr<ApplyCalibrationCommand>>(m, "ApplyCalibrationCommand")
        .def(py::init<>())
        .def(py::init<const dai::CalibrationHandler&>(), py::arg("calibration"))
        .def_readwrite("calibration", &ApplyCalibrationCommand::calibration);

    py::class_<ResetDataCommand, DynamicCalibrationCommand, std::shared_ptr<ResetDataCommand>>(m, "ResetDataCommand").def(py::init<>());

    py::class_<SetPerformanceModeCommand, DynamicCalibrationCommand, std::shared_ptr<SetPerformanceModeCommand>>(m, "SetPerformanceModeCommand")
        .def(py::init<dcl::PerformanceMode>(), "performanceMode"_a = dcl::PerformanceMode::DEFAULT);

    // ----- Call remaining stack -----
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
