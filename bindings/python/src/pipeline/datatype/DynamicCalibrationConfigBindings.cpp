#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

namespace py = pybind11;

void bind_dynamic_calibration_config(py::module& m, void* pCallstack) {
    using namespace dai;
    using DCC = dai::DynamicCalibrationControl;
    using CalibrateCommand = DCC::CalibrateCommand;
    using CalibrationQualityCommand = DCC::CalibrationQualityCommand;
    using StartCalibrationCommand = DCC::StartCalibrationCommand;
    using StopCalibrationCommand = DCC::StopCalibrationCommand;
    using LoadImageCommand = DCC::LoadImageCommand;
    using ApplyCalibrationCommand = DCC::ApplyCalibrationCommand;
    using ResetDataCommand = DCC::ResetDataCommand;
    using SetPerformanceModeCommand = DCC::SetPerformanceModeCommand;

    // Enum (remove if already bound elsewhere)
    py::enum_<dcl::PerformanceMode>(m, "PerformanceMode")
        .value("DEFAULT", dcl::PerformanceMode::DEFAULT)
        .value("STATIC_SCENERY", dcl::PerformanceMode::STATIC_SCENERY)
        .value("OPTIMIZE_SPEED", dcl::PerformanceMode::OPTIMIZE_SPEED)
        .value("OPTIMIZE_PERFORMANCE", dcl::PerformanceMode::OPTIMIZE_PERFORMANCE)
        .value("SKIP_CHECKS", dcl::PerformanceMode::SKIP_CHECKS)
        .export_values();

    // Owning message FIRST (so we can nest under it)
    auto cls = py::class_<DCC, Buffer, std::shared_ptr<DCC>>(m, "DynamicCalibrationControl");

    py::class_<CalibrateCommand, std::shared_ptr<CalibrateCommand>>(cls, "CalibrateCommand")
        .def(py::init<bool>(), py::arg("force") = false)
        .def_readwrite("force", &CalibrateCommand::force);

    py::class_<CalibrationQualityCommand, std::shared_ptr<CalibrationQualityCommand>>(cls, "CalibrationQualityCommand")
        .def(py::init<bool>(), py::arg("force") = false)
        .def_readwrite("force", &CalibrationQualityCommand::force);

    py::class_<StartCalibrationCommand, std::shared_ptr<StartCalibrationCommand>>(cls, "StartCalibrationCommand")
        .def(py::init<float, float>(), py::arg("loadImagePeriod") = 0.5f, py::arg("calibrationPeriod") = 5.0f)
        .def_readwrite("loadImagePeriod", &StartCalibrationCommand::loadImagePeriod)
        .def_readwrite("calibrationPeriod", &StartCalibrationCommand::calibrationPeriod);

    py::class_<StopCalibrationCommand, std::shared_ptr<StopCalibrationCommand>>(cls, "StopCalibrationCommand")
        .def(py::init<>())
        .def_readwrite("nothing", &StopCalibrationCommand::nothing);

    py::class_<LoadImageCommand, std::shared_ptr<LoadImageCommand>>(cls, "LoadImageCommand").def(py::init<>());

    py::class_<ApplyCalibrationCommand, std::shared_ptr<ApplyCalibrationCommand>>(cls, "ApplyCalibrationCommand")
        .def(py::init<>())
        .def(py::init<const dai::CalibrationHandler&>(), py::arg("calibration"))
        .def_readwrite("calibration", &ApplyCalibrationCommand::calibration);

    py::class_<ResetDataCommand, std::shared_ptr<ResetDataCommand>>(cls, "ResetDataCommand").def(py::init<>());

    py::class_<SetPerformanceModeCommand, std::shared_ptr<SetPerformanceModeCommand>>(cls, "SetPerformanceModeCommand")
        .def(py::init<dcl::PerformanceMode>(), py::arg("performanceMode") = dcl::PerformanceMode::DEFAULT)
        .def_readwrite("performanceMode", &SetPerformanceModeCommand::performanceMode);

    cls
        // Default ctor (your C++ should initialize variant to a known alt)
        .def(py::init<>())

        // Per-alternative constructors (keep these; they’re robust)
        .def(py::init<const CalibrateCommand&>(), py::arg("cmd"))
        .def(py::init<const CalibrationQualityCommand&>(), py::arg("cmd"))
        .def(py::init<const StartCalibrationCommand&>(), py::arg("cmd"))
        .def(py::init<const StopCalibrationCommand&>(), py::arg("cmd"))
        .def(py::init<const LoadImageCommand&>(), py::arg("cmd"))
        .def(py::init<const ApplyCalibrationCommand&>(), py::arg("cmd"))
        .def(py::init<const ResetDataCommand&>(), py::arg("cmd"))
        .def(py::init<const SetPerformanceModeCommand&>(), py::arg("cmd"))

        // Manual property for the variant (avoids relying on variant caster)
        .def_property(
            "command",
            [](const DCC& self) -> py::object { return std::visit([](const auto& alt) { return py::cast(alt); }, self.command); },
            [](DCC& self, py::object obj) {
                if(py::isinstance<CalibrateCommand>(obj))
                    self.command = obj.cast<CalibrateCommand>();
                else if(py::isinstance<CalibrationQualityCommand>(obj))
                    self.command = obj.cast<CalibrationQualityCommand>();
                else if(py::isinstance<StartCalibrationCommand>(obj))
                    self.command = obj.cast<StartCalibrationCommand>();
                else if(py::isinstance<StopCalibrationCommand>(obj))
                    self.command = obj.cast<StopCalibrationCommand>();
                else if(py::isinstance<LoadImageCommand>(obj))
                    self.command = obj.cast<LoadImageCommand>();
                else if(py::isinstance<ApplyCalibrationCommand>(obj))
                    self.command = obj.cast<ApplyCalibrationCommand>();
                else if(py::isinstance<ResetDataCommand>(obj))
                    self.command = obj.cast<ResetDataCommand>();
                else if(py::isinstance<SetPerformanceModeCommand>(obj))
                    self.command = obj.cast<SetPerformanceModeCommand>();
                else
                    throw py::type_error(
                        "DynamicCalibrationControl.command must be one of: "
                        "CalibrateCommand, CalibrationQualityCommand, StartCalibrationCommand, StopCalibrationCommand, "
                        "LoadImageCommand, ApplyCalibrationCommand, ResetDataCommand, SetPerformanceModeCommand");
            });

    // Continue your callstack chain
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
