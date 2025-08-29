#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

namespace py = pybind11;

void bind_dynamic_calibration_config(py::module& m, void* pCallstack) {
    using namespace dai;
    using DCC = dai::DynamicCalibrationControl;

    // Owning message FIRST
    auto cls = py::class_<DCC, Buffer, std::shared_ptr<DCC>>(m, "DynamicCalibrationControl");
    auto cmds = py::class_<DCC::Commands>(cls, "Commands");

    // Aliases for readability
    using Calibrate = DCC::Commands::Calibrate;
    using CalibrationQuality = DCC::Commands::CalibrationQuality;
    using StartCalibration = DCC::Commands::StartCalibration;
    using StopCalibration = DCC::Commands::StopCalibration;
    using LoadImage = DCC::Commands::LoadImage;
    using ApplyCalibration = DCC::Commands::ApplyCalibration;
    using ResetData = DCC::Commands::ResetData;
    using SetPerformanceMode = DCC::Commands::SetPerformanceMode;

    // ---- Commands nested inside DynamicCalibrationControl ----
    py::class_<Calibrate, std::shared_ptr<Calibrate>>(cmds, "Calibrate")
        .def(py::init<bool>(), py::arg("force") = false)
        .def_readwrite("force", &Calibrate::force);

    py::class_<CalibrationQuality, std::shared_ptr<CalibrationQuality>>(cmds, "CalibrationQuality")
        .def(py::init<bool>(), py::arg("force") = false)
        .def_readwrite("force", &CalibrationQuality::force);

    py::class_<StartCalibration, std::shared_ptr<StartCalibration>>(cmds, "StartCalibration")
        .def(py::init<float, float>(), py::arg("loadImagePeriod") = 0.5f, py::arg("calibrationPeriod") = 5.0f)
        .def_readwrite("loadImagePeriod", &StartCalibration::loadImagePeriod)
        .def_readwrite("calibrationPeriod", &StartCalibration::calibrationPeriod);

    py::class_<StopCalibration, std::shared_ptr<StopCalibration>>(cmds, "StopCalibration").def(py::init<>());

    py::class_<LoadImage, std::shared_ptr<LoadImage>>(cmds, "LoadImage").def(py::init<>());

    py::class_<ApplyCalibration, std::shared_ptr<ApplyCalibration>>(cmds, "ApplyCalibration")
        .def(py::init<>())
        .def(py::init<const dai::CalibrationHandler&>(), py::arg("calibration"))
        .def_readwrite("calibration", &ApplyCalibration::calibration);

    py::class_<ResetData, std::shared_ptr<ResetData>>(cmds, "ResetData").def(py::init<>());

    py::class_<SetPerformanceMode, std::shared_ptr<SetPerformanceMode>>(cmds, "SetPerformanceMode")
        .def(py::init<dai::node::DynamicCalibration::PerformanceMode>(), py::arg("performanceMode"))
        .def_readwrite("performanceMode", &SetPerformanceMode::performanceMode);

    // ---- Constructors on the owner type (for variant dispatch) ----
    cls.def(py::init<>())
        .def(py::init<const Calibrate&>())
        .def(py::init<const CalibrationQuality&>())
        .def(py::init<const StartCalibration&>())
        .def(py::init<const StopCalibration&>())
        .def(py::init<const LoadImage&>())
        .def(py::init<const ApplyCalibration&>())
        .def(py::init<const ResetData&>())
        .def(py::init<const SetPerformanceMode&>())

        // Manual property exposing the variant
        .def_property(
            "command",
            [](const DCC& self) -> py::object { return std::visit([](const auto& alt) { return py::cast(alt); }, self.command); },
            [](DCC& self, py::object obj) {
                if(py::isinstance<Calibrate>(obj))
                    self.command = obj.cast<Calibrate>();
                else if(py::isinstance<CalibrationQuality>(obj))
                    self.command = obj.cast<CalibrationQuality>();
                else if(py::isinstance<StartCalibration>(obj))
                    self.command = obj.cast<StartCalibration>();
                else if(py::isinstance<StopCalibration>(obj))
                    self.command = obj.cast<StopCalibration>();
                else if(py::isinstance<LoadImage>(obj))
                    self.command = obj.cast<LoadImage>();
                else if(py::isinstance<ApplyCalibration>(obj))
                    self.command = obj.cast<ApplyCalibration>();
                else if(py::isinstance<ResetData>(obj))
                    self.command = obj.cast<ResetData>();
                else if(py::isinstance<SetPerformanceMode>(obj))
                    self.command = obj.cast<SetPerformanceMode>();
                else
                    throw py::type_error("Unsupported command type for DynamicCalibrationControl.Command");
            });

    // Call remaining stack
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
