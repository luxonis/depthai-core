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

    // ---- Commands nested inside DynamicCalibrationControl ----
    py::class_<DCC::CalibrateCommand, std::shared_ptr<DCC::CalibrateCommand>>(cls, "CalibrateCommand")
        .def(py::init<bool>(), py::arg("force") = false)
        .def_readwrite("force", &DCC::CalibrateCommand::force);

    py::class_<DCC::CalibrationQualityCommand, std::shared_ptr<DCC::CalibrationQualityCommand>>(cls, "CalibrationQualityCommand")
        .def(py::init<bool>(), py::arg("force") = false)
        .def_readwrite("force", &DCC::CalibrationQualityCommand::force);

    py::class_<DCC::StartCalibrationCommand, std::shared_ptr<DCC::StartCalibrationCommand>>(cls, "StartCalibrationCommand")
        .def(py::init<float, float>(), py::arg("loadImagePeriod") = 0.5f, py::arg("calibrationPeriod") = 5.0f)
        .def_readwrite("loadImagePeriod", &DCC::StartCalibrationCommand::loadImagePeriod)
        .def_readwrite("calibrationPeriod", &DCC::StartCalibrationCommand::calibrationPeriod);

    py::class_<DCC::StopCalibrationCommand, std::shared_ptr<DCC::StopCalibrationCommand>>(cls, "StopCalibrationCommand").def(py::init<>());

    py::class_<DCC::LoadImageCommand, std::shared_ptr<DCC::LoadImageCommand>>(cls, "LoadImageCommand").def(py::init<>());

    py::class_<DCC::ApplyCalibrationCommand, std::shared_ptr<DCC::ApplyCalibrationCommand>>(cls, "ApplyCalibrationCommand")
        .def(py::init<>())
        .def(py::init<const dai::CalibrationHandler&>(), py::arg("calibration"))
        .def_readwrite("calibration", &DCC::ApplyCalibrationCommand::calibration);

    py::class_<DCC::ResetDataCommand, std::shared_ptr<DCC::ResetDataCommand>>(cls, "ResetDataCommand").def(py::init<>());

    py::class_<DCC::SetPerformanceModeCommand, std::shared_ptr<DCC::SetPerformanceModeCommand>>(cls, "SetPerformanceModeCommand")
        .def(py::init<dai::node::DynamicCalibration::PerformanceMode>(), py::arg("performanceMode"))
        .def_readwrite("performanceMode", &DCC::SetPerformanceModeCommand::performanceMode);

    // ---- Constructors on the owner type (for variant dispatch) ----
    cls.def(py::init<>())
        .def(py::init<const DCC::CalibrateCommand&>())
        .def(py::init<const DCC::CalibrationQualityCommand&>())
        .def(py::init<const DCC::StartCalibrationCommand&>())
        .def(py::init<const DCC::StopCalibrationCommand&>())
        .def(py::init<const DCC::LoadImageCommand&>())
        .def(py::init<const DCC::ApplyCalibrationCommand&>())
        .def(py::init<const DCC::ResetDataCommand&>())
        .def(py::init<const DCC::SetPerformanceModeCommand&>())

        // Manual property exposing the variant
        .def_property(
            "command",
            [](const DCC& self) -> py::object { return std::visit([](const auto& alt) { return py::cast(alt); }, self.command); },
            [](DCC& self, py::object obj) {
                if(py::isinstance<DCC::CalibrateCommand>(obj))
                    self.command = obj.cast<DCC::CalibrateCommand>();
                else if(py::isinstance<DCC::CalibrationQualityCommand>(obj))
                    self.command = obj.cast<DCC::CalibrationQualityCommand>();
                else if(py::isinstance<DCC::StartCalibrationCommand>(obj))
                    self.command = obj.cast<DCC::StartCalibrationCommand>();
                else if(py::isinstance<DCC::StopCalibrationCommand>(obj))
                    self.command = obj.cast<DCC::StopCalibrationCommand>();
                else if(py::isinstance<DCC::LoadImageCommand>(obj))
                    self.command = obj.cast<DCC::LoadImageCommand>();
                else if(py::isinstance<DCC::ApplyCalibrationCommand>(obj))
                    self.command = obj.cast<DCC::ApplyCalibrationCommand>();
                else if(py::isinstance<DCC::ResetDataCommand>(obj))
                    self.command = obj.cast<DCC::ResetDataCommand>();
                else if(py::isinstance<DCC::SetPerformanceModeCommand>(obj))
                    self.command = obj.cast<DCC::SetPerformanceModeCommand>();
                else
                    throw py::type_error("Unsupported command type for DynamicCalibrationControl.command");
            });

    // Call remaining stack
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
