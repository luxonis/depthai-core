#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

namespace py = pybind11;

void bind_dynamic_calibration_control(py::module& m, void* pCallstack) {
    using namespace dai;
    using DCC = dai::DynamicCalibrationControl;

    // Owning message FIRST
    auto cls = py::class_<DCC, Buffer, std::shared_ptr<DCC>>(m, "DynamicCalibrationControl");
    auto cmds = py::class_<DCC::Commands>(cls, "Commands");

    py::enum_<DCC::PerformanceMode>(cls, "PerformanceMode")
        .value("DEFAULT", DCC::PerformanceMode::DEFAULT)
        .value("STATIC_SCENERY", DCC::PerformanceMode::STATIC_SCENERY)
        .value("OPTIMIZE_SPEED", DCC::PerformanceMode::OPTIMIZE_SPEED)
        .value("OPTIMIZE_PERFORMANCE", DCC::PerformanceMode::OPTIMIZE_PERFORMANCE)
        .value("SKIP_CHECKS", DCC::PerformanceMode::SKIP_CHECKS)
        .export_values();

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
        .def(py::init<dai::DynamicCalibrationControl::PerformanceMode>(), py::arg("performanceMode"))
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
        .def(py::init<const SetPerformanceMode&>());
    // Pythonic names + docstrings; defaults match C++.
    // clang-format off
    cls.def_static(
           "calibrate",
           &DCC::calibrate,
           py::arg("force") = false,
           "Create a DynamicCalibrationControl with a Calibrate command.");
    cls.def_static(
           "calibrationQuality",
           &DCC::calibrationQuality,
           py::arg("force") = false,
           "Create a DynamicCalibrationControl with a CalibrationQuality command.");
    cls.def_static(
           "startCalibration",
           &DCC::startCalibration,
           py::arg("loadImagePeriod") = 0.5f,
           py::arg("calibrationPeriod") = 5.0f,
           "Create a DynamicCalibrationControl with a StartCalibration command.");
    cls.def_static(
           "stopCalibration",
           &DCC::stopCalibration,
           "Create a DynamicCalibrationControl with a StopCalibration command.");
    cls.def_static(
           "loadImage",
           &DCC::loadImage,
           "Create a DynamicCalibrationControl with a LoadImage command.");
    cls.def_static(
           "applyCalibration",
           &DCC::applyCalibration,
           py::arg("calibration"),
           "Create a DynamicCalibrationControl with an ApplyCalibration command.");
    cls.def_static(
           "resetData",
           &DCC::resetData,
           "Create a DynamicCalibrationControl with a ResetData command.");
    cls.def_static(
           "setPerformanceMode",
           &DCC::setPerformanceMode,
           py::arg("mode") = dai::DynamicCalibrationControl::PerformanceMode::DEFAULT,
           "Create a DynamicCalibrationControl with a SetPerformanceMode command.");
    // clang-format on
    // Call remaining stack
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
