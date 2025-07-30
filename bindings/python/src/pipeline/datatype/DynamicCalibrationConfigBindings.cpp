#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

void bind_dynamic_calibration_config(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    // Bind enum CalibrationCommand
    py::enum_<DynamicCalibrationConfig::CalibrationCommand>(m, "CalibrationCommand")
        .value("START_CALIBRATION_QUALITY_CHECK", DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK)
        .value("START_RECALIBRATION", DynamicCalibrationConfig::CalibrationCommand::START_RECALIBRATION)
        .value("START_FORCE_CALIBRATION_QUALITY_CHECK", DynamicCalibrationConfig::CalibrationCommand::START_FORCE_CALIBRATION_QUALITY_CHECK)
        .value("START_FORCE_RECALIBRATION", DynamicCalibrationConfig::CalibrationCommand::START_FORCE_RECALIBRATION)
        .value("START_LOADING_IMAGES", DynamicCalibrationConfig::CalibrationCommand::START_LOADING_IMAGES)
        .value("STOP_LOADING_IMAGES", DynamicCalibrationConfig::CalibrationCommand::STOP_LOADING_IMAGES)
        .export_values();

    // Bind DynamicCalibrationConfig
    py::class_<DynamicCalibrationConfig, Buffer, std::shared_ptr<DynamicCalibrationConfig>>(m, "DynamicCalibrationConfig")
        .def(py::init<>())
        .def_readwrite("calibrationCommand", &DynamicCalibrationConfig::calibrationCommand);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
