#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"
#include "depthai/properties/DynamicCalibrationProperties.hpp"

void bind_dynamic_calibration_config(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<DynamicCalibrationConfig, Py<DynamicCalibrationConfig>, Buffer, std::shared_ptr<DynamicCalibrationConfig>> _DynamicCalibrationResults(
        m, "DynamicCalibrationConfig", DOC(dai, DynamicCalibrationConfig));

    py::class_<DynamicCalibrationProperties> _AlgorithmControl(
        _DynamicCalibrationResults, "AlgorithmControl", DOC(dai, DynamicCalibrationConfig, AlgorithmControl));

    py::enum_<DynamicCalibrationProperties::CalibrationCommand> _CalibrationCommand(
        _DynamicCalibrationResults, "CalibrationCommand", DOC(dai, DynamicCalibrationConfig, CalibrationCommand));

    py::enum_<DynamicCalibrationProperties::PerformanceMode>(_AlgorithmControl, "PerformanceMode")
        .value("SKIP_CHECKS", DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS)
        .value("STATIC_SCENERY", DynamicCalibrationProperties::PerformanceMode::STATIC_SCENERY)
        .value("OPTIMIZE_SPEED", DynamicCalibrationProperties::PerformanceMode::OPTIMIZE_SPEED)
        .value("OPTIMIZE_PERFORMANCE", DynamicCalibrationProperties::PerformanceMode::OPTIMIZE_PERFORMANCE)
        .value("DEFAULT", DynamicCalibrationProperties::PerformanceMode::DEFAULT)
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
    // _AlgorithmControl.def(py::init<>())
    //     .def_readwrite("recalibrationMode", &DynamicCalibrationProperties::recalibrationMode, DOC(dai, DynamicCalibrationProperties, recalibrationMode))
    //     .def_readwrite("performanceMode", &DynamicCalibrationProperties::performanceMode, DOC(dai, DynamicCalibrationProperties, performanceMode))
    //     .def_readwrite("timeFrequency", &DynamicCalibrationProperties::timeFrequency, DOC(dai, DynamicCalibrationProperties, timeFrequency));

    // _CalibrationCommand.value("START_CALIBRATION_QUALITY_CHECK", DynamicCalibrationProperties::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK)
    //     .value("START_FORCE_CALIBRATION_QUALITY_CHECK", DynamicCalibrationProperties::CalibrationCommand::START_FORCE_CALIBRATION_QUALITY_CHECK)
    //     .value("START_RECALIBRATION", DynamicCalibrationProperties::CalibrationCommand::START_RECALIBRATION)
    //     .value("START_FORCE_RECALIBRATION", DynamicCalibrationProperties::CalibrationCommand::START_FORCE_RECALIBRATION);

    // // Message
    // _DynamicCalibrationResults.def(py::init<>())
    //     .def("__repr__", &DynamicCalibrationConfig::str)

    //     // .def_readwrite("algorithmControl", &DynamicCalibrationProperties::algorithmControl, DOC(dai, DynamicCalibrationConfig, algorithmControl))
    //     .def_readwrite("calibrationCommand", &DynamicCalibrationProperties::calibrationCommand, DOC(dai, DynamicCalibrationProperties, calibrationCommand));
}
