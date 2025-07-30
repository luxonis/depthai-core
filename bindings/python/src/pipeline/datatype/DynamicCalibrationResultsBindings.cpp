#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"

void bind_dynamic_calibration_results(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    // Bind CoverageData
    py::class_<CoverageData, Buffer, std::shared_ptr<CoverageData>>(m, "CoverageData")
        .def(py::init<>())
        .def_readwrite("coveragePerCellA", &CoverageData::coveragePerCellA)
        .def_readwrite("coveragePerCellB", &CoverageData::coveragePerCellB)
        .def_readwrite("meanCoverage", &CoverageData::meanCoverage)
        .def_readwrite("dataAcquired", &CoverageData::dataAcquired);

    // Bind CalibrationQuality::Data
    py::class_<CalibrationQuality::Data>(m, "CalibrationQualityData")
        .def(py::init<>())
        .def_readwrite("rotationChange", &CalibrationQuality::Data::rotationChange)
        .def_readwrite("epipolarErrorChange", &CalibrationQuality::Data::epipolarErrorChange)
        .def_readwrite("depthErrorDifference", &CalibrationQuality::Data::depthErrorDifference);

    // Bind CalibrationQuality
    py::class_<CalibrationQuality, Buffer, std::shared_ptr<CalibrationQuality>>(m, "CalibrationQuality")
        .def(py::init<>())
        .def_readwrite("data", &CalibrationQuality::data)
        .def_readwrite("info", &CalibrationQuality::info);

    // Bind DynamicCalibrationResult
    py::class_<DynamicCalibrationResult, Buffer, std::shared_ptr<DynamicCalibrationResult>>(m, "DynamicCalibrationResult")
        .def(py::init<>())
        .def(py::init<std::optional<dai::CalibrationHandler>, std::optional<CalibrationQuality::Data>, std::string>(),
             "calibration"_a,
             "calibrationQuality"_a,
             "info"_a)
        .def_readwrite("calibration", &DynamicCalibrationResult::calibration)
        .def_readwrite("calibrationQuality", &DynamicCalibrationResult::calibrationQuality)
        .def_readwrite("info", &DynamicCalibrationResult::info);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
