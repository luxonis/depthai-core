#include <pybind11/stl.h>

#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"

void bind_dynamic_calibration_results(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    using namespace pybind11::literals;
    namespace py = pybind11;

    // CoverageData
    py::class_<CoverageData, Py<CoverageData>, Buffer, py::smart_holder> coverageData(m, "CoverageData");
    py::class_<CoverageData::MarkerCoverage>(coverageData, "MarkerCoverage", DOC(dai, CoverageData, MarkerCoverage))
        .def(py::init<>())
        .def_readwrite("requiredMarkers", &CoverageData::MarkerCoverage::requiredMarkers, DOC(dai, CoverageData, MarkerCoverage, requiredMarkers))
        .def_readwrite("seenBy", &CoverageData::MarkerCoverage::seenBy, DOC(dai, CoverageData, MarkerCoverage, seenBy))
        .def_readwrite("missingMarkers", &CoverageData::MarkerCoverage::missingMarkers, DOC(dai, CoverageData, MarkerCoverage, missingMarkers))
        .def_readwrite("tooSmallPx", &CoverageData::MarkerCoverage::tooSmallPx, DOC(dai, CoverageData, MarkerCoverage, tooSmallPx))
        .def_readwrite("markersAcquired", &CoverageData::MarkerCoverage::markersAcquired, DOC(dai, CoverageData, MarkerCoverage, markersAcquired))
        .def_readwrite("accepted", &CoverageData::MarkerCoverage::accepted, DOC(dai, CoverageData, MarkerCoverage, accepted))
        .def_readwrite("observationSets", &CoverageData::MarkerCoverage::observationSets, DOC(dai, CoverageData, MarkerCoverage, observationSets));
    coverageData.def(py::init<>())
        .def_readwrite("coveragePerCellA", &CoverageData::coveragePerCellA)
        .def_readwrite("coveragePerCellB", &CoverageData::coveragePerCellB)
        .def_readwrite("coveragePerCell", &CoverageData::coveragePerCell)
        .def_readwrite("meanCoverage", &CoverageData::meanCoverage)
        .def_readwrite("dataAcquired", &CoverageData::dataAcquired)
        .def_readwrite("coverageAcquired", &CoverageData::coverageAcquired)
        .def_readwrite("markers", &CoverageData::markers, DOC(dai, CoverageData, markers));

    // CalibrationQuality::Data
    py::class_<CalibrationQuality::Data>(m, "CalibrationQualityData")
        .def(py::init<>())
        .def_readwrite("rotationChange", &CalibrationQuality::Data::rotationChange)              // std::array<float,3>
        .def_readwrite("depthErrorDifference", &CalibrationQuality::Data::depthErrorDifference)  // std::vector<float>
        .def_readwrite("sampsonErrorCurrent", &CalibrationQuality::Data::sampsonErrorCurrent)
        .def_readwrite("sampsonErrorNew", &CalibrationQuality::Data::sampsonErrorNew)
        .def_readwrite("pairwiseRotationDifference", &CalibrationQuality::Data::pairwiseRotationDifference);

    // CalibrationQuality
    py::class_<CalibrationQuality, Py<CalibrationQuality>, Buffer, py::smart_holder>(m, "CalibrationQuality")
        .def(py::init<>())
        .def(py::init<CalibrationQuality::Data, std::string>(), "qualityData"_a, "info"_a)
        .def(py::init<std::string>(), "info"_a)
        .def_readwrite("qualityData", &CalibrationQuality::qualityData)  // std::optional<CalibrationQuality::Data>
        .def_readwrite("info", &CalibrationQuality::info);

    py::class_<CalibrationMetrics, Py<CalibrationMetrics>, Buffer, py::smart_holder>(m, "CalibrationMetrics")
        .def(py::init<>())
        .def_readwrite("calibrationConfidence", &CalibrationMetrics::calibrationConfidence)
        .def_readwrite("dataConfidence", &CalibrationMetrics::dataConfidence);

    // DynamicCalibrationResult::Data
    py::class_<DynamicCalibrationResult::Data>(m, "DynamicCalibrationResultData")
        .def(py::init<>())
        .def_readwrite("newCalibration", &DynamicCalibrationResult::Data::newCalibration)                // dai::CalibrationHandler
        .def_readwrite("currentCalibration", &DynamicCalibrationResult::Data::currentCalibration)        // dai::CalibrationHandler
        .def_readwrite("calibrationDifference", &DynamicCalibrationResult::Data::calibrationDifference)  // CalibrationQuality::Data
        .def_readwrite("dataConfidence", &DynamicCalibrationResult::Data::dataConfidence);

    // DynamicCalibrationResult
    py::class_<DynamicCalibrationResult, Py<DynamicCalibrationResult>, Buffer, py::smart_holder>(m, "DynamicCalibrationResult")
        .def(py::init<>())
        .def(py::init<DynamicCalibrationResult::Data, std::string>(), "data"_a, "info"_a)
        .def(py::init<std::string>(), "info"_a)
        .def_readwrite("calibrationData", &DynamicCalibrationResult::calibrationData)  // std::optional<Data>
        .def_readwrite("info", &DynamicCalibrationResult::info);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}
