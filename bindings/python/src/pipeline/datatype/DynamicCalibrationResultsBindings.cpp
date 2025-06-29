#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"

void bind_dynamic_calibration_results(pybind11::module& m, void* pCallstack) {
    using namespace dai;


    py::class_<DynamicCalibrationResults, Py<DynamicCalibrationResults>, Buffer, std::shared_ptr<DynamicCalibrationResults>> _DynamicCalibrationResults(
        m, "DynamicCalibrationResults", DOC(dai, DynamicCalibrationResults));

    py::class_<DynamicCalibrationResults::CalibrationResult> _CalibrationResult(
        _DynamicCalibrationResults, "CalibrationResult", DOC(dai, DynamicCalibrationResults, CalibrationResult));

    py::class_<DynamicCalibrationResults::CalibrationData> _CalibrationData(
        _DynamicCalibrationResults, "CalibrationData", DOC(dai, DynamicCalibrationResults, CalibrationData));

    py::class_<DynamicCalibrationResults::CoverageData> _CoverageData(
        _DynamicCalibrationResults, "CoverageData", DOC(dai, DynamicCalibrationResults, CoverageData));

    py::class_<DynamicCalibrationResults::CalibrationQuality> _CalibrationQuality(
        _DynamicCalibrationResults, "CalibrationQuality", DOC(dai, DynamicCalibrationResults, CalibrationQuality));

    py::class_<DynamicCalibrationResults::CalibrationQualityResult> _CalibrationQualityResult(
        _DynamicCalibrationResults, "CalibrationQualityResult", DOC(dai, DynamicCalibrationResults, CalibrationQualityResult));


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
    _CalibrationResult
        .def(py::init<>())
        .def_readwrite("calibHandler", &DynamicCalibrationResults::CalibrationResult::calibHandler,
                       DOC(dai, DynamicCalibrationResults, CalibrationResult, calibHandler));

    // Bind CalibrationData
    _CalibrationData
        .def(py::init<>())
        .def_readwrite("rotationChange", &DynamicCalibrationResults::CalibrationData::rotationChange)
        .def_readwrite("epipolarErrorChange", &DynamicCalibrationResults::CalibrationData::epipolarErrorChange)
        .def_readwrite("depthErrorDifference", &DynamicCalibrationResults::CalibrationData::depthErrorDifference);

    // Bind CoverageData
    _CoverageData
        .def(py::init<>())
        .def_readwrite("coveragePerCellA", &DynamicCalibrationResults::CoverageData::coveragePerCellA)
        .def_readwrite("coveragePerCellB", &DynamicCalibrationResults::CoverageData::coveragePerCellB)
        .def_readwrite("meanCoverage", &DynamicCalibrationResults::CoverageData::meanCoverage);

    // Bind CalibrationQuality
    _CalibrationQuality
        .def(py::init<>())
        .def_readwrite("dataAquired", &DynamicCalibrationResults::CalibrationQuality::dataAquired)
        .def_readwrite("coverageQuality", &DynamicCalibrationResults::CalibrationQuality::coverageQuality)
        .def_readwrite("calibrationQuality", &DynamicCalibrationResults::CalibrationQuality::calibrationQuality);

    // Bind CalibrationQualityResult
    _CalibrationQualityResult
        .def(py::init<>())
        .def_readwrite("report", &DynamicCalibrationResults::CalibrationQualityResult::report);


    // Message
    _DynamicCalibrationResults
        .def(py::init<>())
        .def("reset", &DynamicCalibrationResults::reset)
        .def_readwrite("newCalibration", &DynamicCalibrationResults::newCalibration,
                       DOC(dai, DynamicCalibrationResults, newCalibration))
        .def_readwrite("calibOverallQuality", &DynamicCalibrationResults::calibOverallQuality,
                       DOC(dai, DynamicCalibrationResults, calibOverallQuality))
        .def_readwrite("info", &DynamicCalibrationResults::info,
                       DOC(dai, DynamicCalibrationResults, info));
        ;
}
