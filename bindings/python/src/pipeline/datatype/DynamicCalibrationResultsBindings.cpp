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

    py::class_<DynamicCalibrationResults::QualityResult> _QualityResult(
        _DynamicCalibrationResults, "QualityResult", DOC(dai, DynamicCalibrationResults, QualityResult));

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
    _CalibrationResult.def(py::init<>())
        .def_readwrite("eepromData",
                       &DynamicCalibrationResults::CalibrationResult::eepromData,
                       DOC(dai, DynamicCalibrationResults, CalibrationResult, eepromData))
        .def_readwrite("valid",
                       &DynamicCalibrationResults::CalibrationResult::valid,
                       DOC(dai, DynamicCalibrationResults, CalibrationResult, valid))
        .def_readwrite("info",
                       &DynamicCalibrationResults::CalibrationResult::info,
                       DOC(dai, DynamicCalibrationResults, CalibrationResult, info))
        .def("getCalibration",
             &DynamicCalibrationResults::CalibrationResult::getCalibration,
             DOC(dai, DynamicCalibrationResults, CalibrationResult, getCalibration))
        .def("setCalibration",
                &DynamicCalibrationResults::CalibrationResult::setCalibration,
                py::arg("calibration"),
                DOC(dai, DynamicCalibrationResults, CalibrationResult, setCalibration))
        ;

    _QualityResult.def(py::init<>())
        .def_readwrite("value",
                       &DynamicCalibrationResults::QualityResult::value,
                       DOC(dai, DynamicCalibrationResults, QualityResult, value))
        .def_readwrite("valid",
                       &DynamicCalibrationResults::QualityResult::valid,
                       DOC(dai, DynamicCalibrationResults, QualityResult, valid))
        .def_readwrite("info",
                    &DynamicCalibrationResults::QualityResult::info,
                    DOC(dai, DynamicCalibrationResults, QualityResult, info));

    // Message
    _DynamicCalibrationResults.def(py::init<>())
        .def("__repr__", &DynamicCalibrationResults::str)
        .def("reset", &DynamicCalibrationResults::reset, DOC(dai, DynamicCalibrationResults, reset))
        .def_readwrite("quality",
                       &DynamicCalibrationResults::quality,
                       DOC(dai, DynamicCalibrationResults, quality))
        .def_readwrite("calibration",
                       &DynamicCalibrationResults::calibration,
                       DOC(dai, DynamicCalibrationResults, calibration))
        ;
}
