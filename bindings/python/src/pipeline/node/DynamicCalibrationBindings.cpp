#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DynamicCalibration.hpp"

void bind_dynamic_calibration(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<DynamicCalibrationProperties> DynamicCalibrationProperties(m, "DynamicCalibrationProperties", DOC(dai, DynamicCalibrationProperties));
    auto dynamicCalibration = ADD_NODE(DynamicCalibration);

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
    // ---------- CalibrationResult ----------
    py::class_<CalibrationResult>(m, "CalibrationResult")
        .def(py::init<>())
        .def_readwrite("calibration", &CalibrationResult::calibration, DOC(dai, CalibrationResult, calibration))
        .def_readwrite("valid", &CalibrationResult::valid, DOC(dai, CalibrationResult, valid))
        .def_readwrite("info", &CalibrationResult::info, DOC(dai, CalibrationResult, info))
        .def_static("invalid", &CalibrationResult::Invalid, py::arg("reason") = "No result", DOC(dai, CalibrationResult, invalid));

    // ---------- QualityResult ----------
    py::class_<QualityResult>(m, "QualityResult")
        .def(py::init<>())
        .def_readwrite("value", &QualityResult::value, DOC(dai, QualityResult, value))
        .def_readwrite("valid", &QualityResult::valid, DOC(dai, QualityResult, valid))
        .def_readwrite("info", &QualityResult::info, DOC(dai, QualityResult, info))
        .def_static("invalid", &QualityResult::Invalid, py::arg("reason") = "No result", DOC(dai, QualityResult, invalid));

    // ---------- CalibrationResults ----------
    py::class_<CalibrationResults>(m, "CalibrationResults")
        .def(py::init<>())
        .def_readwrite("quality", &CalibrationResults::quality, DOC(dai, CalibrationResults, quality))
        .def_readwrite("calibration", &CalibrationResults::calibration, DOC(dai, CalibrationResults, calibration))
        .def("reset", &CalibrationResults::reset, DOC(dai, CalibrationResults, reset));
    // Node
    dynamicCalibration
        // .def_readonly("inputs", &DynamicCalibration::inputs, DOC(dai, node, DynamicCalibration, inputs))
        .def_readonly("left", &DynamicCalibration::left, DOC(dai, node, DynamicCalibration, left))
        .def_readonly("right", &DynamicCalibration::right, DOC(dai, node, DynamicCalibration, right))
        .def("setRunOnHost", &DynamicCalibration::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, DynamicCalibration, setRunOnHost))
        .def("runOnHost", &DynamicCalibration::runOnHost, DOC(dai, node, DynamicCalibration, runOnHost))
        .def("startCalibQualityCheck", &DynamicCalibration::startCalibQualityCheck, DOC(dai, node, DynamicCalibration, startCalibQualityCheck))
        .def("startRecalibration", &DynamicCalibration::startRecalibration, DOC(dai, node, DynamicCalibration, startRecalibration))
        .def("setContinuousMode", &DynamicCalibration::setContinuousMode, DOC(dai, node, DynamicCalibration, setContinuousMode))
        .def("getCalibQuality", &DynamicCalibration::getCalibQuality, DOC(dai, node, DynamicCalibration, getCalibQuality))
        .def("getNewCalibration", &DynamicCalibration::getNewCalibration, DOC(dai, node, DynamicCalibration, getNewCalibration));
    daiNodeModule.attr("DynamicCalibration").attr("Properties") = DynamicCalibrationProperties;
}
