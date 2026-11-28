#include <pybind11/stl/filesystem.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

void bind_dynamic_calibration(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    namespace py = pybind11;

    py::class_<DynamicCalibrationProperties> DynamicCalibrationProperties(m, "DynamicCalibrationProperties", DOC(dai, DynamicCalibrationProperties));

    auto dynamicCalibration = ADD_NODE(DynamicCalibration);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    dynamicCalibration.def_readonly("qualityOutput", &DynamicCalibration::qualityOutput, DOC(dai, node, DynamicCalibration, qualityOutput))
        .def_readonly("coverageOutput", &DynamicCalibration::coverageOutput, DOC(dai, node, DynamicCalibration, coverageOutput))
        .def_readonly("metricsOutput", &DynamicCalibration::metricsOutput, DOC(dai, node, DynamicCalibration, metricsOutput))
        .def_readonly("calibrationOutput", &DynamicCalibration::calibrationOutput, DOC(dai, node, DynamicCalibration, calibrationOutput))
        .def_readonly("inputControl", &DynamicCalibration::inputControl, DOC(dai, node, DynamicCalibration, inputControl))
        .def_property_readonly(
            "sync", [](DynamicCalibration& node) { return &(*node.sync); }, py::return_value_policy::reference_internal)
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
        .def_property_readonly(
            "inputs", [](DynamicCalibration& node) { return &node.inputs; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "left", [](DynamicCalibration& node) { return &node.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "right", [](DynamicCalibration& node) { return &node.right; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "rgb", [](DynamicCalibration& node) { return &node.rgb; }, py::return_value_policy::reference_internal)
#endif
        .def("setRunOnHost", &DynamicCalibration::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, DynamicCalibration, setRunOnHost))
        .def("runOnHost", &DynamicCalibration::runOnHost, DOC(dai, node, DynamicCalibration, runOnHost))
        .def("setWorldConstraints", &DynamicCalibration::setWorldConstraints, py::arg("constraints"), DOC(dai, node, DynamicCalibration, setWorldConstraints))
        .def("setWorldConstraintsFromJson",
             &DynamicCalibration::setWorldConstraintsFromJson,
             py::arg("json"),
             DOC(dai, node, DynamicCalibration, setWorldConstraintsFromJson))
        .def("setWorldConstraintsFromFile",
             &DynamicCalibration::setWorldConstraintsFromFile,
             py::arg("path"),
             DOC(dai, node, DynamicCalibration, setWorldConstraintsFromFile))
        .def("clearWorldConstraints", &DynamicCalibration::clearWorldConstraints, DOC(dai, node, DynamicCalibration, clearWorldConstraints))
        .def("getWorldConstraints", &DynamicCalibration::getWorldConstraints, DOC(dai, node, DynamicCalibration, getWorldConstraints));

    daiNodeModule.attr("DynamicCalibration").attr("Properties") = DynamicCalibrationProperties;
}
