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

    // Node and Properties declare upfront
    py::class_<DynamicCalibrationProperties> DynamicCalibrationProperties(m, "DynamicCalibrationProperties", DOC(dai, DynamicCalibrationProperties));
    py::class_<DynamicCalibrationThreadPolicy> dynamicCalibrationThreadPolicy(m, "DynamicCalibrationThreadPolicy", DOC(dai, DynamicCalibrationThreadPolicy));

    // Create/bind the node class first; this returns a py::class_<DynamicCalibration,...>
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
    dynamicCalibrationThreadPolicy.def(py::init<>())
        .def_readwrite("schedulerThreads", &DynamicCalibrationThreadPolicy::schedulerThreads, DOC(dai, DynamicCalibrationThreadPolicy, schedulerThreads))
        .def_readwrite("openCvThreads", &DynamicCalibrationThreadPolicy::openCvThreads, DOC(dai, DynamicCalibrationThreadPolicy, openCvThreads))
        .def_readwrite("niceness", &DynamicCalibrationThreadPolicy::niceness, DOC(dai, DynamicCalibrationThreadPolicy, niceness))
        .def_readwrite("cpuAffinity", &DynamicCalibrationThreadPolicy::cpuAffinity, DOC(dai, DynamicCalibrationThreadPolicy, cpuAffinity));

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
        .def("setThreadPolicy", &DynamicCalibration::setThreadPolicy, py::arg("policy"), DOC(dai, node, DynamicCalibration, setThreadPolicy))
        .def("getThreadPolicy", &DynamicCalibration::getThreadPolicy, DOC(dai, node, DynamicCalibration, getThreadPolicy))
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

    // Keep the Properties class attached to the node type
    daiNodeModule.attr("DynamicCalibration").attr("Properties") = DynamicCalibrationProperties;
}
