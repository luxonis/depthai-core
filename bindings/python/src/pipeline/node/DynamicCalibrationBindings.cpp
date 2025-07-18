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
    // Node
    dynamicCalibration
        // .def_property_readonly(
        //           "left", [](DynamicCalibration& node) { return &node.left; }, py::return_value_policy::reference_internal)
        // .def_property_readonly(
        //     "right", [](DynamicCalibration& node) { return &node.right; }, py::return_value_policy::reference_internal)
        // .def_readonly("inputConfig", &DynamicCalibration::inputConfig, DOC(dai, node, DynamicCalibration, inputConfig))
        .def_readonly("outputCalibrationResults", &DynamicCalibration::outputCalibrationResults, DOC(dai, node, DynamicCalibration, outputCalibrationResults))
        .def("setRunOnHost", &DynamicCalibration::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, DynamicCalibration, setRunOnHost))
        .def("runOnHost", &DynamicCalibration::runOnHost, DOC(dai, node, DynamicCalibration, runOnHost))
        .def("setPerformanceMode", &DynamicCalibration::setPerformanceMode, py::arg("mode"), DOC(dai, node, DynamicCalibration, setPerformanceMode))
        .def("setContinuousMode", &DynamicCalibration::setContinuousMode, DOC(dai, node, DynamicCalibration, setContinuousMode))
        .def("setTimeFrequency", &DynamicCalibration::setTimeFrequency, py::arg("int"), DOC(dai, node, DynamicCalibration, setTimeFrequency));
    daiNodeModule.attr("DynamicCalibration").attr("Properties") = DynamicCalibrationProperties;
}
