#include "Common.hpp"
#include "NodeBindings.hpp"

#if defined(DEPTHAI_HAVE_DYNAMIC_CALIBRATION_SUPPORT)

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

        .def_readonly("configInput", &DynamicCalibration::configInput, DOC(dai, node, DynamicCalibration, configInput))
        .def_readonly("qualityOutput", &DynamicCalibration::qualityOutput, DOC(dai, node, DynamicCalibration, qualityOutput))
        .def_readonly("coverageOutput", &DynamicCalibration::coverageOutput, DOC(dai, node, DynamicCalibration, coverageOutput))
        .def_readonly("calibrationOutput", &DynamicCalibration::calibrationOutput, DOC(dai, node, DynamicCalibration, calibrationOutput))
        .def_readonly("commandInput", &DynamicCalibration::commandInput, DOC(dai, node, DynamicCalibration, commandInput))
        .def_property_readonly(
            "left", [](DynamicCalibration& node) { return &node.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "right", [](DynamicCalibration& node) { return &node.right; }, py::return_value_policy::reference_internal)

        .def("setRunOnHost", &DynamicCalibration::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, DynamicCalibration, setRunOnHost))
        .def("runOnHost", &DynamicCalibration::runOnHost, DOC(dai, node, DynamicCalibration, runOnHost));

    daiNodeModule.attr("DynamicCalibration").attr("Properties") = DynamicCalibrationProperties;
}

#else
  // Feature OFF: provide a stub so the symbol resolves if it’s called unconditionally.
  void bind_dynamic_calibration(pybind11::module&, void*) {}
#endif
