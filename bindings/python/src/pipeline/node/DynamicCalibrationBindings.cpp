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

    // Create/bind the node class first; this returns a py::class_<DynamicCalibration,...>
    auto dynamicCalibration = ADD_NODE(DynamicCalibration);

    // Here we bind PerformanceMode inside DynamicCalibration
    {
        namespace dn = dai::node;

        // After `auto dynamicCalibration = ADD_NODE(DynamicCalibration);`

        py::enum_<dn::DynamicCalibration::PerformanceMode>(dynamicCalibration, "PerformanceMode")
            .value("DEFAULT", dn::DynamicCalibration::PerformanceMode::DEFAULT)
            .value("STATIC_SCENERY", dn::DynamicCalibration::PerformanceMode::STATIC_SCENERY)
            .value("OPTIMIZE_SPEED", dn::DynamicCalibration::PerformanceMode::OPTIMIZE_SPEED)
            .value("OPTIMIZE_PERFORMANCE", dn::DynamicCalibration::PerformanceMode::OPTIMIZE_PERFORMANCE)
            .value("SKIP_CHECKS", dn::DynamicCalibration::PerformanceMode::SKIP_CHECKS)
            .export_values();
    }

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
    dynamicCalibration.def_readonly("qualityOutput", &DynamicCalibration::qualityOutput, DOC(dai, node, DynamicCalibration, qualityOutput))
        .def_readonly("coverageOutput", &DynamicCalibration::coverageOutput, DOC(dai, node, DynamicCalibration, coverageOutput))
        .def_readonly("calibrationOutput", &DynamicCalibration::calibrationOutput, DOC(dai, node, DynamicCalibration, calibrationOutput))
        .def_readonly("inputControl", &DynamicCalibration::inputControl, DOC(dai, node, DynamicCalibration, inputControl))
        .def_property_readonly(
            "left", [](DynamicCalibration& node) { return &node.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "right", [](DynamicCalibration& node) { return &node.right; }, py::return_value_policy::reference_internal)
        .def("setRunOnHost", &DynamicCalibration::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, DynamicCalibration, setRunOnHost))
        .def("runOnHost", &DynamicCalibration::runOnHost, DOC(dai, node, DynamicCalibration, runOnHost));

    // Keep the Properties class attached to the node type
    daiNodeModule.attr("DynamicCalibration").attr("Properties") = DynamicCalibrationProperties;
}
