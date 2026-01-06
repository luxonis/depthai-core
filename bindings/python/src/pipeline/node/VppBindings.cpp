#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Vpp.hpp"

void bind_vpp(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Node and Properties declare upfront (no duplicate VppConfig or VppMethod bindings)
    py::class_<VppProperties> vppProperties(m, "VppProperties", DOC(dai, VppProperties));
    auto vpp = ADD_NODE(Vpp);

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

    // Properties
    vppProperties.def_readwrite("initialConfig", &VppProperties::initialConfig, DOC(dai, VppProperties, initialConfig))
        .def_readwrite("numFramesPool", &VppProperties::numFramesPool, DOC(dai, VppProperties, numFramesPool));

    // Node
    vpp.def_readonly("inputConfig", &Vpp::inputConfig, DOC(dai, node, Vpp, inputConfig))
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
        .def_property_readonly(
            "left", [](Vpp& node) { return node.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "right", [](Vpp& node) { return node.right; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "disparity", [](Vpp& node) { return node.disparity; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "confidence", [](Vpp& node) { return node.confidence; }, py::return_value_policy::reference_internal)
        .def("build", &Vpp::build, py::arg("leftInput"), py::arg("rightInput"), py::arg("disparity"), py::arg("confidence"), DOC(dai, node, Vpp, build))
#endif
        .def_readonly("syncedInputs", &Vpp::syncedInputs, DOC(dai, node, Vpp, syncedInputs))
        .def_readonly("leftOut", &Vpp::leftOut, DOC(dai, node, Vpp, leftOut))
        .def_readonly("rightOut", &Vpp::rightOut, DOC(dai, node, Vpp, rightOut))
        .def_property(
            "initialConfig",
            [](Vpp& self) { return self.initialConfig; },
            [](Vpp& self, std::shared_ptr<VppConfig> cfg) { self.initialConfig = cfg; },
            DOC(dai, node, Vpp, initialConfig),
            "Initial config of the node.");

    // ALIAS
    daiNodeModule.attr("Vpp").attr("Properties") = vppProperties;
}
