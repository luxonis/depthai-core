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
        .def_readonly("left", &Vpp::left, DOC(dai, node, Vpp, left))
        .def_readonly("right", &Vpp::right, DOC(dai, node, Vpp, right))
        .def_readonly("disparity", &Vpp::disparity, DOC(dai, node, Vpp, disparity))
        .def_readonly("leftOut", &Vpp::leftOut, DOC(dai, node, Vpp, leftOut))
        .def_readonly("rightOut", &Vpp::rightOut, DOC(dai, node, Vpp, rightOut))
        .def_readonly("initialConfig", &Vpp::initialConfig, DOC(dai, node, Vpp, initialConfig));

    // ALIAS
    daiNodeModule.attr("Vpp").attr("Properties") = vppProperties;
}
