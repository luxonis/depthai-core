#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Thermal.hpp"

void bind_thermal(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Node and Properties declare upfront
    py::class_<ThermalProperties> ThermalProperties(m, "ThermalProperties", DOC(dai, ThermalProperties));
    auto thermal = ADD_NODE(Thermal);

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
    ThermalProperties.def_readwrite("initialConfig", &ThermalProperties::initialConfig, DOC(dai, ThermalProperties, initialConfig))
        .def_readwrite("numFramesPool", &ThermalProperties::numFramesPool, DOC(dai, ThermalProperties, numFramesPool))
        .def_readwrite("boardSocket", &ThermalProperties::boardSocket, DOC(dai, ThermalProperties, boardSocket))
        .def_readwrite("fps", &ThermalProperties::fps, DOC(dai, ThermalProperties, fps));

    // Node
    thermal.def_readonly("initialConfig", &Thermal::initialConfig, DOC(dai, node, Thermal, initialConfig), DOC(dai, node, Thermal, initialConfig))
        .def_readonly("inputConfig", &Thermal::inputConfig, DOC(dai, node, Thermal, inputConfig), DOC(dai, node, Thermal, inputConfig))
        .def_readonly("temperature", &Thermal::temperature, DOC(dai, node, Thermal, temperature), DOC(dai, node, Thermal, temperature))
        .def_readonly("color", &Thermal::color, DOC(dai, node, Thermal, color), DOC(dai, node, Thermal, color))

        .def("build", &Thermal::build, "boardSocket"_a = CameraBoardSocket::AUTO, "fps"_a = float(25), DOC(dai, node, Thermal, build))
        .def("getBoardSocket", &Thermal::getBoardSocket, DOC(dai, node, Thermal, getBoardSocket));
    // ALIAS
    daiNodeModule.attr("Thermal").attr("Properties") = ThermalProperties;
}
