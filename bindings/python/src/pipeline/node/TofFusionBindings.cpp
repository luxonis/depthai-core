#include "Common.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/TofFusion.hpp"

void bind_toffusion(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Node and Properties declare upfront
    py::class_<TofFusionProperties> tofFusionProperties(m, "TofFusionProperties", DOC(dai, TofFusionProperties));
    auto tofFusion = ADD_NODE(TofFusion);

    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////

    // Properties
    tofFusionProperties.def_readwrite("initialConfig", &TofFusionProperties::initialConfig, DOC(dai, TofFusionProperties, initialConfig))
        .def_readwrite("numFramesPool", &TofFusionProperties::numFramesPool, DOC(dai, TofFusionProperties, numFramesPool));

    // TofFusion Node
    tofFusion.def_readonly("inputConfig", &TofFusion::inputConfig, DOC(dai, node, TofFusion, inputConfig))
        .def_readonly("inputTexture", &TofFusion::inputTexture, DOC(dai, node, TofFusion, inputTexture))
        .def_readonly("inputTofDepth", &TofFusion::inputTofDepth, DOC(dai, node, TofFusion, inputTofDepth))
        .def_readonly("inputTofConfidence", &TofFusion::inputTofConfidence, DOC(dai, node, TofFusion, inputTofConfidence))
        .def_readonly("inputNeuralDepth", &TofFusion::inputNeuralDepth, DOC(dai, node, TofFusion, inputNeuralDepth))
        .def_readonly("inputNeuralConfidence", &TofFusion::inputNeuralConfidence, DOC(dai, node, TofFusion, inputNeuralConfidence))
        .def_readonly("fusedDepth", &TofFusion::fusedDepth, DOC(dai, node, TofFusion, fusedDepth))
        .def_readonly("initialConfig", &TofFusion::initialConfig, DOC(dai, node, TofFusion, initialConfig));

    // ALIAS
    daiNodeModule.attr("TofFusion").attr("Properties") = tofFusionProperties;
}
