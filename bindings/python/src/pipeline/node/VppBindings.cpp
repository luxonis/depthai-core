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
    vppProperties
        .def_readwrite("initialConfig", &VppProperties::initialConfig, DOC(dai, VppProperties, initialConfig))
        .def_readwrite("numFramesPool", &VppProperties::numFramesPool, DOC(dai, VppProperties, numFramesPool));

    // Node
    vpp.def_readonly("inputConfig", &Vpp::inputConfig, DOC(dai, node, Vpp, inputConfig))
        .def_readonly("left", &Vpp::left, DOC(dai, node, Vpp, left))
        .def_readonly("right", &Vpp::right, DOC(dai, node, Vpp, right))
        .def_readonly("disparity", &Vpp::disparity, DOC(dai, node, Vpp, disparity))
        .def_readonly("leftOut", &Vpp::leftOut, DOC(dai, node, Vpp, leftOut))
        .def_readonly("rightOut", &Vpp::rightOut, DOC(dai, node, Vpp, rightOut))
        .def_readonly("initialConfig", &Vpp::initialConfig, DOC(dai, node, Vpp, initialConfig))

        .def("setMethod", &Vpp::setMethod, "method"_a, DOC(dai, node, Vpp, setMethod))
        .def("setPatchSize", &Vpp::setPatchSize, "size"_a, DOC(dai, node, Vpp, setPatchSize))
        .def("setAggregationWindowSize", &Vpp::setAggregationWindowSize, "sizeX"_a, "sizeY"_a, DOC(dai, node, Vpp, setAggregationWindowSize))
        .def("setProjectionDirection", &Vpp::setProjectionDirection, "left2right"_a, DOC(dai, node, Vpp, setProjectionDirection))
        .def("setBlending", &Vpp::setBlending, "blending"_a, "blendingOcclusion"_a = 0.0f, DOC(dai, node, Vpp, setBlending))
        .def("setDistancePatch", &Vpp::setDistancePatch, "enable"_a, "gamma"_a = 0.3f, DOC(dai, node, Vpp, setDistancePatch))
        .def("setBilateralPatch", &Vpp::setBilateralPatch, "enable"_a, "spatialSigma"_a = 2.0f, "intensitySigma"_a = 1.0f, "threshold"_a = 0.001f, DOC(dai, node, Vpp, setBilateralPatch))
        .def("setDisparityRange", &Vpp::setDisparityRange, "minThreshold"_a, "maxThreshold"_a, DOC(dai, node, Vpp, setDisparityRange))
        .def("setUniformColor", &Vpp::setUniformColor, "uniform"_a, DOC(dai, node, Vpp, setUniformColor))
        .def("setDiscardOcclusion", &Vpp::setDiscardOcclusion, "discard"_a, DOC(dai, node, Vpp, setDiscardOcclusion))
        .def("setInterpolate", &Vpp::setInterpolate, "interpolate"_a, DOC(dai, node, Vpp, setInterpolate))
        .def("setNumFramesPool", &Vpp::setNumFramesPool, "numFramesPool"_a, DOC(dai, node, Vpp, setNumFramesPool));

    // ALIAS
    daiNodeModule.attr("Vpp").attr("Properties") = vppProperties;
}