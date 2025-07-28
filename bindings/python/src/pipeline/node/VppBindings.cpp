#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Vpp.hpp"

void bind_vpp(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // VppConfig enum
    py::enum_<VppConfig::Method>(m, "VppMethod", DOC(dai, VppConfig, Method))
        .value("RANDOM", VppConfig::Method::RANDOM, DOC(dai, VppConfig, Method, RANDOM))
        .value("MAX_DISTANCE", VppConfig::Method::MAX_DISTANCE, DOC(dai, VppConfig, Method, MAX_DISTANCE));

    // VppConfig class
    py::class_<VppConfig, std::shared_ptr<VppConfig>, Buffer> vppConfig(m, "VppConfig", DOC(dai, VppConfig));

    // Node and Properties declare upfront
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

    // VppConfig
    vppConfig
        .def(py::init<>(), DOC(dai, VppConfig, VppConfig))
        .def_readwrite("method", &VppConfig::method, DOC(dai, VppConfig, method))
        .def_readwrite("patchSize", &VppConfig::patchSize, DOC(dai, VppConfig, patchSize))
        .def_readwrite("windowSizeAggX", &VppConfig::windowSizeAggX, DOC(dai, VppConfig, windowSizeAggX))
        .def_readwrite("windowSizeAggY", &VppConfig::windowSizeAggY, DOC(dai, VppConfig, windowSizeAggY))
        .def_readwrite("left2right", &VppConfig::left2right, DOC(dai, VppConfig, left2right))
        .def_readwrite("blending", &VppConfig::blending, DOC(dai, VppConfig, blending))
        .def_readwrite("blendingOcclusion", &VppConfig::blendingOcclusion, DOC(dai, VppConfig, blendingOcclusion))
        .def_readwrite("useDistancePatch", &VppConfig::useDistancePatch, DOC(dai, VppConfig, useDistancePatch))
        .def_readwrite("useBilateralPatch", &VppConfig::useBilateralPatch, DOC(dai, VppConfig, useBilateralPatch))
        .def_readwrite("distanceGamma", &VppConfig::distanceGamma, DOC(dai, VppConfig, distanceGamma))
        .def_readwrite("bilateralSpatialSigma", &VppConfig::bilateralSpatialSigma, DOC(dai, VppConfig, bilateralSpatialSigma))
        .def_readwrite("bilateralIntensitySigma", &VppConfig::bilateralIntensitySigma, DOC(dai, VppConfig, bilateralIntensitySigma))
        .def_readwrite("bilateralThreshold", &VppConfig::bilateralThreshold, DOC(dai, VppConfig, bilateralThreshold))
        .def_readwrite("uniformColor", &VppConfig::uniformColor, DOC(dai, VppConfig, uniformColor))
        .def_readwrite("discardOcclusion", &VppConfig::discardOcclusion, DOC(dai, VppConfig, discardOcclusion))
        .def_readwrite("interpolate", &VppConfig::interpolate, DOC(dai, VppConfig, interpolate))
        .def_readwrite("disparityMinThreshold", &VppConfig::disparityMinThreshold, DOC(dai, VppConfig, disparityMinThreshold))
        .def_readwrite("disparityMaxThreshold", &VppConfig::disparityMaxThreshold, DOC(dai, VppConfig, disparityMaxThreshold))
        
        .def("setMethod", &VppConfig::setMethod, "method"_a, DOC(dai, VppConfig, setMethod))
        .def("setPatchSize", &VppConfig::setPatchSize, "size"_a, DOC(dai, VppConfig, setPatchSize))
        .def("setAggregationWindowSize", &VppConfig::setAggregationWindowSize, "sizeX"_a, "sizeY"_a, DOC(dai, VppConfig, setAggregationWindowSize))
        .def("setProjectionDirection", &VppConfig::setProjectionDirection, "left2right"_a, DOC(dai, VppConfig, setProjectionDirection))
        .def("setBlending", &VppConfig::setBlending, "blending"_a, "blendingOcclusion"_a, DOC(dai, VppConfig, setBlending))
        .def("setDistancePatch", &VppConfig::setDistancePatch, "enable"_a, "gamma"_a = 0.3f, DOC(dai, VppConfig, setDistancePatch))
        .def("setBilateralPatch", &VppConfig::setBilateralPatch, "enable"_a, "spatialSigma"_a = 2.0f, "intensitySigma"_a = 1.0f, "threshold"_a = 0.001f, DOC(dai, VppConfig, setBilateralPatch))
        .def("setDisparityRange", &VppConfig::setDisparityRange, "minThreshold"_a, "maxThreshold"_a, DOC(dai, VppConfig, setDisparityRange));

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