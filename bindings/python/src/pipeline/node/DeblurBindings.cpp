#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/node/Deblur.hpp"
void bind_deblur(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    py::class_<DeblurProperties> props(m, "DeblurProperties", DOC(dai, DeblurProperties));
    auto node = ADD_NODE(Deblur);
    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    props.def(py::init<>())
        .def_readwrite("strength", &DeblurProperties::strength, DOC(dai, DeblurProperties, strength))
        .def_readwrite("regularization", &DeblurProperties::regularization, DOC(dai, DeblurProperties, regularization))
        .def_readwrite("maxGain", &DeblurProperties::maxGain, DOC(dai, DeblurProperties, maxGain))
        .def_readwrite("minBlur", &DeblurProperties::minBlur, DOC(dai, DeblurProperties, minBlur))
        .def_readwrite("maxBlur", &DeblurProperties::maxBlur, DOC(dai, DeblurProperties, maxBlur))
        .def_readwrite("motionQuantum", &DeblurProperties::motionQuantum, DOC(dai, DeblurProperties, motionQuantum))
        .def_readwrite("forwardBackwardThreshold", &DeblurProperties::forwardBackwardThreshold, DOC(dai, DeblurProperties, forwardBackwardThreshold))
        .def_readwrite("photometricThreshold", &DeblurProperties::photometricThreshold, DOC(dai, DeblurProperties, photometricThreshold))
        .def_readwrite("minTextureVariance", &DeblurProperties::minTextureVariance, DOC(dai, DeblurProperties, minTextureVariance))
        .def_readwrite("minConfidence", &DeblurProperties::minConfidence, DOC(dai, DeblurProperties, minConfidence))
        .def_readwrite("maxCorrection", &DeblurProperties::maxCorrection, DOC(dai, DeblurProperties, maxCorrection))
        .def_readwrite("kernelRadius", &DeblurProperties::kernelRadius, DOC(dai, DeblurProperties, kernelRadius))
        .def_readwrite("maxClusters", &DeblurProperties::maxClusters, DOC(dai, DeblurProperties, maxClusters))
        .def_readwrite("exposureTimeUs", &DeblurProperties::exposureTimeUs, DOC(dai, DeblurProperties, exposureTimeUs))
        .def_readwrite("maxWaitMs", &DeblurProperties::maxWaitMs, DOC(dai, DeblurProperties, maxWaitMs))
        .def_readwrite("cacheSize", &DeblurProperties::cacheSize, DOC(dai, DeblurProperties, cacheSize))
        .def_readwrite("useOpenCL", &DeblurProperties::useOpenCL, DOC(dai, DeblurProperties, useOpenCL));
    node.def_readonly("inputImage", &Deblur::inputImage, DOC(dai, node, Deblur, inputImage))
        .def_readonly("inputFlow", &Deblur::inputFlow, DOC(dai, node, Deblur, inputFlow))
        .def_readonly("output", &Deblur::output, DOC(dai, node, Deblur, output))
        .def_readonly("passthroughInputImage", &Deblur::passthroughInputImage, DOC(dai, node, Deblur, passthroughInputImage))
        .def_readonly("outputMask", &Deblur::outputMask, DOC(dai, node, Deblur, outputMask))
        .def("setConfig", &Deblur::setConfig, py::arg("config"), py::return_value_policy::reference_internal, DOC(dai, node, Deblur, setConfig))
        .def("getConfig", &Deblur::getConfig, DOC(dai, node, Deblur, getConfig));
    daiNodeModule.attr("Deblur").attr("Properties") = props;
}
