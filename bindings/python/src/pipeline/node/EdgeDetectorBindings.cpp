#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/EdgeDetector.hpp"

void bind_edgedetector(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<EdgeDetectorProperties> edgeDetectorProperties(m, "EdgeDetectorProperties", DOC(dai, EdgeDetectorProperties));
    auto edgeDetector = ADD_NODE(EdgeDetector);

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
    edgeDetectorProperties.def_readwrite("initialConfig", &EdgeDetectorProperties::initialConfig, DOC(dai, EdgeDetectorProperties, initialConfig))
        .def_readwrite("outputFrameSize", &EdgeDetectorProperties::outputFrameSize, DOC(dai, EdgeDetectorProperties, outputFrameSize))
        .def_readwrite("numFramesPool", &EdgeDetectorProperties::numFramesPool, DOC(dai, EdgeDetectorProperties, numFramesPool));

    // Node
    edgeDetector.def_readonly("initialConfig", &EdgeDetector::initialConfig, DOC(dai, node, EdgeDetector, initialConfig))
        .def_readonly("inputConfig", &EdgeDetector::inputConfig, DOC(dai, node, EdgeDetector, inputConfig))
        .def_readonly("inputImage", &EdgeDetector::inputImage, DOC(dai, node, EdgeDetector, inputImage))
        .def_readonly("outputImage", &EdgeDetector::outputImage, DOC(dai, node, EdgeDetector, outputImage))
        .def("setNumFramesPool", &EdgeDetector::setNumFramesPool, DOC(dai, node, EdgeDetector, setNumFramesPool))
        .def("setMaxOutputFrameSize", &EdgeDetector::setMaxOutputFrameSize, DOC(dai, node, EdgeDetector, setMaxOutputFrameSize));
    daiNodeModule.attr("EdgeDetector").attr("Properties") = edgeDetectorProperties;
}
