#include "NodeBindings.hpp"
#include "Common.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/ImageManipV2.hpp"

void bind_imagemanipv2(pybind11::module& m, void* pCallstack){

    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    // TODO(themarpe) - properties
    auto imageManip = ADD_NODE(ImageManipV2);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // ImageManip Node
    imageManip
        .def_readonly("inputConfig", &ImageManipV2::inputConfig, DOC(dai, node, ImageManipV2, inputConfig))
        .def_readonly("inputImage", &ImageManipV2::inputImage, DOC(dai, node, ImageManipV2, inputImage))
        .def_readonly("out", &ImageManipV2::out, DOC(dai, node, ImageManipV2, out))
        .def_readonly("initialConfig", &ImageManipV2::initialConfig, DOC(dai, node, ImageManipV2, initialConfig))
        .def("setNumFramesPool", &ImageManipV2::setNumFramesPool, DOC(dai, node, ImageManipV2, setNumFramesPool))
        .def("setMaxOutputFrameSize", &ImageManipV2::setMaxOutputFrameSize, DOC(dai, node, ImageManipV2, setMaxOutputFrameSize))
        ;

}
