#include "Common.hpp"
#include "depthai/pipeline/node/host/VideoDecoder.hpp"

void bind_videodecoder(pybind11::module& m, void* pCallstack) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    using namespace dai;
    using namespace node;
    auto videoDecoder = ADD_NODE_DERIVED(VideoDecoder, ThreadedHostNode);
#endif

    // Declare all types before binding methods that reference them.
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    videoDecoder.def_readonly("input", &VideoDecoder::input, DOC(dai, node, VideoDecoder, input))
        .def_readonly("out", &VideoDecoder::out, DOC(dai, node, VideoDecoder, out))
        .def("setOutFrameType",
             &VideoDecoder::setOutFrameType,
             py::arg("type"),
             py::return_value_policy::reference_internal,
             DOC(dai, node, VideoDecoder, setOutFrameType))
        .def("getOutFrameType", &VideoDecoder::getOutFrameType, DOC(dai, node, VideoDecoder, getOutFrameType));
#endif
}
