#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AudioReplay.hpp"

void bind_audioreplay(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Declare node upfront
    auto audioReplay = ADD_NODE_DERIVED(AudioReplay, ThreadedHostNode);

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    audioReplay.def_readonly("out", &AudioReplay::out, DOC(dai, node, AudioReplay, out))
        .def("setSourceFile", &AudioReplay::setSourceFile, DOC(dai, node, AudioReplay, setSourceFile))
        .def("setLoop", &AudioReplay::setLoop, DOC(dai, node, AudioReplay, setLoop))
        .def("setFps", &AudioReplay::setFps, DOC(dai, node, AudioReplay, setFps))
	.def("getSourceFile", &AudioReplay::getSourceFile, DOC(dai, node, AudioReplay, getSourceFile))
        .def("getLoop", &AudioReplay::getLoop, DOC(dai, node, AudioReplay, getLoop))
        .def("getFps", &AudioReplay::getFps, DOC(dai, node, AudioReplay, getFps))
        .def("getFormat", &AudioReplay::getFormat, DOC(dai, node, AudioReplay, getFormat))
	;
}
