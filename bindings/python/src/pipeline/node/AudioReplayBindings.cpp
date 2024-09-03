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
    auto audioReplay = ADD_NODE(AudioReplay);

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    audioReplay.def_readonly("input", &AudioReplay::input, DOC(dai, node, AudioReplay, input))
        .def("setSourceFile", &AudioReplay::setSourceFile, DOC(dai, node, AudioReplay, setSourceFile))
        .def("setSourceLoop", &AudioReplay::setSourceLoop, DOC(dai, node, AudioReplay, setSourceLoop))
        .def("setSourceFps", &AudioReplay::setSourceFps, DOC(dai, node, AudioReplay, setSourceFps))
	.def("getSourceFile", &AudioReplay::getSourceFile, DOC(dai, node, AudioReplay, getSourceFile))
        .def("getSourceLoop", &AudioReplay::getSourceLoop, DOC(dai, node, AudioReplay, getSourceLoop))
        .def("getSourceFps", &AudioReplay::getSourceFps, DOC(dai, node, AudioReplay, getSourceFps))
        .def("getFormat", &AudioReplay::getFormat, DOC(dai, node, AudioReplay, getFormat))
	;
}
