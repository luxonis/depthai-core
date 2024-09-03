#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AudioOut.hpp"

void bind_audioout(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Declare node upfront
    auto audioOut = ADD_NODE(AudioOut);

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    audioOut.def_readonly("input", &AudioOut::input, DOC(dai, node, AudioOut, input))
        .def("build", &AudioOut::build, DOC(dai, node, AudioOut, build))
        .def("setDeviceName", &AudioOut::setDeviceName, DOC(dai, node, AudioOut, setDeviceName))
        .def("setDevicePath", &AudioOut::setDevicePath, DOC(dai, node, AudioOut, setDevicePath))
        .def("setBitrate", &AudioOut::setBitrate, DOC(dai, node, AudioOut, setBitrate))
        .def("setFps", &AudioOut::setFps, DOC(dai, node, AudioOut, setFps))
        .def("setChannels", &AudioOut::setChannels, DOC(dai, node, AudioOut, setChannels))
        .def("setFormat", &AudioOut::setFormat, DOC(dai, node, AudioOut, setFormat))
	.def("getDeviceName", &AudioOut::getDeviceName, DOC(dai, node, AudioOut, getDeviceName))
        .def("getDevicePath", &AudioOut::getDevicePath, DOC(dai, node, AudioOut, getDevicePath))
        .def("getBitrate", &AudioOut::getBitrate, DOC(dai, node, AudioOut, getBitrate))
        .def("getFps", &AudioOut::getFps, DOC(dai, node, AudioOut, getFps))
        .def("getChannels", &AudioOut::getChannels, DOC(dai, node, AudioOut, getChannels))
        .def("getFormat", &AudioOut::getFormat, DOC(dai, node, AudioOut, getFormat))
	;
}
