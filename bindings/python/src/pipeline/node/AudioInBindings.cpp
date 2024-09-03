#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AudioIn.hpp"

void bind_audioin(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Declare node upfront
    auto audioIn = ADD_NODE(AudioIn);

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    audioIn.def_readonly("input", &AudioIn::input, DOC(dai, node, AudioIn, input))
        .def("build", &AudioIn::build, DOC(dai, node, AudioIn, build))
        .def("setDeviceName", &AudioIn::setDeviceName, DOC(dai, node, AudioIn, setDeviceName))
        .def("setDevicePath", &AudioIn::setDevicePath, DOC(dai, node, AudioIn, setDevicePath))
        .def("setBitrate", &AudioIn::setBitrate, DOC(dai, node, AudioIn, setBitrate))
        .def("setFps", &AudioIn::setFps, DOC(dai, node, AudioIn, setFps))
        .def("setChannels", &AudioIn::setChannels, DOC(dai, node, AudioIn, setChannels))
        .def("setFormat", &AudioIn::setFormat, DOC(dai, node, AudioIn, setFormat))
	.def("getDeviceName", &AudioIn::getDeviceName, DOC(dai, node, AudioIn, getDeviceName))
        .def("getDevicePath", &AudioIn::getDevicePath, DOC(dai, node, AudioIn, getDevicePath))
        .def("getBitrate", &AudioIn::getBitrate, DOC(dai, node, AudioIn, getBitrate))
        .def("getFps", &AudioIn::getFps, DOC(dai, node, AudioIn, getFps))
        .def("getChannels", &AudioIn::getChannels, DOC(dai, node, AudioIn, getChannels))
        .def("getFormat", &AudioIn::getFormat, DOC(dai, node, AudioIn, getFormat))
	;
}
