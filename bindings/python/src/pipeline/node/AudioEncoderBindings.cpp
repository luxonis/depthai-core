#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AudioEncoder.hpp"

void bind_audioencoder(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Declare node upfront
    auto audioEncoder = ADD_NODE(AudioEncoder);

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    audioEncoder.def_readonly("input", &AudioEncoder::input, DOC(dai, node, AudioEncoder, input))
        .def_readonly("out", &AudioEncoder::out, DOC(dai, node, AudioEncoder, out))
        .def("build", &AudioEncoder::build, DOC(dai, node, AudioEncoder, build))
        .def("setBitrate", &AudioEncoder::setBitrate, DOC(dai, node, AudioEncoder, setBitrate))
        .def("setChannels", &AudioEncoder::setChannels, DOC(dai, node, AudioEncoder, setChannels))
        .def("setFormat", &AudioEncoder::setFormat, DOC(dai, node, AudioEncoder, setFormat))
        .def("getBitrate", &AudioEncoder::getBitrate, DOC(dai, node, AudioEncoder, getBitrate))
        .def("getChannels", &AudioEncoder::getChannels, DOC(dai, node, AudioEncoder, getChannels))
        .def("getFormat", &AudioEncoder::getFormat, DOC(dai, node, AudioEncoder, getFormat))
	.def("setRunOnHost", &AudioEncoder::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, AudioEncoder, setRunOnHost))
        .def("runOnHost", &AudioEncoder::runOnHost, DOC(dai, node, AudioEncoder, runOnHost))
	;
}
