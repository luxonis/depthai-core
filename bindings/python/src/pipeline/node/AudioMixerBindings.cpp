#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AudioMixer.hpp"

void bind_audiomixer(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Declare node upfront
    auto audioMixer = ADD_NODE(AudioMixer);

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    audioMixer.def_readonly("outputs", &AudioMixer::outputs, DOC(dai, node, AudioMixer, outputs))
        .def("inputs", &AudioMixer::inputs, DOC(dai, node, AudioMixer, inputs))
        .def("build", &AudioMixer::build,DOC(dai, node, AudioMixer, build))
        .def("registerSource", &AudioMixer::registerSource, DOC(dai, node, AudioMixer, registerSource))
        .def("registerSink", &AudioMixer::registerSink, DOC(dai, node, AudioMixer, registerSink))
        .def("linkSourceToSink", &AudioMixer::linkSourceToSink, DOC(dai, node, AudioMixer, linkSourceToSink))
	.def("unregisterSource", &AudioMixer::unregisterSource, DOC(dai, node, AudioMixer, unregisterSource))
        .def("unregisterSink", &AudioMixer::unregisterSink, DOC(dai, node, AudioMixer, unregisterSink))
        .def("unlinkSourceFromSink", &AudioMixer::unlinkSourceFromSink, DOC(dai, node, AudioMixer, unlinkSourceFromSink))
	;

}
