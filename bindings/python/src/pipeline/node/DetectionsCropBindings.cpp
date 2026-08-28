#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/DetectionsCrop.hpp"

void bind_detectionscrop(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto detectionsCrop = ADD_NODE_DERIVED(DetectionsCrop, DeviceNodeGroup);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    detectionsCrop
        .def_property_readonly(
            "inputDetections",
            [](DetectionsCrop& self) { return &self.inputDetections; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, DetectionsCrop, inputDetections))
        .def_property_readonly(
            "inputImage",
            [](DetectionsCrop& self) { return &self.inputImage; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, DetectionsCrop, inputImage))
        .def_property_readonly(
            "out", [](DetectionsCrop& self) { return &self.out; }, py::return_value_policy::reference_internal, DOC(dai, node, DetectionsCrop, out))
        .def("runOnHost", &DetectionsCrop::runOnHost, DOC(dai, node, DetectionsCrop, runOnHost))
        .def("setRunOnHost", &DetectionsCrop::setRunOnHost, py::arg("runOnHost") = true, DOC(dai, node, DetectionsCrop, setRunOnHost));
}
