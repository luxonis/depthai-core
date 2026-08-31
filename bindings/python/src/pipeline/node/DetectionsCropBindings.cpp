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
        .def_property_readonly(
            "cropConfigGenerator",
            [](DetectionsCrop& self) { return &*self.cropConfigGenerator; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, DetectionsCrop, cropConfigGenerator))
        .def_property_readonly(
            "imageManip",
            [](DetectionsCrop& self) { return &*self.imageManip; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, DetectionsCrop, imageManip));
}
