#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/FocusedDepth.hpp"
#include "depthai/pipeline/node/Sync.hpp"

void bind_focused_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = addNode<FocusedDepth, DeviceNodeGroup>("FocusedDepth", "RVC4-focused depth on a dynamic ROI (stereo or neural backend).");

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    py::enum_<FocusedDepth::Backend>(node, "Backend")
        .value("AUTO", FocusedDepth::Backend::AUTO)
        .value("STEREO", FocusedDepth::Backend::STEREO)
        .value("NEURAL", FocusedDepth::Backend::NEURAL);

    node.def_static("pickNeuralModelForRoi", &FocusedDepth::pickNeuralModelForRoi, py::arg("width"), py::arg("height"))
        .def("setBackend", &FocusedDepth::setBackend, py::arg("backend"))
        .def("getBackend", &FocusedDepth::getBackend)
        .def("setTargetFps", &FocusedDepth::setTargetFps, py::arg("fps"))
        .def("setMaxRoi", &FocusedDepth::setMaxRoi, py::arg("width"), py::arg("height"))
        .def("setNeuralModelOverride", &FocusedDepth::setNeuralModelOverride, py::arg("model"))
        .def_readonly("stereoInitialConfig", &FocusedDepth::stereoInitialConfig)
        .def_property_readonly("left", [](FocusedDepth& n) -> Node::Input& { return n.left; }, py::return_value_policy::reference_internal)
        .def_property_readonly("right", [](FocusedDepth& n) -> Node::Input& { return n.right; }, py::return_value_policy::reference_internal)
        .def_property_readonly("roi", [](FocusedDepth& n) -> Node::Input& { return n.roi; }, py::return_value_policy::reference_internal)
        .def_property_readonly("sync", [](FocusedDepth& n) -> Sync& { return *n.sync; }, py::return_value_policy::reference_internal)
        .def_property_readonly("depth", [](FocusedDepth& n) -> Node::Output& { return n.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](FocusedDepth& n) -> Node::Output& { return n.confidence(); }, py::return_value_policy::reference_internal)
        .def("getStereoDepth", &FocusedDepth::getStereoDepth, py::return_value_policy::reference_internal)
        .def("getNeuralDepth", &FocusedDepth::getNeuralDepth, py::return_value_policy::reference_internal);
}
