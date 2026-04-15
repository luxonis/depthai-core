#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/common/DeviceModelZoo.hpp"

/// Python bindings for dai.node.Depth (composite StereoDepth / NeuralDepth group).
void bind_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = ADD_NODE_DERIVED(Depth, DeviceNodeGroup);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // C++ build() is a no-op (neural model for RVC4 is fixed in Depth::buildInternal). Queues before pipeline.build().
    node.def("build", py::overload_cast<DeviceModelZoo>(&Depth::build), py::arg("neuralModel") = DeviceModelZoo::NEURAL_DEPTH_SMALL)
        .def_property_readonly("depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal)
        .def("getStereoDepth", &Depth::getStereoDepth, py::return_value_policy::reference_internal,
             "Underlying StereoDepth on non-RVC4 platforms; None on RVC4. Lifetime tied to Depth.")
        .def("getNeuralDepth", &Depth::getNeuralDepth, py::return_value_policy::reference_internal,
             "Underlying NeuralDepth on RVC4; None on other platforms. Lifetime tied to Depth.");
}
