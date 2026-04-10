#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/common/DeviceModelZoo.hpp"

void bind_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = ADD_NODE_DERIVED(Depth, DeviceNodeGroup);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    node.def("build", py::overload_cast<DeviceModelZoo>(&Depth::build), py::arg("neuralModel") = DeviceModelZoo::NEURAL_DEPTH_SMALL)
        .def_property_readonly("depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal)
        .def("getStereoDepth", &Depth::getStereoDepth)
        .def("getNeuralDepth", &Depth::getNeuralDepth);
}
