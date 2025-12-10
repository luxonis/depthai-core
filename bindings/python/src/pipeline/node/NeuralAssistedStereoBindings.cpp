#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"

void bind_neuralassistedstereo(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<NeuralAssistedStereoProperties> properties(m, "NeuralAssistedStereoProperties", DOC(dai, NeuralAssistedStereoProperties));

    auto node = ADD_NODE(NeuralAssistedStereo);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Properties
    properties.def_readwrite("vppConfig", &NeuralAssistedStereoProperties::vppConfig, DOC(dai, NeuralAssistedStereoProperties, vppConfig))
        .def_readwrite("stereoConfig", &NeuralAssistedStereoProperties::stereoConfig, DOC(dai, NeuralAssistedStereoProperties, stereoConfig));

    // Node
    node.def_property_readonly(
            "left",
            [](const NeuralAssistedStereo& n) { return &n.rectification->input1; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, left))
        .def_property_readonly(
            "right",
            [](const NeuralAssistedStereo& n) { return &n.rectification->input2; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, right))
        .def_property_readonly(
            "rectifiedLeft",
            [](const NeuralAssistedStereo& n) { return &n.rectification->output1; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, rectifiedLeft))
        .def_property_readonly(
            "rectifiedRight",
            [](const NeuralAssistedStereo& n) { return &n.rectification->output2; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, rectifiedRight))
        .def_property_readonly(
            "vppLeft",
            [](const NeuralAssistedStereo& n) { return &n.vpp->leftOut; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, vppLeft))
        .def_property_readonly(
            "vppRight",
            [](const NeuralAssistedStereo& n) { return &n.vpp->rightOut; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, vppRight))
        .def_property_readonly(
            "neuralDisparity",
            [](const NeuralAssistedStereo& n) { return &n.neuralDepth->disparity; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, neuralDisparity))
        .def_property_readonly(
            "neuralConfidence",
            [](const NeuralAssistedStereo& n) { return &n.neuralDepth->confidence; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, neuralConfidence))
        .def_property_readonly(
            "inputVppConfig",
            [](const NeuralAssistedStereo& n) { return &n.inputVppConfig; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, inputVppConfig))
        .def_property_readonly(
            "inputStereoConfig",
            [](const NeuralAssistedStereo& n) { return &n.inputStereoConfig; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, inputStereoConfig))
        .def_property_readonly(
            "inputNeuralConfig",
            [](const NeuralAssistedStereo& n) { return &n.inputNeuralConfig; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, inputNeuralConfig))
        .def_readonly("vppConfig", &NeuralAssistedStereo::vppConfig, DOC(dai, node, NeuralAssistedStereo, vppConfig))
        .def_readonly("stereoConfig", &NeuralAssistedStereo::stereoConfig, DOC(dai, node, NeuralAssistedStereo, stereoConfig))
        .def_readonly("neuralConfig", &NeuralAssistedStereo::neuralConfig, DOC(dai, node, NeuralAssistedStereo, neuralConfig))
        .def_property_readonly("depth", [](NeuralAssistedStereo& self) -> Node::Output& { return self.depth; }, DOC(dai, node, NeuralAssistedStereo, depth))
        .def_property_readonly("disparity", [](NeuralAssistedStereo& self) -> Node::Output& { return self.disparity; }, DOC(dai, node, NeuralAssistedStereo, disparity))
        .def("build",
             &NeuralAssistedStereo::build,
             py::arg("leftInput"),
             py::arg("rightInput"),
             py::arg("neuralModel") = DeviceModelZoo::NEURAL_DEPTH_NANO,
             DOC(dai, node, NeuralAssistedStereo, build))
        .def_property_readonly(
            "rectification",
            [](NeuralAssistedStereo& n) { return &(*n.rectification); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, rectification))
        .def_property_readonly(
            "neuralDepth",
            [](NeuralAssistedStereo& n) { return &(*n.neuralDepth); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, neuralDepth))
        .def_property_readonly(
            "vpp", [](NeuralAssistedStereo& n) { return &(*n.vpp); }, py::return_value_policy::reference_internal, DOC(dai, node, NeuralAssistedStereo, vpp))
        .def_property_readonly(
            "stereoDepth",
            [](NeuralAssistedStereo& n) { return &(*n.stereoDepth); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralAssistedStereo, stereoDepth));

    // ALIAS
    daiNodeModule.attr("NeuralAssistedStereo").attr("Properties") = properties;
}
