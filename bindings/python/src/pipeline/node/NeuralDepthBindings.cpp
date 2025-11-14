#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"

void bind_neuraldepth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<NeuralDepthProperties> properties(m, "NeuralDepthProperties", DOC(dai, NeuralDepthProperties));

    auto node = ADD_NODE(NeuralDepth);

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

    // Node
    node.def_property_readonly(
            "left",
            [](const NeuralDepth& n) { return &n.sync->inputs["left"]; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralDepth, left))
        .def_property_readonly(
            "right",
            [](const NeuralDepth& n) { return &n.sync->inputs["right"]; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralDepth, right))
        .def_property_readonly(
            "rectifiedLeft",
            [](const NeuralDepth& n) { return &n.rectification->output1; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralDepth, rectifiedLeft))
        .def_property_readonly(
            "rectifiedRight",
            [](const NeuralDepth& n) { return &n.rectification->output2; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralDepth, rectifiedRight))
        .def_static("getInputSize", &NeuralDepth::getInputSize, py::arg("model"), DOC(dai, node, NeuralDepth, getInputSize))
        .def("setRectification", &NeuralDepth::setRectification, py::arg("enable"), DOC(dai, node, NeuralDepth, setRectification))
        .def_readonly("inputConfig", &NeuralDepth::inputConfig, DOC(dai, node, NeuralDepth, inputConfig))
        .def_readonly("initialConfig", &NeuralDepth::initialConfig, DOC(dai, node, NeuralDepth, initialConfig))
        .def_readonly("disparity", &NeuralDepth::disparity, DOC(dai, node, NeuralDepth, disparity), DOC(dai, node, NeuralDepth, disparity))
        .def_readonly("depth", &NeuralDepth::depth, DOC(dai, node, NeuralDepth, depth), DOC(dai, node, NeuralDepth, depth))
        .def_readonly("edge", &NeuralDepth::edge, DOC(dai, node, NeuralDepth, edge), DOC(dai, node, NeuralDepth, edge))
        .def_readonly("confidence", &NeuralDepth::confidence, DOC(dai, node, NeuralDepth, confidence), DOC(dai, node, NeuralDepth, confidence))
        .def("build",
             &NeuralDepth::build,
             py::arg("leftInput"),
             py::arg("rightInput"),
             py::arg("model") = DeviceModelZoo::NEURAL_DEPTH_SMALL,
             DOC(dai, node, NeuralDepth, build))
        .def_property_readonly(
            "sync", [](NeuralDepth& n) { return &(*n.sync); }, py::return_value_policy::reference_internal, DOC(dai, node, NeuralDepth, sync))
        .def_property_readonly(
            "neuralNetwork",
            [](NeuralDepth& n) { return &(*n.neuralNetwork); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralDepth, neuralNetwork))
        .def_property_readonly(
            "messageDemux",
            [](NeuralDepth& n) { return &(*n.messageDemux); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralDepth, messageDemux))
        .def_property_readonly(
            "rectification",
            [](NeuralDepth& n) { return &(*n.rectification); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralDepth, rectification));
    // ALIAS
    daiNodeModule.attr("NeuralDepth").attr("Properties") = properties;
}
