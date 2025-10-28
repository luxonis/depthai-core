#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"

void bind_neural_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<NeuralDepthProperties> properties(m, "NeuralDepthProperties", DOC(dai, NeuralDepthProperties));
    py::enum_<NeuralDepthProperties::ModelType> neuralDepthPropertiesModelType(
        properties, "ModelType", DOC(dai, NeuralDepthProperties, ModelType));
    neuralDepthPropertiesModelType
        .value("LARGE", NeuralDepthProperties::ModelType::LARGE)
        .value("MEDIUM", NeuralDepthProperties::ModelType::MEDIUM)
        .value("SMALL", NeuralDepthProperties::ModelType::SMALL)
        .value("NANO", NeuralDepthProperties::ModelType::NANO)
        ;

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

    // Properties
    properties.def_readwrite("modelType", &NeuralDepthProperties::modelType, DOC(dai, NeuralDepthProperties, modelType))
        ;

    // Node
    node.def_readonly("left", &NeuralDepth::left, DOC(dai, node, NeuralDepth, left), DOC(dai, node, NeuralDepth, left))
        .def_readonly("right", &NeuralDepth::right, DOC(dai, node, NeuralDepth, right), DOC(dai, node, NeuralDepth, right))
        .def_readonly("disparity", &NeuralDepth::disparity, DOC(dai, node, NeuralDepth, disparity), DOC(dai, node, NeuralDepth, disparity))
        .def_readonly("depth", &NeuralDepth::depth, DOC(dai, node, NeuralDepth, depth), DOC(dai, node, NeuralDepth, depth))
        .def_readonly("edge", &NeuralDepth::edge, DOC(dai, node, NeuralDepth, edge), DOC(dai, node, NeuralDepth, edge))
        .def_readonly("confidence", &NeuralDepth::confidence, DOC(dai, node, NeuralDepth, confidence), DOC(dai, node, NeuralDepth, confidence))
        .def("setModelType", &NeuralDepth::setModelType, DOC(dai, node, NeuralDepth, setModelType), DOC(dai, node, NeuralDepth, setModelType))
        .def("getModelType", &NeuralDepth::getModelType, DOC(dai, node, NeuralDepth, getModelType), DOC(dai, node, NeuralDepth, getModelType))
        ;
    // ALIAS
    daiNodeModule.attr("NeuralDepth").attr("Properties") = properties;
}
