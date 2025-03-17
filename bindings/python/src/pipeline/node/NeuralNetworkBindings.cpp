#include <optional>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"

void bind_neuralnetwork(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<NeuralNetworkProperties, std::shared_ptr<NeuralNetworkProperties>> neuralNetworkProperties(
        m, "NeuralNetworkProperties", DOC(dai, NeuralNetworkProperties));
    auto neuralNetwork = ADD_NODE(NeuralNetwork);

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
    neuralNetworkProperties.def_readwrite("blobSize", &NeuralNetworkProperties::blobSize)
        .def_readwrite("blobUri", &NeuralNetworkProperties::blobUri)
        .def_readwrite("numFrames", &NeuralNetworkProperties::numFrames)
        .def_readwrite("numThreads", &NeuralNetworkProperties::numThreads)
        .def_readwrite("numNCEPerThread", &NeuralNetworkProperties::numNCEPerThread);

    // Node
    neuralNetwork.def_readonly("input", &NeuralNetwork::input, DOC(dai, node, NeuralNetwork, input))
        .def_readonly("out", &NeuralNetwork::out, DOC(dai, node, NeuralNetwork, out))
        .def_readonly("passthrough", &NeuralNetwork::passthrough, DOC(dai, node, NeuralNetwork, passthrough))
        .def("setBlobPath", &NeuralNetwork::setBlobPath, py::arg("path"), DOC(dai, node, NeuralNetwork, setBlobPath))
        .def("setNumPoolFrames", &NeuralNetwork::setNumPoolFrames, py::arg("numFrames"), DOC(dai, node, NeuralNetwork, setNumPoolFrames))
        .def("setNumInferenceThreads", &NeuralNetwork::setNumInferenceThreads, py::arg("numThreads"), DOC(dai, node, NeuralNetwork, setNumInferenceThreads))
        .def("setNumNCEPerInferenceThread",
             &NeuralNetwork::setNumNCEPerInferenceThread,
             py::arg("numNCEPerThread"),
             DOC(dai, node, NeuralNetwork, setNumNCEPerInferenceThread))
        .def("getNumInferenceThreads", &NeuralNetwork::getNumInferenceThreads, DOC(dai, node, NeuralNetwork, getNumInferenceThreads))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&>(&NeuralNetwork::setNNArchive),
             py::arg("nnArchive"),
             DOC(dai, node, NeuralNetwork, setNNArchive))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&, int>(&NeuralNetwork::setNNArchive),
             py::arg("nnArchive"),
             py::arg("numShaves"),
             DOC(dai, node, NeuralNetwork, setNNArchive, 2))
        .def("setFromModelZoo",
             py::overload_cast<NNModelDescription, bool>(&NeuralNetwork::setFromModelZoo),
             py::arg("description"),
             py::arg("useCached"),
             DOC(dai, node, NeuralNetwork, setFromModelZoo))
        .def("build",
             py::overload_cast<dai::Node::Output&, const NNArchive&>(&NeuralNetwork::build),
             py::arg("input"),
             py::arg("nnArchive"),
             DOC(dai, node, NeuralNetwork, build))
        .def("build",
             py::overload_cast<const std::shared_ptr<Camera>&, dai::NNModelDescription, std::optional<float>>(&NeuralNetwork::build),
             py::arg("input"),
             py::arg("modelDesc"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, NeuralNetwork, build, 2))
        .def("build",
             py::overload_cast<const std::shared_ptr<Camera>&, dai::NNArchive, std::optional<float>>(&NeuralNetwork::build),
             py::arg("input"),
             py::arg("model"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, NeuralNetwork, build, 3))
        .def(
            "build",
            [](NeuralNetwork& self, const std::shared_ptr<Camera>& input, const std::string& model, std::optional<float> fps) {
                return self.build(input, NNModelDescription{model}, fps);
            },
            py::arg("input"),
            py::arg("model"),
            py::arg("fps") = std::nullopt,
            DOC(dai, node, NeuralNetwork, build))
        .def("build",
             py::overload_cast<const std::shared_ptr<ReplayVideo>&, const NNArchive&, std::optional<float>>(&NeuralNetwork::build),
             py::arg("input"),
             py::arg("model"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, NeuralNetwork, build, 4))
        .def("build",
             py::overload_cast<const std::shared_ptr<ReplayVideo>&, const NNArchive&, std::optional<float>>(&NeuralNetwork::build),
             py::arg("input"),
             py::arg("nnArchive"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, NeuralNetwork, build, 5))
        .def("setBlob", py::overload_cast<dai::OpenVINO::Blob>(&NeuralNetwork::setBlob), py::arg("blob"), DOC(dai, node, NeuralNetwork, setBlob))
        .def("setBlob", py::overload_cast<const dai::Path&>(&NeuralNetwork::setBlob), py::arg("path"), DOC(dai, node, NeuralNetwork, setBlob, 2))
        .def("setModelPath", &NeuralNetwork::setModelPath, py::arg("modelPath"), DOC(dai, node, NeuralNetwork, setModelPath))
        .def("setNumShavesPerInferenceThread",
             &NeuralNetwork::setNumShavesPerInferenceThread,
             py::arg("numShavesPerInferenceThread"),
             DOC(dai, node, NeuralNetwork, setNumShavesPerInferenceThread))
        .def("setBackend", &NeuralNetwork::setBackend, py::arg("setBackend"), DOC(dai, node, NeuralNetwork, setBackend))
        .def("setBackendProperties", &NeuralNetwork::setBackendProperties, py::arg("setBackendProperties"), DOC(dai, node, NeuralNetwork, setBackendProperties))

        .def_readonly("inputs", &NeuralNetwork::inputs, DOC(dai, node, NeuralNetwork, inputs))
        .def_readonly("passthroughs", &NeuralNetwork::passthroughs, DOC(dai, node, NeuralNetwork, passthroughs))

        ;
    // Properties alias
    daiNodeModule.attr("NeuralNetwork").attr("Properties") = neuralNetworkProperties;
}
