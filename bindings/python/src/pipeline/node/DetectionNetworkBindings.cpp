#include <pybind11/cast.h>

#include <memory>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/NodeGroup.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/utility/CompilerWarnings.hpp"
void bind_detectionnetwork(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto detectionNetwork = ADD_NODE_DERIVED(DetectionNetwork, DeviceNodeGroup);

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

    // DetectionNetwork Properties
    // detectionNetworkProperties.def_readwrite("parser", &DetectionNetworkProperties::parser);

    // DetectionNetwork Node
    detectionNetwork
#define DETECTION_NETWORK_BUILD_ARGS Node::Output &input, NNArchive &nnArchive
#define DETECTION_NETWORK_BUILD_PYARGS py::arg("input"), py::arg("nnArchive")
#define DETECTION_NETWORK_ARGS float confidenceThreshold
#define DETECTION_NETWORK_PYARGS py::arg("confidenceThreshold") = 0.5
    // TODO (Zimamazim) Automatically fetch default arguments to avoid duplicity
#define DETECTION_NETWORK_CODE(OP) self OP setConfidenceThreshold(confidenceThreshold);
        .def(
            "build",
            [](DetectionNetwork& self, DETECTION_NETWORK_BUILD_ARGS, DETECTION_NETWORK_ARGS) {
                self.build(input, nnArchive);
                DETECTION_NETWORK_CODE(.)
                return std::static_pointer_cast<DetectionNetwork>(self.shared_from_this());
            },
            DETECTION_NETWORK_BUILD_PYARGS,
            DETECTION_NETWORK_PYARGS)
        .def("build",
             py::overload_cast<const std::shared_ptr<Camera>&, NNModelDescription, std::optional<float>>(&DetectionNetwork::build),
             py::arg("input"),
             py::arg("model"),
             py::arg("fps") = std::nullopt)
        .def("build",
             ([](DetectionNetwork& self, const std::shared_ptr<Camera>& input, std::string model, std::optional<float> fps) {
                 return self.build(input, NNModelDescription{model}, fps);
             }),
             py::arg("input"),
             py::arg("model"),
             py::arg("fps") = std::nullopt)
        .def("build",
             py::overload_cast<const std::shared_ptr<Camera>&, const NNArchive&, std::optional<float>>(&DetectionNetwork::build),
             py::arg("input"),
             py::arg("nnArchive"),
             py::arg("fps") = std::nullopt)
        .def("build",
             py::overload_cast<const std::shared_ptr<ReplayVideo>&, NNModelDescription, std::optional<float>>(&DetectionNetwork::build),
             py::arg("input"),
             py::arg("model"),
             py::arg("fps") = std::nullopt)
        .def("build",
             py::overload_cast<const std::shared_ptr<ReplayVideo>&, const NNArchive&, std::optional<float>>(&DetectionNetwork::build),
             py::arg("input"),
             py::arg("nnArchive"),
             py::arg("fps") = std::nullopt)
        .def(py::init([](DETECTION_NETWORK_BUILD_ARGS, DETECTION_NETWORK_ARGS) {
                 auto self = getImplicitPipeline()->create<DetectionNetwork>();
                 self->build(input, nnArchive);
                 DETECTION_NETWORK_CODE(->)
                 return self;
             }),
             DETECTION_NETWORK_BUILD_PYARGS,
             DETECTION_NETWORK_PYARGS)
        // Copied from NN node
        .def("setBlobPath", &DetectionNetwork::setBlobPath, py::arg("path"), DOC(dai, node, DetectionNetwork, setBlobPath))
        .def("setNumPoolFrames", &DetectionNetwork::setNumPoolFrames, py::arg("numFrames"), DOC(dai, node, DetectionNetwork, setNumPoolFrames))
        .def("setNumInferenceThreads",
             &DetectionNetwork::setNumInferenceThreads,
             py::arg("numThreads"),
             DOC(dai, node, DetectionNetwork, setNumInferenceThreads))
        .def("setNumNCEPerInferenceThread",
             &DetectionNetwork::setNumNCEPerInferenceThread,
             py::arg("numNCEPerThread"),
             DOC(dai, node, DetectionNetwork, setNumNCEPerInferenceThread))
        .def("getNumInferenceThreads", &DetectionNetwork::getNumInferenceThreads, DOC(dai, node, DetectionNetwork, getNumInferenceThreads))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&>(&DetectionNetwork::setNNArchive),
             py::arg("archive"),
             DOC(dai, node, DetectionNetwork, setNNArchive))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&, int>(&DetectionNetwork::setNNArchive),
             py::arg("archive"),
             py::arg("numShaves"),
             DOC(dai, node, DetectionNetwork, setNNArchive))
        .def("setFromModelZoo",
             py::overload_cast<NNModelDescription, bool>(&DetectionNetwork::setFromModelZoo),
             py::arg("description"),
             py::arg("useCached") = false,
             DOC(dai, node, DetectionNetwork, setFromModelZoo))
        .def("setBlob", py::overload_cast<dai::OpenVINO::Blob>(&DetectionNetwork::setBlob), py::arg("blob"), DOC(dai, node, DetectionNetwork, setBlob))
        .def("setBlob", py::overload_cast<const dai::Path&>(&DetectionNetwork::setBlob), py::arg("path"), DOC(dai, node, DetectionNetwork, setBlob, 2))
        .def("setModelPath", &DetectionNetwork::setModelPath, py::arg("modelPath"), DOC(dai, node, DetectionNetwork, setModelPath))
        .def("setNumShavesPerInferenceThread",
             &DetectionNetwork::setNumShavesPerInferenceThread,
             py::arg("numShavesPerInferenceThread"),
             DOC(dai, node, DetectionNetwork, setNumShavesPerInferenceThread))
        .def("setBackend", &DetectionNetwork::setBackend, py::arg("setBackend"), DOC(dai, node, DetectionNetwork, setBackend))
        .def("setBackendProperties",
             &DetectionNetwork::setBackendProperties,
             py::arg("setBackendProperties"),
             DOC(dai, node, DetectionNetwork, setBackendProperties))

        // Detection specific properties
        .def_property_readonly(
            "input",
            [](const DetectionNetwork& n) { return &n.neuralNetwork->input; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralNetwork, input))
        .def_property_readonly(
            "out",
            [](const DetectionNetwork& n) { return &n.detectionParser->out; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, DetectionNetwork, out))
        .def_property_readonly(
            "outNetwork",
            [](const DetectionNetwork& n) { return &n.neuralNetwork->out; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, DetectionNetwork, outNetwork))
        .def_property_readonly(
            "passthrough",
            [](const DetectionNetwork& n) { return &n.neuralNetwork->passthrough; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralNetwork, passthrough))
        .def("setConfidenceThreshold", &DetectionNetwork::setConfidenceThreshold, py::arg("thresh"), DOC(dai, node, DetectionNetwork, setConfidenceThreshold))
        .def("getClasses", &DetectionNetwork::getClasses, DOC(dai, node, DetectionNetwork, getClasses))
        .def("getConfidenceThreshold", &DetectionNetwork::getConfidenceThreshold, DOC(dai, node, DetectionNetwork, getConfidenceThreshold));
    // ALIAS
    // daiNodeModule.attr("DetectionNetwork").attr("Properties") = detectionNetworkProperties;
}
