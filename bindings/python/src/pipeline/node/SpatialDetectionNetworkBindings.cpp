#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

void bind_spatialdetectionnetwork(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<SpatialDetectionNetworkProperties, std::shared_ptr<SpatialDetectionNetworkProperties>> spatialDetectionNetworkProperties(
        m, "SpatialDetectionNetworkProperties", DOC(dai, SpatialDetectionNetworkProperties));
    auto spatialDetectionNetwork = ADD_NODE_DERIVED(SpatialDetectionNetwork, DeviceNode);

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
    spatialDetectionNetworkProperties.def_readwrite("detectedBBScaleFactor", &SpatialDetectionNetworkProperties::detectedBBScaleFactor)
        .def_readwrite("depthThresholds", &SpatialDetectionNetworkProperties::depthThresholds);

    // Node
    spatialDetectionNetwork
        // Copied from NN node
        .def("build",
             py::overload_cast<const std::shared_ptr<Camera>&, const std::shared_ptr<StereoDepth>&, NNModelDescription, std::optional<float>>(
                 &SpatialDetectionNetwork::build),
             py::arg("input"),
             py::arg("stereo"),
             py::arg("model"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, SpatialDetectionNetwork, build))
        .def("build",
             ([](SpatialDetectionNetwork& self,
                 const std::shared_ptr<Camera>& input,
                 const std::shared_ptr<StereoDepth>& stereo,
                 std::string model,
                 std::optional<float> fps) { return self.build(input, stereo, NNModelDescription{model}, fps); }),
             py::arg("input"),
             py::arg("stereo"),
             py::arg("model"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, SpatialDetectionNetwork, build, 2))
        .def("build",
             py::overload_cast<const std::shared_ptr<Camera>&, const std::shared_ptr<StereoDepth>&, const NNArchive&, std::optional<float>>(
                 &SpatialDetectionNetwork::build),
             py::arg("input"),
             py::arg("stereo"),
             py::arg("nnArchive"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, SpatialDetectionNetwork, build, 2))
        .def("setBlobPath", &SpatialDetectionNetwork::setBlobPath, py::arg("path"), DOC(dai, node, SpatialDetectionNetwork, setBlobPath))
        .def("setNumPoolFrames", &SpatialDetectionNetwork::setNumPoolFrames, py::arg("numFrames"), DOC(dai, node, SpatialDetectionNetwork, setNumPoolFrames))
        .def("setNumInferenceThreads",
             &SpatialDetectionNetwork::setNumInferenceThreads,
             py::arg("numThreads"),
             DOC(dai, node, SpatialDetectionNetwork, setNumInferenceThreads))
        .def("setNumNCEPerInferenceThread",
             &SpatialDetectionNetwork::setNumNCEPerInferenceThread,
             py::arg("numNCEPerThread"),
             DOC(dai, node, SpatialDetectionNetwork, setNumNCEPerInferenceThread))
        .def("getNumInferenceThreads", &SpatialDetectionNetwork::getNumInferenceThreads, DOC(dai, node, SpatialDetectionNetwork, getNumInferenceThreads))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&>(&SpatialDetectionNetwork::setNNArchive),
             py::arg("archive"),
             DOC(dai, node, SpatialDetectionNetwork, setNNArchive))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&, int>(&SpatialDetectionNetwork::setNNArchive),
             py::arg("archive"),
             py::arg("numShaves"),
             DOC(dai, node, SpatialDetectionNetwork, setNNArchive))
        .def("setFromModelZoo",
             py::overload_cast<NNModelDescription, bool>(&SpatialDetectionNetwork::setFromModelZoo),
             py::arg("description"),
             py::arg("useCached"),
             DOC(dai, node, SpatialDetectionNetwork, setFromModelZoo))
        .def("setBlob",
             py::overload_cast<dai::OpenVINO::Blob>(&SpatialDetectionNetwork::setBlob),
             py::arg("blob"),
             DOC(dai, node, SpatialDetectionNetwork, setBlob))
        .def("setBlob",
             py::overload_cast<const dai::Path&>(&SpatialDetectionNetwork::setBlob),
             py::arg("path"),
             DOC(dai, node, SpatialDetectionNetwork, setBlob, 2))
        .def("setModelPath", &SpatialDetectionNetwork::setModelPath, py::arg("modelPath"), DOC(dai, node, SpatialDetectionNetwork, setModelPath))
        .def("setNumShavesPerInferenceThread",
             &SpatialDetectionNetwork::setNumShavesPerInferenceThread,
             py::arg("numShavesPerInferenceThread"),
             DOC(dai, node, SpatialDetectionNetwork, setNumShavesPerInferenceThread))
        .def("setBackend", &SpatialDetectionNetwork::setBackend, py::arg("setBackend"), DOC(dai, node, SpatialDetectionNetwork, setBackend))
        .def("setBackendProperties",
             &SpatialDetectionNetwork::setBackendProperties,
             py::arg("setBackendProperties"),
             DOC(dai, node, SpatialDetectionNetwork, setBackendProperties))

        // Detection parser
        .def("setConfidenceThreshold",
             &SpatialDetectionNetwork::setConfidenceThreshold,
             py::arg("thresh"),
             DOC(dai, node, SpatialDetectionNetwork, setConfidenceThreshold))
        .def("getConfidenceThreshold", &SpatialDetectionNetwork::getConfidenceThreshold, DOC(dai, node, SpatialDetectionNetwork, getConfidenceThreshold))
        .def("getClasses", &SpatialDetectionNetwork::getClasses, DOC(dai, node, SpatialDetectionNetwork, getClasses))
        .def_property_readonly(
            "input",
            [](const SpatialDetectionNetwork& n) { return &n.neuralNetwork->input; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralNetwork, input))
        .def_property_readonly(
            "out",
            [](const SpatialDetectionNetwork& n) { return &n.detectionParser->out; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, SpatialDetectionNetwork, out))
        .def_property_readonly(
            "outNetwork",
            [](const SpatialDetectionNetwork& n) { return &n.neuralNetwork->out; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, SpatialDetectionNetwork, outNetwork))
        .def_property_readonly(
            "passthrough",
            [](const SpatialDetectionNetwork& n) { return &n.neuralNetwork->passthrough; },
            py::return_value_policy::reference_internal,
            DOC(dai, node, NeuralNetwork, passthrough))

        .def_readonly("inputDepth", &SpatialDetectionNetwork::inputDepth, DOC(dai, node, SpatialDetectionNetwork, inputDepth))
        .def_readonly("out", &SpatialDetectionNetwork::out, DOC(dai, node, SpatialDetectionNetwork, out))
        .def_readonly("boundingBoxMapping", &SpatialDetectionNetwork::boundingBoxMapping, DOC(dai, node, SpatialDetectionNetwork, boundingBoxMapping))
        .def_readonly("passthroughDepth", &SpatialDetectionNetwork::passthroughDepth, DOC(dai, node, SpatialDetectionNetwork, passthroughDepth))
        .def_readonly("spatialLocationCalculatorOutput",
                      &SpatialDetectionNetwork::spatialLocationCalculatorOutput,
                      DOC(dai, node, SpatialDetectionNetwork, spatialLocationCalculatorOutput))

        .def("setDepthLowerThreshold",
             &SpatialDetectionNetwork::setDepthLowerThreshold,
             py::arg("lowerThreshold"),
             DOC(dai, node, SpatialDetectionNetwork, setDepthLowerThreshold))
        .def("setBoundingBoxScaleFactor",
             &SpatialDetectionNetwork::setBoundingBoxScaleFactor,
             py::arg("scaleFactor"),
             DOC(dai, node, SpatialDetectionNetwork, setBoundingBoxScaleFactor))
        .def("setDepthUpperThreshold",
             &SpatialDetectionNetwork::setDepthUpperThreshold,
             py::arg("upperThreshold"),
             DOC(dai, node, SpatialDetectionNetwork, setDepthUpperThreshold))
        .def("setSpatialCalculationAlgorithm",
             &SpatialDetectionNetwork::setSpatialCalculationAlgorithm,
             py::arg("calculationAlgorithm"),
             DOC(dai, node, SpatialDetectionNetwork, setSpatialCalculationAlgorithm));
    // ALIAS
    daiNodeModule.attr("SpatialDetectionNetwork").attr("Properties") = spatialDetectionNetworkProperties;
}
