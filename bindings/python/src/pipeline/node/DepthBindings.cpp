#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Depth.hpp"

void bind_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = addNode<Depth, DeviceNodeGroup>("Depth", DOC(dai, node, Depth));

    // Run nested binders first so Depth::Algorithm is registered before any method uses it as a default type.
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    py::enum_<Depth::Algorithm> depthAlgorithm(node, "Algorithm");
    depthAlgorithm.value("AUTO", Depth::Algorithm::AUTO)
        .value("STEREO", Depth::Algorithm::STEREO)
        .value("NEURAL", Depth::Algorithm::NEURAL)
        .value("NEURAL_ASSISTED_STEREO", Depth::Algorithm::NEURAL_ASSISTED_STEREO)
        .value("TOF", Depth::Algorithm::TOF)
        .value("GPU_STEREO", Depth::Algorithm::GPU_STEREO);

    node.def("getAlgorithm", &Depth::getAlgorithm)
        .def("getResolvedAlgorithm", &Depth::getResolvedAlgorithm)
        .def("getResolvedNeuralModel", &Depth::getResolvedNeuralModel)
        .def("getResolvedStereoPreset", &Depth::getResolvedStereoPreset)
        .def_static("exceedsStereoDepthMaxResolution", &Depth::exceedsStereoDepthMaxResolution)
        .def_static("selectNeuralDepthModel", &Depth::selectNeuralDepthModel, py::arg("user_width"), py::arg("user_height"),
                    py::arg("target_fps"), py::arg("supported_models") = std::vector<DeviceModelZoo>{})
        .def_static("selectStereoDepthPreset", &Depth::selectStereoDepthPreset, py::arg("target_fps"))
        .def(
            "build",
            [](Depth& self, py::object fps) {
                std::optional<float> optFps;
                if(!fps.is_none()) {
                    optFps = fps.cast<float>();
                }
                return self.build(optFps);
            },
            py::arg("fps") = py::none())
        .def(
            "build",
            [](Depth& self, Depth::Algorithm algorithm, py::object fps, py::object stereoSize) {
                std::optional<float> optFps;
                if(!fps.is_none()) {
                    optFps = fps.cast<float>();
                }
                std::optional<std::pair<uint32_t, uint32_t>> optSize;
                if(!stereoSize.is_none()) {
                    const auto size = stereoSize.cast<std::pair<uint32_t, uint32_t>>();
                    optSize = size;
                }
                return self.build(algorithm, optFps, optSize);
            },
            py::arg("algorithm"),
            py::arg("fps") = py::none(),
            py::arg("stereo_size") = py::none())
        .def("build", py::overload_cast<DeviceModelZoo>(&Depth::build), py::arg("neural_model"))
        .def_property_readonly("depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal)
        .def("hasConfidence", &Depth::hasConfidence);
}
