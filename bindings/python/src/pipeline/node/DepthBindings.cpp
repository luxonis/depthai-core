#include <optional>
#include <type_traits>
#include <utility>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Depth.hpp"

void bind_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = addNode<Depth, DeviceNodeGroup>("Depth", DOC(dai, node, Depth));

    auto configToPy = [](const Depth::Config& config) -> py::object {
        return std::visit(
            [](const auto& value) -> py::object {
                using T = std::decay_t<decltype(value)>;
                if constexpr(std::is_same_v<T, std::monostate>) {
                    return py::none();
                } else {
                    return py::cast(value);
                }
            },
            config);
    };

    // Run nested binders first so Depth::Algorithm is registered before any method uses it as a default type.
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    py::enum_<Depth::Algorithm> depthAlgorithm(node, "Algorithm", DOC(dai, node, Depth, Algorithm));
    depthAlgorithm.value("AUTO", Depth::Algorithm::AUTO)
        .value("STEREO", Depth::Algorithm::STEREO)
        .value("NEURAL", Depth::Algorithm::NEURAL)
        .value("NEURAL_ASSISTED_STEREO", Depth::Algorithm::NEURAL_ASSISTED_STEREO)
        .value("TOF", Depth::Algorithm::TOF)
        .value("GPU_STEREO", Depth::Algorithm::GPU_STEREO);

    node.def("getRequestedAlgorithm", &Depth::getRequestedAlgorithm, DOC(dai, node, Depth, getRequestedAlgorithm))
        .def("setAlgorithm", &Depth::setAlgorithm, py::arg("algorithm"), DOC(dai, node, Depth, setAlgorithm))
        .def(
            "getRequestedConfig",
            [configToPy](const Depth& d) -> py::object { return d.getRequestedConfig().has_value() ? configToPy(*d.getRequestedConfig()) : py::none(); },
            DOC(dai, node, Depth, getRequestedConfig))
        .def(
            "setConfig", [](Depth& self, DeviceModelZoo config) { return self.setConfig(config); }, py::arg("config"), DOC(dai, node, Depth, setConfig))
        .def(
            "setConfig",
            [](Depth& self, StereoDepth::PresetMode config) { return self.setConfig(config); },
            py::arg("config"),
            DOC(dai, node, Depth, setConfig))
        .def(
            "setConfig", [](Depth& self, py::none) { return self.setConfig(std::monostate{}); }, py::arg("config"), DOC(dai, node, Depth, setConfig))
        .def("getResolvedAlgorithm", &Depth::getResolvedAlgorithm, DOC(dai, node, Depth, getResolvedAlgorithm))
        .def(
            "getResolvedConfig", [configToPy](const Depth& d) { return configToPy(d.getResolvedConfig()); }, DOC(dai, node, Depth, getResolvedConfig))
        .def("setAlignTo", &Depth::setAlignTo, py::arg("alignTo"), DOC(dai, node, Depth, setAlignTo))
        .def("build",
             static_cast<std::shared_ptr<Depth> (Depth::*)(std::optional<float>)>(&Depth::build),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, Depth, build))
        .def("build",
             static_cast<std::shared_ptr<Depth> (Depth::*)(Depth::Algorithm, std::optional<float>)>(&Depth::build),
             py::arg("algorithm"),
             py::arg("fps") = std::nullopt,
             DOC(dai, node, Depth, build))
        .def(
            "build",
            static_cast<std::shared_ptr<Depth> (Depth::*)(Depth::Algorithm, std::optional<float>, std::optional<std::pair<uint32_t, uint32_t>>)>(&Depth::build),
            py::arg("algorithm"),
            py::arg("fps") = std::nullopt,
            py::arg("stereo_size") = std::nullopt,
            DOC(dai, node, Depth, build, 3))
        .def(
            "build",
            [](Depth& self,
               Depth::Algorithm algorithm,
               DeviceModelZoo config,
               std::optional<float> fps,
               std::optional<std::pair<uint32_t, uint32_t>> stereoSize) { return self.build(algorithm, config, fps, stereoSize); },
            py::arg("algorithm"),
            py::arg("config"),
            py::arg("fps") = std::nullopt,
            py::arg("stereo_size") = std::nullopt,
            DOC(dai, node, Depth, build, 4))
        .def(
            "build",
            [](Depth& self,
               Depth::Algorithm algorithm,
               StereoDepth::PresetMode config,
               std::optional<float> fps,
               std::optional<std::pair<uint32_t, uint32_t>> stereoSize) { return self.build(algorithm, config, fps, stereoSize); },
            py::arg("algorithm"),
            py::arg("config"),
            py::arg("fps") = std::nullopt,
            py::arg("stereo_size") = std::nullopt,
            DOC(dai, node, Depth, build, 4))
        .def(
            "build",
            [](Depth& self, Depth::Algorithm algorithm, py::none, std::optional<float> fps, std::optional<std::pair<uint32_t, uint32_t>> stereoSize) {
                return self.build(algorithm, std::monostate{}, fps, stereoSize);
            },
            py::arg("algorithm"),
            py::arg("config"),
            py::arg("fps") = std::nullopt,
            py::arg("stereo_size") = std::nullopt,
            DOC(dai, node, Depth, build, 4))
        .def_property_readonly(
            "depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal, DOC(dai, node, Depth, depth))
        .def_property_readonly(
            "confidence",
            [](Depth& d) -> Node::Output& { return d.confidence(); },
            py::return_value_policy::reference_internal,
            DOC(dai, node, Depth, confidence));
}
