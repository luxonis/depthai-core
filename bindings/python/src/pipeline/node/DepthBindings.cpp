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

    py::class_<Depth::RequestedOutput>(node, "RequestedOutput")
        .def_property_readonly(
            "depth", [](Depth::RequestedOutput& output) -> Node::Output& { return output.depth.get(); }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "confidence", [](Depth::RequestedOutput& output) -> Node::Output& { return output.confidence.get(); }, py::return_value_policy::reference_internal);

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
        .def("requestOutput",
             static_cast<Depth::RequestedOutput* (Depth::*)(std::optional<float>)>(&Depth::requestOutput),
             py::arg("fps") = std::nullopt,
             py::return_value_policy::reference_internal,
             DOC(dai, node, Depth, depth))
        .def("requestOutput",
             static_cast<Depth::RequestedOutput* (Depth::*)(const std::pair<uint32_t, uint32_t>&, std::optional<float>)>(&Depth::requestOutput),
             py::arg("size"),
             py::arg("fps") = std::nullopt,
             py::return_value_policy::reference_internal,
             DOC(dai, node, Depth, depth))
        .def("build", static_cast<std::shared_ptr<Depth> (Depth::*)()>(&Depth::build), DOC(dai, node, Depth, build))
        .def("build", static_cast<std::shared_ptr<Depth> (Depth::*)(Depth::Algorithm)>(&Depth::build), py::arg("algorithm"), DOC(dai, node, Depth, build))
        .def(
            "build",
            [](Depth& self, Depth::Algorithm algorithm, DeviceModelZoo config) { return self.build(algorithm, config); },
            py::arg("algorithm"),
            py::arg("config"),
            DOC(dai, node, Depth, build, 4))
        .def(
            "build",
            [](Depth& self, Depth::Algorithm algorithm, StereoDepth::PresetMode config) { return self.build(algorithm, config); },
            py::arg("algorithm"),
            py::arg("config"),
            DOC(dai, node, Depth, build, 4))
        .def(
            "build",
            [](Depth& self, Depth::Algorithm algorithm, py::none) { return self.build(algorithm, std::monostate{}); },
            py::arg("algorithm"),
            py::arg("config"),
            DOC(dai, node, Depth, build, 4));
}
