#include <optional>
#include <string>
#include <utility>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Depth.hpp"

void bind_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = addNode<Depth, DeviceNodeGroup>("Depth", DOC(dai, node, Depth));

    // Replace the default factory pushed by addNode with one that forwards `pipeline.create(dai.node.Depth, ...)`
    // keyword arguments to `Depth::build(...)`, so AUTO selection sees the user's FPS / resolution / algorithm
    // before lazy wiring runs. Done **before** recursing into nested binders so .back() is unambiguously the
    // Depth entry. The kwargs lambda runs at user-call time, after all nested binders (including the
    // Depth::Algorithm enum) are registered.
    pyNodeCreateMap.back().second = [](dai::Pipeline& p,
                                       py::object /*class_*/,
                                       const py::args& /*args*/,
                                       const py::kwargs& kwargs) -> std::shared_ptr<dai::Node> {
        auto depth = p.create<Depth>();
        std::optional<float> fps;
        std::optional<std::pair<uint32_t, uint32_t>> resolution;
        std::optional<Depth::Algorithm> algorithm;
        for(auto item : kwargs) {
            const std::string key = py::cast<std::string>(item.first);
            if(key == "fps" || key == "FPS") {
                if(!item.second.is_none()) fps = item.second.cast<float>();
            } else if(key == "resolution" || key == "stereo_size") {
                if(!item.second.is_none()) resolution = item.second.cast<std::pair<uint32_t, uint32_t>>();
            } else if(key == "algorithm") {
                if(!item.second.is_none()) algorithm = item.second.cast<Depth::Algorithm>();
            } else {
                throw py::type_error("pipeline.create(dai.node.Depth, ...) got unexpected keyword argument '" + key
                                     + "' (supported: fps/FPS, resolution/stereo_size, algorithm)");
            }
        }
        if(algorithm.has_value() || resolution.has_value()) {
            depth->build(algorithm.value_or(Depth::Algorithm::AUTO), fps, resolution);
        } else if(fps.has_value()) {
            depth->build(fps);
        }
        return depth;
    };

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

    // Translate a Depth::Config variant to a Python value (model, preset, or None).
    auto configToPy = [](const Depth::Config& c) -> py::object {
        if(const auto* m = std::get_if<DeviceModelZoo>(&c)) return py::cast(*m);
        if(const auto* p = std::get_if<StereoDepth::PresetMode>(&c)) return py::cast(*p);
        return py::none();
    };

    node.def("getAlgorithm", &Depth::getAlgorithm)
        .def("getResolvedAlgorithm", &Depth::getResolvedAlgorithm)
        .def("getResolvedPreset", [configToPy](const Depth& d) { return configToPy(d.getResolvedPreset()); })
        .def_static("exceedsStereoDepthMaxResolution", &Depth::exceedsStereoDepthMaxResolution)
        .def_static(
            "selectBackend",
            [configToPy](py::object resolution,
                         float targetFps,
                         const std::vector<Depth::Algorithm>& supportedAlgorithms,
                         const std::vector<DeviceModelZoo>& supportedModels) {
                std::optional<std::pair<uint32_t, uint32_t>> optRes;
                if(!resolution.is_none()) {
                    optRes = resolution.cast<std::pair<uint32_t, uint32_t>>();
                }
                const auto sel = Depth::selectBackend(optRes, targetFps, supportedAlgorithms, supportedModels);
                return py::make_tuple(sel.algorithm, configToPy(sel.config));
            },
            py::arg("resolution") = py::none(),
            py::arg("target_fps") = 30.f,
            py::arg("supported_algorithms") = std::vector<Depth::Algorithm>{Depth::Algorithm::NEURAL,
                                                                             Depth::Algorithm::NEURAL_ASSISTED_STEREO,
                                                                             Depth::Algorithm::STEREO,
                                                                             Depth::Algorithm::GPU_STEREO},
            py::arg("supported_models") = std::vector<DeviceModelZoo>{})
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
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal);
}
