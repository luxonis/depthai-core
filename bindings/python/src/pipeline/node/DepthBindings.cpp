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

    auto configToPy = [](const Depth::Config& c) -> py::object {
        if(const auto* m = std::get_if<DeviceModelZoo>(&c)) return py::cast(*m);
        if(const auto* p = std::get_if<StereoDepth::PresetMode>(&c)) return py::cast(*p);
        return py::none();
    };

    auto configFromPy = [](py::handle value) -> Depth::Config {
        if(value.is_none()) {
            return std::monostate{};
        }
        try {
            return value.cast<DeviceModelZoo>();
        } catch(const py::cast_error&) {
        }
        try {
            return value.cast<StereoDepth::PresetMode>();
        } catch(const py::cast_error&) {
        }
        throw py::type_error("Depth config must be None, dai.DeviceModelZoo, or dai.node.StereoDepth.PresetMode");
    };

    // Replace the default factory pushed by addNode with one that forwards `pipeline.create(dai.node.Depth, ...)`
    // keyword arguments to `Depth::build(...)`, so AUTO selection sees the user's FPS / resolution / algorithm
    // before lazy wiring runs. Done **before** recursing into nested binders so .back() is unambiguously the
    // Depth entry. The kwargs lambda runs at user-call time, after all nested binders (including the
    // Depth::Algorithm enum) are registered.
    pyNodeCreateMap.back().second = [configFromPy](dai::Pipeline& p,
                                                   py::object /*class_*/,
                                                   const py::args& /*args*/,
                                                   const py::kwargs& kwargs) -> std::shared_ptr<dai::Node> {
        auto depth = p.create<Depth>();
        std::optional<float> fps;
        std::optional<std::pair<uint32_t, uint32_t>> resolution;
        std::optional<Depth::Algorithm> algorithm;
        std::optional<Depth::Config> config;
        for(auto item : kwargs) {
            const std::string key = py::cast<std::string>(item.first);
            if(key == "fps" || key == "FPS") {
                if(!item.second.is_none()) fps = item.second.cast<float>();
            } else if(key == "resolution" || key == "stereo_size") {
                if(!item.second.is_none()) resolution = item.second.cast<std::pair<uint32_t, uint32_t>>();
            } else if(key == "algorithm") {
                if(!item.second.is_none()) algorithm = item.second.cast<Depth::Algorithm>();
            } else if(key == "config") {
                config = configFromPy(item.second);
            } else {
                throw py::type_error("pipeline.create(dai.node.Depth, ...) got unexpected keyword argument '" + key
                                     + "' (supported: fps/FPS, resolution/stereo_size, algorithm, config)");
            }
        }
        if(config.has_value()) {
            if(!algorithm.has_value()) {
                throw py::type_error("pipeline.create(dai.node.Depth, config=...) also requires an explicit algorithm");
            }
            depth->build(*algorithm, *config, fps, resolution);
        } else if(algorithm.has_value() || resolution.has_value()) {
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

    node.def("getRequestedAlgorithm", &Depth::getRequestedAlgorithm)
        .def("setAlgorithm", &Depth::setAlgorithm, py::arg("algorithm"))
        .def("getRequestedConfig", [configToPy](const Depth& d) -> py::object {
            return d.getRequestedConfig().has_value() ? configToPy(*d.getRequestedConfig()) : py::none();
        })
        .def("setConfig", [configFromPy](Depth& self, py::object config) { return self.setConfig(configFromPy(config)); }, py::arg("config"))
        .def("getResolvedAlgorithm", &Depth::getResolvedAlgorithm)
        .def("getResolvedConfig", [configToPy](const Depth& d) { return configToPy(d.getResolvedConfig()); })
        .def(
            "build",
            [](Depth& self, py::object value) {
                if(value.is_none()) {
                    return self.build(std::optional<float>{});
                }
                try {
                    return self.build(value.cast<Depth::Algorithm>());
                } catch(const py::cast_error&) {
                }
                try {
                    return self.build(value.cast<DeviceModelZoo>());
                } catch(const py::cast_error&) {
                }
                return self.build(value.cast<float>());
            },
            py::arg("fps") = py::none())
        // Single algorithm-leading overload: `config=None` auto-picks the profile; passing an explicit
        // `config` pins the (algorithm, config) pair. Merged into one binding so the second positional
        // argument is unambiguously the config (both fps and config would otherwise be py::object).
        .def(
            "build",
            [configFromPy](Depth& self, Depth::Algorithm algorithm, py::object config, py::object fps, py::object stereoSize) {
                std::optional<float> optFps;
                if(!fps.is_none()) {
                    optFps = fps.cast<float>();
                }
                std::optional<std::pair<uint32_t, uint32_t>> optSize;
                if(!stereoSize.is_none()) {
                    optSize = stereoSize.cast<std::pair<uint32_t, uint32_t>>();
                }
                if(config.is_none()) {
                    return self.build(algorithm, optFps, optSize);
                }
                return self.build(algorithm, configFromPy(config), optFps, optSize);
            },
            py::arg("algorithm"),
            py::arg("config") = py::none(),
            py::arg("fps") = py::none(),
            py::arg("stereo_size") = py::none())
        .def("build", py::overload_cast<DeviceModelZoo>(&Depth::build), py::arg("neural_model"))
        .def_property_readonly("depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal);
}
