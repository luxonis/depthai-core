#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNodeGroup.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"

void bind_depth(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    auto node = addNode<Depth, DeviceNodeGroup>("Depth", DOC(dai, node, Depth));
    // pipeline.create(Depth, algorithm=...) / positional Algorithm; default addNode factory ignores kwargs.
    pyNodeCreateMap.back().second = [](dai::Pipeline& p, py::object /*class_*/, const py::args& args, const py::kwargs& kwargs) -> std::shared_ptr<dai::Node> {
        py::object algKw = kwargs.attr("get")("algorithm", py::none());
        if(!algKw.is_none()) {
            return p.create<Depth>(algKw.cast<Depth::Algorithm>());
        }
        if(args.size() >= 1) {
            return p.create<Depth>(args[0].cast<Depth::Algorithm>());
        }
        return p.create<Depth>();
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

    // py::none() default + cast inside lambda avoids pybind11 converting a default before DeviceModelZoo is ready.
    node.def(
            "build",
            [](Depth& self, py::object neuralModel) {
                DeviceModelZoo model = DeviceModelZoo::NEURAL_DEPTH_SMALL;
                if(!neuralModel.is_none()) {
                    model = neuralModel.cast<DeviceModelZoo>();
                }
                return self.build(model);
            },
            py::arg("neuralModel") = py::none())
        .def("getAlgorithm", &Depth::getAlgorithm)
        .def("setAlgorithm", &Depth::setAlgorithm, py::arg("algorithm"))
        .def("setNeuralAssistedStereoModel", &Depth::setNeuralAssistedStereoModel, py::arg("model"))
        .def("setNeuralAssistedStereoRectify", &Depth::setNeuralAssistedStereoRectify, py::arg("rectify"))
        // Same pattern as build: avoid C++ enum defaults on py::arg until types are registered.
        .def(
            "setTofOptions",
            [](Depth& d, py::object boardSocket, py::object presetMode, py::object fps) {
                CameraBoardSocket socket = CameraBoardSocket::AUTO;
                if(!boardSocket.is_none()) {
                    socket = boardSocket.cast<CameraBoardSocket>();
                }
                ImageFiltersPresetMode preset = ImageFiltersPresetMode::TOF_MID_RANGE;
                if(!presetMode.is_none()) {
                    preset = presetMode.cast<ImageFiltersPresetMode>();
                }
                std::optional<float> optFps;
                if(!fps.is_none()) {
                    optFps = fps.cast<float>();
                }
                return d.setTofOptions(socket, preset, std::move(optFps));
            },
            py::arg("board_socket") = py::none(),
            py::arg("preset_mode") = py::none(),
            py::arg("fps") = py::none())
        .def_property_readonly("depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal)
        .def("getStereoDepth", &Depth::getStereoDepth, py::return_value_policy::reference_internal)
        .def("getNeuralDepth", &Depth::getNeuralDepth, py::return_value_policy::reference_internal)
        .def("getNeuralAssistedStereo", &Depth::getNeuralAssistedStereo, py::return_value_policy::reference_internal)
        .def("getToF", &Depth::getToF, py::return_value_policy::reference_internal)
        .def("getGPUStereo", &Depth::getGPUStereo, py::return_value_policy::reference_internal);
}
