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

    auto node = ADD_NODE_DERIVED(Depth, DeviceNodeGroup);

    py::enum_<Depth::Algorithm> depthAlgorithm(node, "Algorithm");
    depthAlgorithm.value("AUTO", Depth::Algorithm::AUTO)
        .value("STEREO", Depth::Algorithm::STEREO)
        .value("NEURAL", Depth::Algorithm::NEURAL)
        .value("NEURAL_ASSISTED_STEREO", Depth::Algorithm::NEURAL_ASSISTED_STEREO)
        .value("TOF", Depth::Algorithm::TOF)
        .value("GPU_STEREO", Depth::Algorithm::GPU_STEREO);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    node.def("build", py::overload_cast<DeviceModelZoo>(&Depth::build), py::arg("neuralModel") = DeviceModelZoo::NEURAL_DEPTH_SMALL)
        .def("setAlgorithm", &Depth::setAlgorithm, py::arg("algorithm"))
        .def("setNeuralAssistedStereoModel", &Depth::setNeuralAssistedStereoModel, py::arg("model"))
        .def("setNeuralAssistedStereoRectify", &Depth::setNeuralAssistedStereoRectify, py::arg("rectify"))
        .def(
            "setTofOptions",
            [](Depth& d, CameraBoardSocket boardSocket, ImageFiltersPresetMode presetMode, py::object fps) {
                std::optional<float> optFps;
                if(!fps.is_none()) {
                    optFps = fps.cast<float>();
                }
                return d.setTofOptions(boardSocket, presetMode, std::move(optFps));
            },
            py::arg("board_socket") = CameraBoardSocket::AUTO,
            py::arg("preset_mode") = ImageFiltersPresetMode::TOF_MID_RANGE,
            py::arg("fps") = py::none())
        .def_property_readonly("depth", [](Depth& d) -> Node::Output& { return d.depth(); }, py::return_value_policy::reference_internal)
        .def_property_readonly("confidence", [](Depth& d) -> Node::Output& { return d.confidence(); }, py::return_value_policy::reference_internal)
        .def("getStereoDepth", &Depth::getStereoDepth, py::return_value_policy::reference_internal)
        .def("getNeuralDepth", &Depth::getNeuralDepth, py::return_value_policy::reference_internal)
        .def("getNeuralAssistedStereo", &Depth::getNeuralAssistedStereo, py::return_value_policy::reference_internal)
        .def("getToF", &Depth::getToF, py::return_value_policy::reference_internal)
        .def("getGPUStereo", &Depth::getGPUStereo, py::return_value_policy::reference_internal);
}
