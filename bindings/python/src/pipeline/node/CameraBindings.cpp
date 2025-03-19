#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

void bind_camera(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Declare node upfront
    auto camera = ADD_NODE(Camera);

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    camera.def_readonly("inputControl", &Camera::inputControl, DOC(dai, node, Camera, inputControl))
        .def_readonly("initialControl", &Camera::initialControl, DOC(dai, node, Camera, initialControl))
        .def_readonly("mockIsp", &Camera::mockIsp, DOC(dai, node, Camera, mockIsp))
        .def_readonly("raw", &Camera::raw, DOC(dai, node, Camera, raw))
        .def("build",
             py::overload_cast<dai::CameraBoardSocket, std::optional<std::pair<uint32_t, uint32_t>>, std::optional<float>>(&Camera::build),
             "boardSocket"_a = CameraBoardSocket::AUTO,
             "sensorResolution"_a = std::nullopt,
             "sensorFps"_a = std::nullopt,
             DOC(dai, node, Camera, build))
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def("build",
             py::overload_cast<dai::CameraBoardSocket, ReplayVideo&>(&Camera::build),
             py::arg("boardSocket"),
             py::arg("replayNode"),
             DOC(dai, node, Camera, build))
        .def("build", py::overload_cast<ReplayVideo&>(&Camera::build), py::arg("replayNode"), DOC(dai, node, Camera, build))
#endif
        // .def("setBoardSocket", &Camera::setBoardSocket, "boardSocket"_a, DOC(dai, node, Camera, setBoardSocket))
        .def("getBoardSocket", &Camera::getBoardSocket, DOC(dai, node, Camera, getBoardSocket))
        .def("setMockIsp", &Camera::setMockIsp, "mockIsp"_a, DOC(dai, node, Camera, setMockIsp))
        // .def("setCamera", &Camera::setCamera, "name"_a, DOC(dai, node, Camera, setCamera))
        // .def("getCamera", &Camera::getCamera, DOC(dai, node, Camera, getCamera))
        .def("requestOutput",
             py::overload_cast<std::pair<uint32_t, uint32_t>, std::optional<ImgFrame::Type>, ImgResizeMode, std::optional<float>, std::optional<bool>>(
                 &Camera::requestOutput),
             "size"_a,
             "type"_a = std::nullopt,
             "resizeMode"_a = dai::ImgResizeMode::CROP,
             "fps"_a = std::nullopt,
             "enableUndistortion"_a = std::nullopt,
             py::return_value_policy::reference_internal,
             DOC(dai, node, Camera, requestOutput))
        .def("requestOutput",
             py::overload_cast<const Capability&, bool>(&Camera::requestOutput),
             "capability"_a,
             "onHost"_a,
             py::return_value_policy::reference_internal,
             DOC(dai, node, Camera, requestOutput, 2))
        .def("requestFullResolutionOutput",
             &Camera::requestFullResolutionOutput,
             "type"_a = std::nullopt,
             "fps"_a = std::nullopt,
             "useHighestResolution"_a = false,
             py::return_value_policy::reference_internal,
             DOC(dai, node, Camera, requestFullResolutionOutput));
}
