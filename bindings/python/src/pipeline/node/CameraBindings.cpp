#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"

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
#define CAMERA_ARGS CameraBoardSocket boardSocket
#define CAMERA_PYARGS "boardSocket"_a = CameraBoardSocket::AUTO
    // TODO (Zimamazim) Automatically fetch default arguments to avoid duplicity
#define CAMERA_CODE(OP) self OP setBoardSocket(boardSocket);
        .def(
            "build",
            [](Camera& self, CAMERA_ARGS) {
                self.build();
                CAMERA_CODE(.)
                return std::static_pointer_cast<Camera>(self.shared_from_this());
            },
            CAMERA_PYARGS)
        .def(py::init([](CAMERA_ARGS) {
                 auto self = getImplicitPipeline()->create<Camera>();
                 self->build();
                 CAMERA_CODE(->)
                 return self;
             }),
             CAMERA_PYARGS)
        .def("setBoardSocket", &Camera::setBoardSocket, "boardSocket"_a, DOC(dai, node, Camera, setBoardSocket))
        .def("getBoardSocket", &Camera::getBoardSocket, DOC(dai, node, Camera, getBoardSocket))
        .def("setCamera", &Camera::setCamera, "name"_a, DOC(dai, node, Camera, setCamera))
        .def("getCamera", &Camera::getCamera, DOC(dai, node, Camera, getCamera))
        .def("requestOutput",
             py::overload_cast<std::pair<uint32_t, uint32_t>, ImgFrame::Type, ImgResizeMode, uint32_t>(&Camera::requestOutput),
             "size"_a,
             "type"_a = dai::ImgFrame::Type::NV12,
             "resizeMode"_a = dai::ImgResizeMode::CROP,
             "fps"_a = 30,
             py::return_value_policy::reference_internal,
             DOC(dai, node, Camera, requestOutput))
        .def("requestOutput",
             py::overload_cast<const Capability&, bool>(&Camera::requestOutput),
             "capability"_a,
             "onHost"_a,
             py::return_value_policy::reference_internal,
             DOC(dai, node, Camera, requestOutput, 2));
}
