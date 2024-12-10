#include "ImgFrameCapabilityBindings.hpp"

// depthai
#include "depthai/capabilities/Capability.hpp"
#include "depthai/capabilities/ImgFrameCapability.hpp"

void ImgFrameCapabilityBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ImgFrameCapability, Capability> imgFrameCapability(m, "ImgFrameCapability", DOC(dai, ImgFrameCapability));
    py::enum_<ImgResizeMode> imgResizeMode(m, "ImgResizeMode", DOC(dai, ImgResizeMode));

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings

    // ImgResizeMode
    imgResizeMode.value("CROP", ImgResizeMode::CROP).value("STRETCH", ImgResizeMode::STRETCH).value("LETTERBOX", ImgResizeMode::LETTERBOX);

    // ImgFrameCapability
    imgFrameCapability.def(py::init<>())
        .def_readwrite("size", &ImgFrameCapability::size)
        .def_readwrite("fps", &ImgFrameCapability::fps)
        .def_readwrite("type", &ImgFrameCapability::type)
        .def_readwrite("resizeMode", &ImgFrameCapability::resizeMode)
        .def_readwrite("enableUndistortion", &ImgFrameCapability::enableUndistortion)

        ;
}
