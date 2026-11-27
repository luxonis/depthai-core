#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_imagealignconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ImageAlignConfig, Py<ImageAlignConfig>, Buffer, py::smart_holder> imageAlignConfig(
        m, "ImageAlignConfig", DOC(dai, ImageAlignConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    imageAlignConfig.def(py::init<>())
        .def("__repr__", &ImageAlignConfig::str)
        .def_readwrite("staticDepthPlane", &ImageAlignConfig::staticDepthPlane, DOC(dai, ImageAlignConfig, staticDepthPlane));
}
