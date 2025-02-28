#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_imagealignconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ImageAlignConfig, Py<ImageAlignConfig>, Buffer, std::shared_ptr<ImageAlignConfig>> imageAlignConfig(
        m, "ImageAlignConfig", DOC(dai, ImageAlignConfig));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Message
    imageAlignConfig.def(py::init<>())
        .def("__repr__", &ImageAlignConfig::str)
        .def_readwrite("staticDepthPlane", &ImageAlignConfig::staticDepthPlane, DOC(dai, ImageAlignConfig, staticDepthPlane));
}
