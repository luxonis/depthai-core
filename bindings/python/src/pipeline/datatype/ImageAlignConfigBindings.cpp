#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"
#include <unordered_map>
#include <memory>

// depthai
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

//pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_imagealignconfig(pybind11::module& m, void* pCallstack){

    using namespace dai;

    py::class_<ImageAlignConfig, Py<ImageAlignConfig>, Buffer, std::shared_ptr<ImageAlignConfig>> imageAlignConfig(m, "ImageAlignConfig", DOC(dai, ImageAlignConfig));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Message
    imageAlignConfig
        .def(py::init<>())
        // .def(py::init<std::shared_ptr<ToFConfig>>())
        .def_readwrite("staticDepthPlane", &ImageAlignConfig::staticDepthPlane, DOC(dai, ToFConfig, staticDepthPlane))

        .def("serialize", &ImageAlignConfig::serialize, py::arg("metadata"), py::arg("datatype"), DOC(dai, ToFConfig, serialize))
        ;

    // add aliases
    // m.attr("ToFConfig").attr("DepthParams") = m.attr("ToFConfig").attr("DepthParams");

}
