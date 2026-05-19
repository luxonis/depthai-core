#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_transformable(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Transformable, Py<Transformable>, std::shared_ptr<Transformable>> transformable(m, "Transformable");
    py::class_<TransformableBuffer, Py<TransformableBuffer>, Buffer, Transformable, std::shared_ptr<TransformableBuffer>> transformableBuffer(
        m, "TransformableBuffer");

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

    transformable.def(py::init<>())
        .def("getTransformation", &Transformable::getTransformation, DOC(dai, ImgFrame, getTransformation))
        .def("setTransformation", &Transformable::setTransformation, py::arg("transformation"), DOC(dai, ImgFrame, setTransformation))
        .def("transformTo", &Transformable::transformTo, py::arg("target"));

    transformableBuffer.def(py::init<>());
}
