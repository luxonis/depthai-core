#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/Iterable.hpp"
#include "pipeline/CommonBindings.hpp"

namespace {

class PyIterableBuffer : public dai::IterableBuffer {
   public:
    using dai::IterableBuffer::IterableBuffer;

    std::size_t getSize() const override {
        PYBIND11_OVERLOAD_PURE(std::size_t, dai::IterableBuffer, getSize);
    }
};

class PyIterableTransformableBuffer : public dai::IterableTransformableBuffer {
   public:
    using dai::IterableTransformableBuffer::IterableTransformableBuffer;

    std::size_t getSize() const override {
        PYBIND11_OVERLOAD_PURE(std::size_t, dai::IterableTransformableBuffer, getSize);
    }

    std::shared_ptr<dai::TransformableBuffer> transformTo(const dai::ImgTransformation& target) const override {
        PYBIND11_OVERLOAD(std::shared_ptr<dai::TransformableBuffer>, dai::IterableTransformableBuffer, transformTo, target);
    }
};

}  // namespace

void bind_iterable(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Iterable, std::shared_ptr<Iterable>> iterable(m, "Iterable");
    py::class_<IterableBuffer, PyIterableBuffer, Buffer, Iterable, std::shared_ptr<IterableBuffer>> iterableBuffer(m, "IterableBuffer");
    py::class_<IterableTransformableBuffer, PyIterableTransformableBuffer, TransformableBuffer, Iterable, std::shared_ptr<IterableTransformableBuffer>>
        iterableTransformableBuffer(m, "IterableTransformableBuffer");

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

    iterable.def("getSize", &Iterable::getSize);

    iterableBuffer.def(py::init<>()).def("getSize", [](const IterableBuffer& self) { return self.getSize(); }).def("__len__", [](const IterableBuffer& self) {
        return self.getSize();
    });

    iterableTransformableBuffer.def(py::init<>())
        .def("getSize", [](const IterableTransformableBuffer& self) { return self.getSize(); })
        .def("__len__", [](const IterableTransformableBuffer& self) { return self.getSize(); });
}
