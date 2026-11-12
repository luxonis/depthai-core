#include "depthai_pybind11_tests.hpp"

#include <pybind11/stl.h>

#include <stdexcept>

#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

namespace {
// Only native ownership: Python references or keep_alive here would mask
// missing Python-instance retention in the production message bindings.
struct MessageOwner {
    std::shared_ptr<dai::ADatatype> message;
    explicit MessageOwner(std::shared_ptr<dai::ADatatype> value) : message(std::move(value)) {}
};
}  // namespace

TEST_SUBMODULE(message_holder, m) {
    py::module_::import("depthai");
    py::class_<MessageOwner>(m, "MessageOwner")
        .def(py::init<std::shared_ptr<dai::ADatatype>>())
        .def("get", [](const MessageOwner& owner) { return owner.message; })
        .def("visualize", [](const MessageOwner& owner) {
            auto buffer = std::dynamic_pointer_cast<dai::Buffer>(owner.message);
            if(!buffer) throw std::invalid_argument("Expected a Buffer");
            buffer->getVisualizationMessage();
        })
        .def("transform", [](const MessageOwner& owner, const dai::ImgTransformation& target) {
            auto buffer = std::dynamic_pointer_cast<dai::TransformableBuffer>(owner.message);
            if(!buffer) throw std::invalid_argument("Expected a TransformableBuffer");
            return buffer->transformTo(target);
        });
}
