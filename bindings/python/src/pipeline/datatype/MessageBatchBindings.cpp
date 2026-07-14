#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/MessageBatch.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_buffervector(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<MessageBatch, Py<MessageBatch>, Buffer, std::shared_ptr<MessageBatch>> bufferVector(m, "MessageBatch");

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    bufferVector.def(py::init<>())
        .def(py::init<std::vector<std::shared_ptr<Buffer>>>(), py::arg("buffers"))
        .def("__repr__", &MessageBatch::str)
        .def("__len__", &MessageBatch::size)
        .def("__getitem__", [](const MessageBatch& msg, std::size_t index) { return msg.at(index); })
        .def(
            "__iter__", [](const MessageBatch& msg) { return py::make_iterator(msg.begin(), msg.end()); }, py::keep_alive<0, 1>())
        .def("push_back", &MessageBatch::push_back, py::arg("buffer"))
        .def(
            "getBuffers", [](const MessageBatch& msg) { return msg.getBuffers(); }, py::return_value_policy::copy)
        .def("setBuffers", py::overload_cast<const std::vector<std::shared_ptr<Buffer>>&>(&MessageBatch::setBuffers), py::arg("buffers"))
        .def("getTimestamp", &MessageBatch::Buffer::getTimestamp)
        .def("getTimestampDevice", &MessageBatch::Buffer::getTimestampDevice)
        .def("getTimestampSystem", &MessageBatch::Buffer::getTimestampSystem)
        .def("getSequenceNum", &MessageBatch::Buffer::getSequenceNum)
        .def("setTimestamp", &MessageBatch::setTimestamp, py::arg("timestamp"))
        .def("setTimestampDevice", &MessageBatch::setTimestampDevice, py::arg("timestampDevice"))
        .def("setTimestampSystem", &MessageBatch::setTimestampSystem, py::arg("timestampSystem"))
        .def("setSequenceNum", &MessageBatch::setSequenceNum, py::arg("sequenceNum"));
}
