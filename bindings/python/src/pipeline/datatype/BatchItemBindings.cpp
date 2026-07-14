#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/BatchItem.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_iterable(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<BatchItem, Py<BatchItem>, Buffer, std::shared_ptr<BatchItem>> iterable(m, "BatchItem");

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    iterable.def(py::init<>())
        .def(py::init<std::shared_ptr<ADatatype>, std::uint32_t, std::uint32_t, std::uint32_t>(),
             py::arg("payload"),
             py::arg("batchSize"),
             py::arg("batchIndex"),
             py::arg("itemIndex"))
        .def_readwrite("batchSize", &BatchItem::batchSize)
        .def_readwrite("batchIndex", &BatchItem::batchIndex)
        .def_readwrite("itemIndex", &BatchItem::itemIndex)
        .def("setPayload", &BatchItem::setPayload, py::arg("payload"))
        .def("getPayload", static_cast<std::shared_ptr<ADatatype> (BatchItem::*)() const>(&BatchItem::getPayload))
        .def("getSequenceNum", &BatchItem::Buffer::getSequenceNum)
        .def("setSequenceNum", &BatchItem::setSequenceNum, py::arg("sequenceNum"))
        .def("getTimestamp", &BatchItem::Buffer::getTimestamp)
        .def("setTimestamp", &BatchItem::setTimestamp, py::arg("timestamp"));
}
