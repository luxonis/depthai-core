#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/Buffer.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_buffer(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Buffer, Py<Buffer>, ADatatype, py::smart_holder> buffer(m, "Buffer", DOC(dai, Buffer));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    buffer.def(py::init<>(), DOC(dai, Buffer, Buffer))
        .def(py::init<size_t>(), DOC(dai, Buffer, Buffer, 2))
        .def("__repr__", &Buffer::str)
        .def(
            "getData",
            [](Buffer& buffer) {
                auto memory = std::make_unique<std::shared_ptr<Memory>>(buffer.data);  // Keeps the allocation alive when setData replaces the buffer storage
                const auto data = (*memory)->getData();
                py::capsule owner(memory.get(), [](void* ptr) { delete static_cast<std::shared_ptr<Memory>*>(ptr); });
                memory.release();
                return py::array_t<uint8_t>(data.size(), data.data(), owner);
            },
            DOC(dai, Buffer, getData))
        .def(
            "setData",
            [](Buffer& buffer, const std::vector<std::uint8_t>& data) { buffer.setData(std::vector<std::uint8_t>(data)); },
            DOC(dai, Buffer, setData))
        .def(
            "setData",
            [](Buffer& buffer, py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast> array) {
                buffer.setData({array.data(), array.data() + array.nbytes()});
            },
            DOC(dai, Buffer, setData))
        .def(
            "setData",
            [](Buffer& buffer, py::bytes data) {
                std::string str = data.cast<std::string>();
                buffer.setData({str.data(), str.data() + str.size()});
            },
            DOC(dai, Buffer, setData))
        .def("getTimestamp", &Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getTimestampSystem", &Buffer::getTimestampSystem, DOC(dai, Buffer, getTimestampSystem))
        .def("getSequenceNum", &Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("setTimestamp", &Buffer::setTimestamp, py::arg("timestamp"), DOC(dai, Buffer, setTimestamp))
        .def("setTimestampDevice", &Buffer::setTimestampDevice, py::arg("timestampDevice"), DOC(dai, Buffer, setTimestampDevice))
        .def("setTimestampSystem", &Buffer::setTimestampSystem, DOC(dai, Buffer, setTimestampSystem))
        .def("setSequenceNum", &Buffer::setSequenceNum, py::arg("sequenceNum"), DOC(dai, Buffer, setSequenceNum))
        .def(
            "setBufferMetadataFrom", py::overload_cast<const std::shared_ptr<Buffer>&>(&Buffer::setBufferMetadataFrom), DOC(dai, Buffer, setBufferMetadataFrom))
        .def("getVisualizationMessage", &Buffer::getVisualizationMessage, DOC(dai, Buffer, getVisualizationMessage));
}
