#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/MessageGroup.hpp"

// pybind
#include <pybind11/cast.h>
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_message_group(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<MessageGroup, Py<MessageGroup>, Buffer, std::shared_ptr<MessageGroup>> messageGroup(m, "MessageGroup", DOC(dai, MessageGroup));

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

    // // Metadata / raw
    // rawMessageGroup
    //     .def(py::init<>())
    //     .def_readwrite("group", &RawMessageGroup::group)
    //     ;

    // Message
    messageGroup.def(py::init<>())
        .def("__repr__", &MessageGroup::str)
        .def("__getitem__", [](MessageGroup& msg, const std::string& name) { return msg[name]; })
        .def("__setitem__", [](MessageGroup& msg, const std::string& name, std::shared_ptr<ADatatype> data) { return msg.add(name, data); })
        .def(
            "__iter__",
            [](MessageGroup& msg) { return py::make_iterator(msg.begin(), msg.end()); },
            py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def("isSynced", &MessageGroup::isSynced, py::arg("thresholdNs"), DOC(dai, MessageGroup, isSynced))
        .def("getIntervalNs", &MessageGroup::getIntervalNs, DOC(dai, MessageGroup, getIntervalNs))
        .def("getNumMessages", &MessageGroup::getNumMessages, DOC(dai, MessageGroup, getNumMessages))
        .def("getMessageNames", &MessageGroup::getMessageNames, DOC(dai, MessageGroup, getMessageNames))
        .def("getTimestamp", &MessageGroup::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &MessageGroup::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getTimestampSystem", &MessageGroup::Buffer::getTimestampSystem, DOC(dai, Buffer, getTimestampSystem))
        .def("getSequenceNum", &MessageGroup::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("setTimestamp", &MessageGroup::setTimestamp, py::arg("timestamp"), DOC(dai, Buffer, setTimestamp))
        .def("setTimestampDevice", &MessageGroup::setTimestampDevice, py::arg("timestampDevice"), DOC(dai, Buffer, setTimestampDevice))
        .def("setTimestampSystem", &MessageGroup::setTimestampSystem, py::arg("timestampSystem"), DOC(dai, Buffer, setTimestampSystem))
        .def("setSequenceNum", &MessageGroup::setSequenceNum, py::arg("sequenceNum"), DOC(dai, Buffer, setSequenceNum));
}
