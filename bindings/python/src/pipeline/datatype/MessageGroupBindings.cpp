#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "pipeline/CommonBindings.hpp"

#include <pybind11/cast.h>

void bind_message_group(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Link> link(m, "Link");
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

    link.def(py::init<>())
        .def_readwrite("parentMessageIndex", &Link::parentMessageIndex)
        .def_readwrite("itemIndex", &Link::itemIndex)
        .def_readwrite("childNodeIndex", &Link::childNodeIndex);

    messageGroup.def(py::init<>())
        .def("__repr__", &MessageGroup::str)
        .def("__getitem__", [](MessageGroup& msg, const std::string& name) { return msg[name]; })
        .def("__getitem__", [](MessageGroup& msg, uint32_t index) { return msg[index]; })
        .def("__setitem__", [](MessageGroup& msg, const std::string& name, std::shared_ptr<ADatatype> data) { return msg.addMessage(name, data); })
        .def("__setitem__", [](MessageGroup& msg, uint32_t index, std::shared_ptr<ADatatype> data) { return msg.addMessage(index, data); })
        .def("__contains__", [](MessageGroup& msg, const std::string& name) { return msg.get(name) != nullptr; })
        .def("__contains__", [](MessageGroup& msg, uint32_t index) { return msg.get(index) != nullptr; })
        .def("__len__", [](const MessageGroup& msg) { return msg.getNumMessages(); })
        .def(
            "__iter__",
            [](MessageGroup& msg) { return py::make_iterator(msg.begin(), msg.end()); },
            py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def(
            "items",
            [](MessageGroup& msg) { return py::make_iterator(msg.begin(), msg.end()); },
            py::keep_alive<0, 1>() /* Essential: keep object alive while iterator exists */)
        .def("keys", &MessageGroup::getMessageIndices)
        .def("values", [](MessageGroup& msg) {
            std::vector<std::shared_ptr<ADatatype>> values;
            values.reserve(msg.group.size());
            for(auto& [_, value] : msg.group) {
                values.push_back(value);
            }
            return values;
        })
        .def("get", [](MessageGroup& msg, const std::string& name) { return msg.get(name); }, py::arg("index"))
        .def("get", [](MessageGroup& msg, uint32_t index) { return msg.get(index); }, py::arg("index"))
        .def("addMessage", [](MessageGroup& msg, const std::string& name, std::shared_ptr<ADatatype> data) { return msg.addMessage(name, data); },
             py::arg("index"),
             py::arg("message"))
        .def("addMessage", [](MessageGroup& msg, uint32_t index, std::shared_ptr<ADatatype> data) { return msg.addMessage(index, data); },
             py::arg("index"),
             py::arg("message"))
        .def("setNode", &MessageGroup::setNode, py::arg("nodeIndex"), py::arg("node"), DOC(dai, MessageGroup, setNode))
        .def("getNode", &MessageGroup::getNode, py::arg("nodeIndex"), DOC(dai, MessageGroup, getNode))
        .def("addLink",
             py::overload_cast<uint32_t, uint32_t, uint32_t>(&MessageGroup::addLink),
             py::arg("parentNodeIndex"),
             py::arg("childNodeIndex"),
             py::arg("parentItemIndex") = 0,
             DOC(dai, MessageGroup, addLink))
        .def("addLink", py::overload_cast<const dai::Link&>(&MessageGroup::addLink), py::arg("link"), DOC(dai, MessageGroup, addLink, 2))
        .def("hasLink", &MessageGroup::hasLink, py::arg("parentNodeIndex"), py::arg("parentItemIndex"), py::arg("childNodeIndex"), DOC(dai, MessageGroup, hasLink))
        .def("removeLink", &MessageGroup::removeLink, py::arg("linkIndex"), DOC(dai, MessageGroup, removeLink))
        .def("removeMessageNode", &MessageGroup::removeMessageNode, py::arg("nodeIndex"), DOC(dai, MessageGroup, removeMessageNode))
        .def("getLinksFromParent",
             py::overload_cast<uint32_t>(&MessageGroup::getLinksFromParent, py::const_),
             py::arg("parentNodeIndex"),
             DOC(dai, MessageGroup, getLinksFromParent))
        .def("getLinksFromParent",
             py::overload_cast<uint32_t, uint32_t>(&MessageGroup::getLinksFromParent, py::const_),
             py::arg("parentNodeIndex"),
             py::arg("parentItemIndex"),
             DOC(dai, MessageGroup, getLinksFromParent, 2))
        .def("getLinksToChild", &MessageGroup::getLinksToChild, py::arg("childNodeIndex"), DOC(dai, MessageGroup, getLinksToChild))
        .def("getChildren",
             py::overload_cast<uint32_t>(&MessageGroup::getChildren, py::const_),
             py::arg("parentNodeIndex"),
             DOC(dai, MessageGroup, getChildren))
        .def("getChildren",
             py::overload_cast<uint32_t, uint32_t>(&MessageGroup::getChildren, py::const_),
             py::arg("parentNodeIndex"),
             py::arg("parentItemIndex"),
             DOC(dai, MessageGroup, getChildren, 2))
        .def("getParents", &MessageGroup::getParents, py::arg("childNodeIndex"), DOC(dai, MessageGroup, getParents))
        .def("isLeaf", &MessageGroup::isLeaf, py::arg("nodeIndex"), DOC(dai, MessageGroup, isLeaf))
        .def("isRoot", &MessageGroup::isRoot, py::arg("nodeIndex"), DOC(dai, MessageGroup, isRoot))
        .def("getRootMessageNodes", &MessageGroup::getRootMessageNodes, DOC(dai, MessageGroup, getRootMessageNodes))
        .def("getMessageSiblings", &MessageGroup::getMessageSiblings, py::arg("nodeIndex"), DOC(dai, MessageGroup, getMessageSiblings))
        .def("depthFirstOrder", &MessageGroup::depthFirstOrder, py::arg("rootNodeIndex"), DOC(dai, MessageGroup, depthFirstOrder))
        .def("breadthFirstOrder", &MessageGroup::breadthFirstOrder, py::arg("rootNodeIndex"), DOC(dai, MessageGroup, breadthFirstOrder))
        .def("getLastMessageIndex", &MessageGroup::getLastMessageIndex, DOC(dai, MessageGroup, getLastMessageIndex))
        .def("isSynced", &MessageGroup::isSynced, py::arg("thresholdNs"), DOC(dai, MessageGroup, isSynced))
        .def("getIntervalNs", &MessageGroup::getIntervalNs, DOC(dai, MessageGroup, getIntervalNs))
        .def("getNumMessages", &MessageGroup::getNumMessages, DOC(dai, MessageGroup, getNumMessages))
        .def("getMessageNames", &MessageGroup::getMessageNames)
        .def("getMessageIndices", &MessageGroup::getMessageIndices)
        .def("getTimestamp", &MessageGroup::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &MessageGroup::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &MessageGroup::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("setTimestamp", &MessageGroup::setTimestamp, py::arg("timestamp"), DOC(dai, Buffer, setTimestamp))
        .def("setTimestampDevice", &MessageGroup::setTimestampDevice, py::arg("timestampDevice"), DOC(dai, Buffer, setTimestampDevice))
        .def("setSequenceNum", &MessageGroup::setSequenceNum, py::arg("sequenceNum"), DOC(dai, Buffer, setSequenceNum));
}
