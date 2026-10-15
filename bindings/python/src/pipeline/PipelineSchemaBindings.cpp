#include <pybind11/stl_bind.h>

#include "PipelineBindings.hpp"
#include "depthai/pipeline/PipelineSchema.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<dai::NodeConnectionSchema>);

void bind_pipeline_schema(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NodeConnectionSchema> connection(m, "NodeConnectionSchema", DOC(dai, NodeConnectionSchema));
    py::class_<NodeIoInfo> io(m, "NodeIoInfo", DOC(dai, NodeIoInfo));
    py::enum_<NodeIoInfo::Type> ioType(io, "Type", DOC(dai, NodeIoInfo, Type));
    py::class_<NodeObjInfo> node(m, "NodeObjInfo", DOC(dai, NodeObjInfo));
    py::class_<PipelineSchema> schema(m, "PipelineSchema", DOC(dai, PipelineSchema));
    py::bind_vector<std::vector<NodeConnectionSchema>>(m, "VectorNodeConnectionSchema");

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    py::implicitly_convertible<py::list, std::vector<NodeConnectionSchema>>();
    connection.def(py::init<>())
        .def_readwrite("node1Id", &NodeConnectionSchema::node1Id, DOC(dai, NodeConnectionSchema, node1Id))
        .def_readwrite("node1OutputGroup", &NodeConnectionSchema::node1OutputGroup, DOC(dai, NodeConnectionSchema, node1OutputGroup))
        .def_readwrite("node1Output", &NodeConnectionSchema::node1Output, DOC(dai, NodeConnectionSchema, node1Output))
        .def_readwrite("node2Id", &NodeConnectionSchema::node2Id, DOC(dai, NodeConnectionSchema, node2Id))
        .def_readwrite("node2InputGroup", &NodeConnectionSchema::node2InputGroup, DOC(dai, NodeConnectionSchema, node2InputGroup))
        .def_readwrite("node2Input", &NodeConnectionSchema::node2Input, DOC(dai, NodeConnectionSchema, node2Input));

    ioType.value("MSender", NodeIoInfo::Type::MSender)
        .value("SSender", NodeIoInfo::Type::SSender)
        .value("MReceiver", NodeIoInfo::Type::MReceiver)
        .value("SReceiver", NodeIoInfo::Type::SReceiver);
    io.def(py::init<>())
        .def_readwrite("group", &NodeIoInfo::group, DOC(dai, NodeIoInfo, group))
        .def_readwrite("name", &NodeIoInfo::name, DOC(dai, NodeIoInfo, name))
        .def_readwrite("type", &NodeIoInfo::type, DOC(dai, NodeIoInfo, type))
        .def_readwrite("blocking", &NodeIoInfo::blocking, DOC(dai, NodeIoInfo, blocking))
        .def_readwrite("queueSize", &NodeIoInfo::queueSize, DOC(dai, NodeIoInfo, queueSize))
        .def_readwrite("waitForMessage", &NodeIoInfo::waitForMessage, DOC(dai, NodeIoInfo, waitForMessage))
        .def_readwrite("id", &NodeIoInfo::id, DOC(dai, NodeIoInfo, id));

    node.def(py::init<>())
        .def_readwrite("id", &NodeObjInfo::id, DOC(dai, NodeObjInfo, id))
        .def_readwrite("parentId", &NodeObjInfo::parentId, DOC(dai, NodeObjInfo, parentId))
        .def_readwrite("name", &NodeObjInfo::name, DOC(dai, NodeObjInfo, name))
        .def_readwrite("alias", &NodeObjInfo::alias, DOC(dai, NodeObjInfo, alias))
        .def_readwrite("deviceId", &NodeObjInfo::deviceId, DOC(dai, NodeObjInfo, deviceId))
        .def_readwrite("deviceNode", &NodeObjInfo::deviceNode, DOC(dai, NodeObjInfo, deviceNode))
        .def_readwrite("builtInNode", &NodeObjInfo::builtInNode, DOC(dai, NodeObjInfo, builtInNode))
        .def_readwrite("properties", &NodeObjInfo::properties, DOC(dai, NodeObjInfo, properties))
        .def_readwrite("logLevel", &NodeObjInfo::logLevel, DOC(dai, NodeObjInfo, logLevel))
        .def_readwrite("ioInfo", &NodeObjInfo::ioInfo, DOC(dai, NodeObjInfo, ioInfo));

    schema.def(py::init<>())
        .def_readwrite("connections", &PipelineSchema::connections, DOC(dai, PipelineSchema, connections))
        .def_readwrite("globalProperties", &PipelineSchema::globalProperties, DOC(dai, PipelineSchema, globalProperties))
        .def_readwrite("nodes", &PipelineSchema::nodes, DOC(dai, PipelineSchema, nodes))
        .def_readwrite("bridges", &PipelineSchema::bridges, DOC(dai, PipelineSchema, bridges));
}
