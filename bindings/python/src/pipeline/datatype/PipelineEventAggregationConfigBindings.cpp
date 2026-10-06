#include <pybind11/stl_bind.h>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<dai::NodeEventAggregationConfig>);

void bind_pipelineeventaggregationconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NodeEventAggregationConfig> nodeConfig(m, "NodeEventAggregationConfig", DOC(dai, NodeEventAggregationConfig));
    py::class_<PipelineEventAggregationConfig, Py<PipelineEventAggregationConfig>, Buffer, std::shared_ptr<PipelineEventAggregationConfig>> config(
        m, "PipelineEventAggregationConfig", DOC(dai, PipelineEventAggregationConfig));

    auto* callstack = static_cast<Callstack*>(pCallstack);
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    py::bind_vector<std::vector<NodeEventAggregationConfig>>(m, "NodeEventAggregationConfigs");
    py::implicitly_convertible<py::list, std::vector<NodeEventAggregationConfig>>();

    nodeConfig.def(py::init<>())
        .def_readwrite("nodeId", &NodeEventAggregationConfig::nodeId, DOC(dai, NodeEventAggregationConfig, nodeId))
        .def_readwrite("inputs", &NodeEventAggregationConfig::inputs, DOC(dai, NodeEventAggregationConfig, inputs))
        .def_readwrite("outputs", &NodeEventAggregationConfig::outputs, DOC(dai, NodeEventAggregationConfig, outputs))
        .def_readwrite("others", &NodeEventAggregationConfig::others, DOC(dai, NodeEventAggregationConfig, others))
        .def_readwrite("events", &NodeEventAggregationConfig::events, DOC(dai, NodeEventAggregationConfig, events));

    config.def(py::init<>())
        .def_readwrite("nodes", &PipelineEventAggregationConfig::nodes, DOC(dai, PipelineEventAggregationConfig, nodes))
        .def_readwrite(
            "repeatIntervalSeconds", &PipelineEventAggregationConfig::repeatIntervalSeconds, DOC(dai, PipelineEventAggregationConfig, repeatIntervalSeconds));
}
