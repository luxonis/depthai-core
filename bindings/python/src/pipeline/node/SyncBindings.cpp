#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/properties/SyncProperties.hpp"

void bind_sync(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<SyncProperties> syncProperties(m, "SyncProperties", DOC(dai, SyncProperties));
    py::enum_<SyncProperties::TimestampSource> tsSource(syncProperties, "TimestampSource");
    auto sync = ADD_NODE(Sync);

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

    tsSource.value("DEFAULT", SyncProperties::TimestampSource::DEFAULT)
        .value("DEVICE", SyncProperties::TimestampSource::DEVICE)
        .value("HOST", SyncProperties::TimestampSource::HOST)
        .value("SYSTEM", SyncProperties::TimestampSource::SYSTEM);

    sync.attr("TimestampSource") = tsSource;

    // Properties
    syncProperties.def_readwrite("syncThresholdNs", &SyncProperties::syncThresholdNs, DOC(dai, SyncProperties, syncThresholdNs))
        .def_readwrite("syncAttempts", &SyncProperties::syncAttempts, DOC(dai, SyncProperties, syncAttempts))
        .def_readwrite("processor", &SyncProperties::processor, DOC(dai, SyncProperties, processor))
        .def_readwrite("timestampSource", &SyncProperties::timestampSource, DOC(dai, SyncProperties, timestampSource));

    // Node
    sync.def_readonly("out", &Sync::out, DOC(dai, node, Sync, out))
        .def_readonly("inputs", &Sync::inputs, DOC(dai, node, Sync, inputs))
        .def("setSyncThreshold", &Sync::setSyncThreshold, py::arg("syncThreshold"), DOC(dai, node, Sync, setSyncThreshold))
        .def("setSyncAttempts", &Sync::setSyncAttempts, py::arg("maxDataSize"), DOC(dai, node, Sync, setSyncAttempts))
        .def("getSyncThreshold", &Sync::getSyncThreshold, DOC(dai, node, Sync, getSyncThreshold))
        .def("getSyncAttempts", &Sync::getSyncAttempts, DOC(dai, node, Sync, getSyncAttempts))
        .def("setProcessor", &Sync::setProcessor, py::arg("processorType"), DOC(dai, node, Sync, setProcessor))
        .def("getProcessor", &Sync::getProcessor, DOC(dai, node, Sync, getProcessor))
        .def("setTimestampSource", &Sync::setTimestampSource, py::arg("source"), DOC(dai, node, Sync, setTimestampSource))
        .def("getTimestampSource", &Sync::getTimestampSource, DOC(dai, node, Sync, getTimestampSource))
        .def("setRunOnHost", &Sync::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, Sync, setRunOnHost))
        .def("runOnHost", &Sync::runOnHost, DOC(dai, node, Sync, runOnHost));
    daiNodeModule.attr("Sync").attr("Properties") = syncProperties;
}
