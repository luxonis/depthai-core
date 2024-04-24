#include "Common.hpp"

#include "depthai/pipeline/node/host/Record.hpp"

void bind_record(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace node;

    auto record = ADD_NODE_DERIVED(Record, ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Node
    record.def_readonly("input", &Record::input, DOC(dai, node, Record, input))
        .def("setRecordFile", &Record::setRecordFile, py::arg("recordFile"), DOC(dai, node, Record, setRecordFile))
        .def("setCompressionLevel", &Record::setCompressionLevel, py::arg("compressionLevel"), DOC(dai, node, Record, setCompressionLevel));

}
