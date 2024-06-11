#include "Common.hpp"

#include "depthai/pipeline/node/host/Record.hpp"

void bind_record(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace node;

    auto recordVideo = ADD_NODE_DERIVED(RecordVideo, ThreadedHostNode);
    auto recordMessage = ADD_NODE_DERIVED(RecordMessage, ThreadedHostNode);

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
    recordVideo.def_readonly("input", &RecordVideo::input, DOC(dai, node, RecordVideo, input))
        .def("setRecordMetadataFile", &RecordVideo::setRecordMetadataFile, py::arg("recordFile"), DOC(dai, node, RecordVideo, setRecordMetadataFile))
        .def("setRecordVideoFile", &RecordVideo::setRecordVideoFile, py::arg("recordFile"), DOC(dai, node, RecordVideo, setRecordVideoFile))
        .def("setCompressionLevel", &RecordVideo::setCompressionLevel, py::arg("compressionLevel"), DOC(dai, node, RecordVideo, setCompressionLevel))
        .def("getRecordMetadataFile", &RecordVideo::getRecordMetadataFile, DOC(dai, node, RecordVideo, getRecordMetadataFile))
        .def("getRecordVideoFile", &RecordVideo::getRecordVideoFile, DOC(dai, node, RecordVideo, getRecordVideoFile))
        .def("getCompressionLevel", &RecordVideo::getCompressionLevel, DOC(dai, node, RecordVideo, getCompressionLevel));

    recordMessage.def_readonly("input", &RecordMessage::input, DOC(dai, node, RecordMessage, input))
        .def("setRecordFile", &RecordMessage::setRecordFile, py::arg("recordFile"), DOC(dai, node, RecordMessage, setRecordFile))
        .def("setRecordFile", &RecordMessage::setRecordFile, py::arg("recordFile"), DOC(dai, node, RecordMessage, setRecordFile))
        .def("setCompressionLevel", &RecordMessage::setCompressionLevel, py::arg("compressionLevel"), DOC(dai, node, RecordMessage, setCompressionLevel))
        .def("getRecordFile", &RecordMessage::getRecordFile, DOC(dai, node, RecordMessage, getRecordFile))
        .def("getCompressionLevel", &RecordMessage::getCompressionLevel, DOC(dai, node, RecordMessage, getCompressionLevel));

}
