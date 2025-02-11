#include <pybind11/stl/filesystem.h>

#include "Common.hpp"
#include "depthai/pipeline/node/host/Record.hpp"

void bind_record(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace node;

    auto recordVideo = ADD_NODE_DERIVED(RecordVideo, ThreadedHostNode);
    auto recordMessage = ADD_NODE_DERIVED(RecordMetadataOnly, ThreadedHostNode);

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
    // Node
    recordVideo.def_readonly("input", &RecordVideo::input, DOC(dai, node, RecordVideo, input))
        .def("setRecordMetadataFile", &RecordVideo::setRecordMetadataFile, py::arg("recordFile"), DOC(dai, node, RecordVideo, setRecordMetadataFile))
        .def("setRecordVideoFile", &RecordVideo::setRecordVideoFile, py::arg("recordFile"), DOC(dai, node, RecordVideo, setRecordVideoFile))
        .def("setCompressionLevel", &RecordVideo::setCompressionLevel, py::arg("compressionLevel"), DOC(dai, node, RecordVideo, setCompressionLevel))
        .def("getRecordMetadataFile", &RecordVideo::getRecordMetadataFile, DOC(dai, node, RecordVideo, getRecordMetadataFile))
        .def("getRecordVideoFile", &RecordVideo::getRecordVideoFile, DOC(dai, node, RecordVideo, getRecordVideoFile))
        .def("getCompressionLevel", &RecordVideo::getCompressionLevel, DOC(dai, node, RecordVideo, getCompressionLevel));

    recordMessage.def_readonly("input", &RecordMetadataOnly::input, DOC(dai, node, RecordMetadataOnly, input))
        .def("setRecordFile", &RecordMetadataOnly::setRecordFile, py::arg("recordFile"), DOC(dai, node, RecordMetadataOnly, setRecordFile))
        .def("setCompressionLevel",
             &RecordMetadataOnly::setCompressionLevel,
             py::arg("compressionLevel"),
             DOC(dai, node, RecordMetadataOnly, setCompressionLevel))
        .def("getRecordFile", &RecordMetadataOnly::getRecordFile, DOC(dai, node, RecordMetadataOnly, getRecordFile))
        .def("getCompressionLevel", &RecordMetadataOnly::getCompressionLevel, DOC(dai, node, RecordMetadataOnly, getCompressionLevel));
}
