#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "Common.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/utility/RecordingStore.hpp"

void bind_record(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace node;

    auto recordVideo = ADD_NODE(RecordVideo);
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
        .def_readonly("inputControl", &RecordVideo::inputControl, DOC(dai, node, RecordVideo, inputControl))
        .def_readonly("status", &RecordVideo::status, DOC(dai, node, RecordVideo, status))
        .def("setFps", &RecordVideo::setFps, py::arg("fps"), DOC(dai, node, RecordVideo, setFps))
        .def("setStreamFps", &RecordVideo::setStreamFps, py::arg("fps"), DOC(dai, node, RecordVideo, setStreamFps))
        .def("setRecordMetadataFile", &RecordVideo::setRecordMetadataFile, py::arg("recordFile"), DOC(dai, node, RecordVideo, setRecordMetadataFile))
        .def("setRecordVideoFile", &RecordVideo::setRecordVideoFile, py::arg("recordFile"), DOC(dai, node, RecordVideo, setRecordVideoFile))
        .def("setCompressionLevel", &RecordVideo::setCompressionLevel, py::arg("compressionLevel"), DOC(dai, node, RecordVideo, setCompressionLevel))
        .def("setFormat", &RecordVideo::setFormat, py::arg("format"), DOC(dai, node, RecordVideo, setFormat))
        .def("setRecordingName", &RecordVideo::setRecordingName, py::arg("name"), DOC(dai, node, RecordVideo, setRecordingName))
        .def("setOutputDirectory", &RecordVideo::setOutputDirectory, py::arg("path"), DOC(dai, node, RecordVideo, setOutputDirectory))
        .def("setStreamName", &RecordVideo::setStreamName, py::arg("name"), DOC(dai, node, RecordVideo, setStreamName))
        .def("setWaitForTrigger", &RecordVideo::setWaitForTrigger, py::arg("waitForTrigger"), DOC(dai, node, RecordVideo, setWaitForTrigger))
        .def("setPreTriggerFrames", &RecordVideo::setPreTriggerFrames, py::arg("frames"), DOC(dai, node, RecordVideo, setPreTriggerFrames))
        .def("setMaxPreTriggerBytes", &RecordVideo::setMaxPreTriggerBytes, py::arg("bytes"), DOC(dai, node, RecordVideo, setMaxPreTriggerBytes))
        .def("setTransferToHost", &RecordVideo::setTransferToHost, py::arg("transfer"), DOC(dai, node, RecordVideo, setTransferToHost))
        .def("setKeepOnDevice", &RecordVideo::setKeepOnDevice, py::arg("keep"), DOC(dai, node, RecordVideo, setKeepOnDevice))
        .def("setRunOnHost", &RecordVideo::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, RecordVideo, setRunOnHost))
        .def("getRecordMetadataFile", &RecordVideo::getRecordMetadataFile, DOC(dai, node, RecordVideo, getRecordMetadataFile))
        .def("getRecordVideoFile", &RecordVideo::getRecordVideoFile, DOC(dai, node, RecordVideo, getRecordVideoFile))
        .def("getCompressionLevel", &RecordVideo::getCompressionLevel, DOC(dai, node, RecordVideo, getCompressionLevel))
        .def("getFormat", &RecordVideo::getFormat, DOC(dai, node, RecordVideo, getFormat))
        .def("getRecordingName", &RecordVideo::getRecordingName, DOC(dai, node, RecordVideo, getRecordingName))
        .def("getOutputDirectory", &RecordVideo::getOutputDirectory, DOC(dai, node, RecordVideo, getOutputDirectory))
        .def("getStreamName", &RecordVideo::getStreamName, DOC(dai, node, RecordVideo, getStreamName))
        .def("getFps", &RecordVideo::getFps, DOC(dai, node, RecordVideo, getFps))
        .def("getWaitForTrigger", &RecordVideo::getWaitForTrigger, DOC(dai, node, RecordVideo, getWaitForTrigger))
        .def("getPreTriggerFrames", &RecordVideo::getPreTriggerFrames, DOC(dai, node, RecordVideo, getPreTriggerFrames))
        .def("getMaxPreTriggerBytes", &RecordVideo::getMaxPreTriggerBytes, DOC(dai, node, RecordVideo, getMaxPreTriggerBytes))
        .def("getTransferToHost", &RecordVideo::getTransferToHost, DOC(dai, node, RecordVideo, getTransferToHost))
        .def("getKeepOnDevice", &RecordVideo::getKeepOnDevice, DOC(dai, node, RecordVideo, getKeepOnDevice))
        .def("isLegacyLayout", &RecordVideo::isLegacyLayout, DOC(dai, node, RecordVideo, isLegacyLayout))
        .def("getStatus", &RecordVideo::getStatus, DOC(dai, node, RecordVideo, getStatus));

    recordMessage.def_readonly("input", &RecordMetadataOnly::input, DOC(dai, node, RecordMetadataOnly, input))
        .def("setRecordFile", &RecordMetadataOnly::setRecordFile, py::arg("recordFile"), DOC(dai, node, RecordMetadataOnly, setRecordFile))
        .def("setCompressionLevel",
             &RecordMetadataOnly::setCompressionLevel,
             py::arg("compressionLevel"),
             DOC(dai, node, RecordMetadataOnly, setCompressionLevel))
        .def("getRecordFile", &RecordMetadataOnly::getRecordFile, DOC(dai, node, RecordMetadataOnly, getRecordFile))
        .def("getCompressionLevel", &RecordMetadataOnly::getCompressionLevel, DOC(dai, node, RecordMetadataOnly, getCompressionLevel));

    daiNodeModule.attr("RecordMessage") = recordMessage;

    m.def("makeRecordingTimestamp", &makeRecordingTimestamp, DOC(dai, makeRecordingTimestamp));
    m.def("makeDefaultRecordingName", &makeDefaultRecordingName, DOC(dai, makeDefaultRecordingName));
}
