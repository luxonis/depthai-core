#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "Common.hpp"
#include "depthai/pipeline/node/host/Replay.hpp"

void bind_replay(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace node;

    auto replayVideo = ADD_NODE(ReplayVideo);
    auto replayMessage = ADD_NODE(ReplayMetadataOnly);

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
    replayVideo.def_readonly("out", &ReplayVideo::out, DOC(dai, node, ReplayVideo, out))
        .def("setReplayMetadataFile", &ReplayVideo::setReplayMetadataFile, py::arg("replayFile"), DOC(dai, node, ReplayVideo, setReplayMetadataFile))
        .def("setReplayVideoFile", &ReplayVideo::setReplayVideoFile, py::arg("replayVideoFile"), DOC(dai, node, ReplayVideo, setReplayVideoFile))
        .def("setOutFrameType", &ReplayVideo::setOutFrameType, py::arg("frameType"), DOC(dai, node, ReplayVideo, setOutFrameType))
        .def("setSize", py::overload_cast<int, int>(&ReplayVideo::setSize), py::arg("width"), py::arg("height"), DOC(dai, node, ReplayVideo, setSize))
        .def("setSize", py::overload_cast<std::tuple<int, int>>(&ReplayVideo::setSize), py::arg("size"), DOC(dai, node, ReplayVideo, setSize))
        .def("setFps", &ReplayVideo::setFps, py::arg("fps"), DOC(dai, node, ReplayVideo, setFps))
        .def("setLoop", &ReplayVideo::setLoop, py::arg("loop"), DOC(dai, node, ReplayVideo, setLoop))
        .def("setSource", &ReplayVideo::setSource, py::arg("path"), py::arg("location") = ReplaySourceLocation::HOST, DOC(dai, node, ReplayVideo, setSource))
        .def("setMetadataFile", &ReplayVideo::setMetadataFile, py::arg("path"), DOC(dai, node, ReplayVideo, setMetadataFile))
        .def("setDecodeMode", &ReplayVideo::setDecodeMode, py::arg("mode"), DOC(dai, node, ReplayVideo, setDecodeMode))
        .def("setRunOnHost", &ReplayVideo::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, ReplayVideo, setRunOnHost))
        .def("setPersistentUpload", &ReplayVideo::setPersistentUpload, py::arg("persistent"), DOC(dai, node, ReplayVideo, setPersistentUpload))
        .def("setArchiveEntry", &ReplayVideo::setArchiveEntry, py::arg("entry"), DOC(dai, node, ReplayVideo, setArchiveEntry))
        .def("setStopPipelineAtEnd", &ReplayVideo::setStopPipelineAtEnd, py::arg("stopPipeline"), DOC(dai, node, ReplayVideo, setStopPipelineAtEnd))
        .def("getReplayMetadataFile", &ReplayVideo::getReplayMetadataFile, DOC(dai, node, ReplayVideo, getReplayMetadataFile))
        .def("getReplayVideoFile", &ReplayVideo::getReplayVideoFile, DOC(dai, node, ReplayVideo, getReplayVideoFile))
        .def("getOutFrameType", &ReplayVideo::getOutFrameType, DOC(dai, node, ReplayVideo, getOutFrameType))
        .def("getOutFrameTypeSet", &ReplayVideo::getOutFrameTypeSet, DOC(dai, node, ReplayVideo, getOutFrameTypeSet))
        .def("getSize", &ReplayVideo::getSize, DOC(dai, node, ReplayVideo, getSize))
        .def("getFps", &ReplayVideo::getFps, DOC(dai, node, ReplayVideo, getFps))
        .def("getLoop", &ReplayVideo::getLoop, DOC(dai, node, ReplayVideo, getLoop))
        .def("getSourceLocation", &ReplayVideo::getSourceLocation, DOC(dai, node, ReplayVideo, getSourceLocation))
        .def("getDecodeMode", &ReplayVideo::getDecodeMode, DOC(dai, node, ReplayVideo, getDecodeMode))
        .def("getPersistentUpload", &ReplayVideo::getPersistentUpload, DOC(dai, node, ReplayVideo, getPersistentUpload))
        .def("getArchiveEntry", &ReplayVideo::getArchiveEntry, DOC(dai, node, ReplayVideo, getArchiveEntry))
        .def("getStopPipelineAtEnd", &ReplayVideo::getStopPipelineAtEnd, DOC(dai, node, ReplayVideo, getStopPipelineAtEnd))
        .def_static("isSupportedOutFrameType", &ReplayVideo::isSupportedOutFrameType, py::arg("type"), DOC(dai, node, ReplayVideo, isSupportedOutFrameType))
        .def_static("setReplayCacheLimit", &ReplayVideo::setReplayCacheLimit, py::arg("bytes"), DOC(dai, node, ReplayVideo, setReplayCacheLimit))
        .def_static("getReplayCacheInfo", &ReplayVideo::getReplayCacheInfo, DOC(dai, node, ReplayVideo, getReplayCacheInfo))
        .def_static("clearReplayCache", &ReplayVideo::clearReplayCache, DOC(dai, node, ReplayVideo, clearReplayCache));

    replayMessage.def_readonly("out", &ReplayMetadataOnly::out, DOC(dai, node, ReplayMetadataOnly, out))
        .def("setReplayFile", &ReplayMetadataOnly::setReplayFile, py::arg("replayFile"), DOC(dai, node, ReplayMetadataOnly, setReplayFile))
        .def("setFps", &ReplayMetadataOnly::setFps, py::arg("fps"), DOC(dai, node, ReplayMetadataOnly, setFps))
        .def("setLoop", &ReplayMetadataOnly::setLoop, py::arg("loop"), DOC(dai, node, ReplayMetadataOnly, setLoop))
        .def("setSource",
             &ReplayMetadataOnly::setSource,
             py::arg("path"),
             py::arg("location") = ReplaySourceLocation::HOST,
             DOC(dai, node, ReplayMetadataOnly, setSource))
        .def("setRunOnHost", &ReplayMetadataOnly::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, ReplayMetadataOnly, setRunOnHost))
        .def("setPersistentUpload", &ReplayMetadataOnly::setPersistentUpload, py::arg("persistent"), DOC(dai, node, ReplayMetadataOnly, setPersistentUpload))
        .def("setArchiveEntry", &ReplayMetadataOnly::setArchiveEntry, py::arg("entry"), DOC(dai, node, ReplayMetadataOnly, setArchiveEntry))
        .def("setStopPipelineAtEnd",
             &ReplayMetadataOnly::setStopPipelineAtEnd,
             py::arg("stopPipeline"),
             DOC(dai, node, ReplayMetadataOnly, setStopPipelineAtEnd))
        .def("getReplayFile", &ReplayMetadataOnly::getReplayFile, DOC(dai, node, ReplayMetadataOnly, getReplayFile))
        .def("getFps", &ReplayMetadataOnly::getFps, DOC(dai, node, ReplayMetadataOnly, getFps))
        .def("getLoop", &ReplayMetadataOnly::getLoop, DOC(dai, node, ReplayMetadataOnly, getLoop))
        .def("getSourceLocation", &ReplayMetadataOnly::getSourceLocation, DOC(dai, node, ReplayMetadataOnly, getSourceLocation))
        .def("getPersistentUpload", &ReplayMetadataOnly::getPersistentUpload, DOC(dai, node, ReplayMetadataOnly, getPersistentUpload))
        .def("getArchiveEntry", &ReplayMetadataOnly::getArchiveEntry, DOC(dai, node, ReplayMetadataOnly, getArchiveEntry))
        .def("getStopPipelineAtEnd", &ReplayMetadataOnly::getStopPipelineAtEnd, DOC(dai, node, ReplayMetadataOnly, getStopPipelineAtEnd));

    daiNodeModule.attr("ReplayMessage") = replayMessage;
}
