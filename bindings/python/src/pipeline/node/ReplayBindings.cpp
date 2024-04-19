#include "Common.hpp"

#include "depthai/pipeline/node/host/Replay.hpp"

void bind_replay(pybind11::module& m, void* pCallstack){

    using namespace dai;
    using namespace node;

    auto replay = ADD_NODE_DERIVED(Replay, HostNode);

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
    replay.def_readonly("out", &Replay::out, DOC(dai, node, Replay, out))
        .def("setReplayFile", &Replay::setReplayFile, py::arg("replayFile"), DOC(dai, node, Replay, setReplayFile))
        .def("setReplayVideo", &Replay::setReplayVideo, py::arg("replayVideo"), DOC(dai, node, Replay, setReplayVideo))
        .def("setOutFrameType", &Replay::setOutFrameType, py::arg("frameType"), DOC(dai, node, Replay, setOutFrameType));

}
