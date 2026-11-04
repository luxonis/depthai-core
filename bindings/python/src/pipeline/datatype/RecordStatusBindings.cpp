#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/RecordStatus.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_record_status(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    py::class_<RecordStatus, Buffer, py::smart_holder> recordStatus(m, "RecordStatus", DOC(dai, RecordStatus));
    py::enum_<RecordStatus::State> state(recordStatus, "State", DOC(dai, RecordStatus, State));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    state.value("STARTED", RecordStatus::State::STARTED).value("STOPPED", RecordStatus::State::STOPPED).value("ERROR", RecordStatus::State::ERROR);

    recordStatus.def(py::init<>())
        .def_readwrite("state", &RecordStatus::state, DOC(dai, RecordStatus, state))
        .def_readwrite("segment", &RecordStatus::segment, DOC(dai, RecordStatus, segment))
        .def_readwrite("frameCount", &RecordStatus::frameCount, DOC(dai, RecordStatus, frameCount))
        .def_readwrite("firstSequenceNum", &RecordStatus::firstSequenceNum, DOC(dai, RecordStatus, firstSequenceNum))
        .def_readwrite("lastSequenceNum", &RecordStatus::lastSequenceNum, DOC(dai, RecordStatus, lastSequenceNum))
        .def_readwrite("skippedFrames", &RecordStatus::skippedFrames, DOC(dai, RecordStatus, skippedFrames))
        .def_readwrite("path", &RecordStatus::path, DOC(dai, RecordStatus, path))
        .def_readwrite("error", &RecordStatus::error, DOC(dai, RecordStatus, error))
        .def("getDatatype", &RecordStatus::getDatatype, DOC(dai, RecordStatus, getDatatype));
}
