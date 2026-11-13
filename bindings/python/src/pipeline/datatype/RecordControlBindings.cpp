#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/RecordControl.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_record_control(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    py::class_<RecordControl, Buffer, py::smart_holder> recordControl(m, "RecordControl", DOC(dai, RecordControl));
    py::enum_<RecordControl::Command> command(recordControl, "Command", DOC(dai, RecordControl, Command));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    command.value("START", RecordControl::Command::START).value("STOP", RecordControl::Command::STOP);

    recordControl.def(py::init<>())
        .def_readwrite("command", &RecordControl::command, DOC(dai, RecordControl, command))
        .def_readwrite("maxFrames", &RecordControl::maxFrames, DOC(dai, RecordControl, maxFrames))
        .def_readwrite("maxDurationMs", &RecordControl::maxDurationMs, DOC(dai, RecordControl, maxDurationMs))
        .def_readwrite("maxSizeKb", &RecordControl::maxSizeKb, DOC(dai, RecordControl, maxSizeKb))
        .def_static("start", py::overload_cast<>(&RecordControl::start), DOC(dai, RecordControl, start))
        .def_static(
            "start", py::overload_cast<std::int32_t, std::int32_t>(&RecordControl::start), "maxFrames"_a, "maxDurationMs"_a, DOC(dai, RecordControl, start, 2))
        .def_static("start",
                    py::overload_cast<std::int32_t, std::int32_t, std::int32_t>(&RecordControl::start),
                    "maxFrames"_a,
                    "maxDurationMs"_a,
                    "maxSizeKb"_a,
                    DOC(dai, RecordControl, start, 3))
        .def_static("stop", &RecordControl::stop, DOC(dai, RecordControl, stop))
        .def("getDatatype", &RecordControl::getDatatype, DOC(dai, RecordControl, getDatatype));
}
