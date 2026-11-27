#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/BenchmarkReport.hpp"

void bind_benchmarkreport(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<BenchmarkReport, Py<BenchmarkReport>, Buffer, py::smart_holder> benchmarkReport(m, "BenchmarkReport", DOC(dai, BenchmarkReport));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    benchmarkReport.def(py::init<>())
        .def("__repr__", &BenchmarkReport::str)
        .def_readonly("fps", &BenchmarkReport::fps)
        .def_readonly("timeTotal", &BenchmarkReport::timeTotal)
        .def_readonly("numMessagesReceived", &BenchmarkReport::numMessagesReceived)
        .def_readonly("latencies", &BenchmarkReport::latencies)
        .def_readonly("averageLatency", &BenchmarkReport::averageLatency);
}
