#include <memory>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"

void bind_benchmarkreport(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawBenchmarkReport, RawBuffer, std::shared_ptr<RawBenchmarkReport>> rawBenchmarkReport(m, "RawBenchmarkReport", DOC(dai, RawBenchmarkReport));
    py::class_<BenchmarkReport, Py<BenchmarkReport>, Buffer, std::shared_ptr<BenchmarkReport>> benchmarkReport(m, "BenchmarkReport", DOC(dai, BenchmarkReport));

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

    // Message
    benchmarkReport.def(py::init<>())
        .def("__repr__", &BenchmarkReport::str)
        .def_readonly("fps", &BenchmarkReport::fps)
        .def_readonly("timeTotal", &BenchmarkReport::timeTotal)
        .def_readonly("numMessagesReceived", &BenchmarkReport::numMessagesReceived)
        .def_readonly("latencies", &BenchmarkReport::latencies)
        .def_readonly("averageLatency", &BenchmarkReport::averageLatency);
}
