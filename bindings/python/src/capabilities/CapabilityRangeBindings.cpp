#include "CapabilityRangeBindings.hpp"

// for std::optional and std::variant support
#include <pybind11/stl.h>

// depthai
#include "depthai/capabilities/CapabilityRange.hpp"

void CapabilityRangeBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<CapabilityRange<uint32_t>> capabilityRange(m, "CapabilityRangeUint", DOC(dai, CapabilityRangeUint));
    py::class_<CapabilityRange<std::tuple<uint32_t, uint32_t>>> capabilityRangeTuple(m, "CapabilityRangeUintTuple", DOC(dai, CapabilityRangeUintTuple));

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    capabilityRange.def(py::init<>()).def_readwrite("value", &CapabilityRange<uint32_t>::value);
    capabilityRangeTuple.def(py::init<>()).def_readwrite("value", &CapabilityRange<std::tuple<uint32_t, uint32_t>>::value);
}
