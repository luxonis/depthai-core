#include "CapabilityBindings.hpp"

// depthai
#include "depthai/capabilities/Capability.hpp"

void CapabilityBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Capability> capability(m, "Capability", DOC(dai, Capability));

    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // NO actual bindings as it's a pure virtual class
}
