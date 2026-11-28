#include "CapabilityBindings.hpp"

#include "depthai/capabilities/Capability.hpp"

void CapabilityBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<Capability> capability(m, "Capability", DOC(dai, Capability));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    capability.def("getName", &dai::Capability::getName, py::return_value_policy::reference_internal, DOC(dai, Capability, getName));
}
