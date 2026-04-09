#include "PlatformBindings.hpp"

// depthai
#include "depthai/device/Platform.hpp"

void PlatformBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::enum_<Platform> platform(m, "Platform", DOC(dai, Platform));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Bind Platform
    platform.value("RVC2", Platform::RVC2, DOC(dai, Platform, RVC2))
        .value("RVC3", Platform::RVC3, DOC(dai, Platform, RVC3))
        .value("RVC4", Platform::RVC4, DOC(dai, Platform, RVC4));

    // Platform - string conversion
    m.def("platform2string", &platform2string, DOC(dai, platform2string));
    m.def("string2platform", &string2platform, DOC(dai, string2platform));
}
