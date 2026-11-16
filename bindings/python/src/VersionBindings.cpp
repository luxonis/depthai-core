#include "VersionBindings.hpp"

// depthai
#include "depthai/device/Version.hpp"

void VersionBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::class_<Version> version(m, "Version", DOC(dai, Version));
    py::enum_<Version::PreReleaseType> preReleaseType(version, "PreReleaseType", DOC(dai, Version, PreReleaseType));

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

    preReleaseType.value("ALPHA", Version::PreReleaseType::ALPHA)
        .value("BETA", Version::PreReleaseType::BETA)
        .value("RC", Version::PreReleaseType::RC)
        .value("NONE", Version::PreReleaseType::NONE);

    version.def(py::init<const std::string&>(), py::arg("v"), DOC(dai, Version, Version))
        .def(py::init<unsigned, unsigned, unsigned, const Version::PreReleaseType&, const std::optional<uint16_t>&, const std::string&>(),
             py::arg("major"),
             py::arg("minor"),
             py::arg("patch"),
             py::arg("type") = Version::PreReleaseType::NONE,
             py::arg("preReleaseVersion") = std::nullopt,
             py::arg("buildInfo") = "",
             DOC(dai, Version, Version, 2))
        .def(py::init<unsigned, unsigned, unsigned, const std::string&>(),
             py::arg("major"),
             py::arg("minor"),
             py::arg("patch"),
             py::arg("buildInfo"),
             DOC(dai, Version, Version, 3))
        .def("__str__", &Version::toString, DOC(dai, Version, toString))
        .def("toString", &Version::toString, DOC(dai, Version, toString))
        .def("__eq__", &Version::operator==, DOC(dai, Version, operator_eq))
        .def("__lt__", &Version::operator<, DOC(dai, Version, operator_lt))
        .def("__gt__", &Version::operator>, DOC(dai, Version, operator_gt))
        .def("__le__", &Version::operator<=, DOC(dai, Version, operator_le))
        .def("__ge__", &Version::operator>=, DOC(dai, Version, operator_ge))
        .def("toStringSemver", &Version::toStringSemver, DOC(dai, Version, toStringSemver))
        .def("getBuildInfo", &Version::getBuildInfo, DOC(dai, Version, getBuildInfo));
}
