#include "NNArchiveConfigBindings.hpp"

#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

// depthai
#include "depthai/nn_archive/NNArchiveConfig.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<uint8_t>);

void NNArchiveConfigBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NNArchiveConfig> nnArchiveConfig(m, "NNArchiveConfig", DOC(dai, NNArchiveConfig));

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

    nnArchiveConfig
        .def(py::init<const dai::Path&, NNArchiveEntry::Compression>(),
             py::arg("path"),
             py::arg("compression") = NNArchiveEntry::Compression::AUTO,
             DOC(dai, NNArchiveConfig, NNArchiveConfig))
        .def(py::init<const std::vector<uint8_t>&, NNArchiveEntry::Compression>(),
             py::arg("data"),
             py::arg("compression") = NNArchiveEntry::Compression::AUTO,
             DOC(dai, NNArchiveConfig, NNArchiveConfig))
        .def(py::init<const std::function<int()>&,
                      const std::function<std::shared_ptr<std::vector<uint8_t>>()>&,
                      const std::function<int64_t(int64_t, NNArchiveEntry::Seek)>&,
                      const std::function<int64_t(int64_t)>&,
                      const std::function<int()>&,
                      NNArchiveEntry::Compression>());
}
