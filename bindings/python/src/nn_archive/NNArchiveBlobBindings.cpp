#include "NNArchiveBlobBindings.hpp"

#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "depthai/nn_archive/NNArchiveBlob.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<uint8_t>);

void NNArchiveBlobBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NNArchiveBlob> nnArchiveBlob(m, "NNArchiveBlob", DOC(dai, NNArchiveBlob));

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

    nnArchiveBlob
        .def(py::init<const NNArchiveConfig&, const std::vector<uint8_t>&, NNArchiveEntry::Compression>(),
             py::arg("config"),
             py::arg("data"),
             py::arg("compression") = NNArchiveEntry::Compression::AUTO,
             DOC(dai, NNArchiveBlob, NNArchiveBlob))
        .def(py::init<const NNArchiveConfig&, const dai::Path&, NNArchiveEntry::Compression>(),
             py::arg("config"),
             py::arg("path"),
             py::arg("compression") = NNArchiveEntry::Compression::AUTO,
             DOC(dai, NNArchiveBlob, NNArchiveBlob))
        .def(py::init<const NNArchiveConfig&,
                      const std::function<int()>&,
                      const std::function<std::shared_ptr<std::vector<uint8_t>>()>&,
                      const std::function<int64_t(int64_t, NNArchiveEntry::Seek)>&,
                      const std::function<int64_t(int64_t)>&,
                      const std::function<int()>&,
                      NNArchiveEntry::Compression>(),
             py::arg("config"),
             py::arg("openCallback"),
             py::arg("readCallback"),
             py::arg("seekCallback"),
             py::arg("skipCallback"),
             py::arg("closeCallback"),
             py::arg("compression") = NNArchiveEntry::Compression::AUTO)
        .def("getOpenVINOBlob", &NNArchiveBlob::getOpenVINOBlob, DOC(dai, NNArchiveBlob, getOpenVINOBlob));
}
