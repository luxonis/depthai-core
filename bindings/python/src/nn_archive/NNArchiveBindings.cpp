#include "NNArchiveBindings.hpp"

// depthai
#include "depthai/nn_archive/NNArchive.hpp"

void NNArchiveBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NNArchive> nnArchive(m, "NNArchive", DOC(dai, NNArchive));
    py::class_<NNArchiveEntry> nnArchiveEntry(m, "NNArchiveEntry", DOC(dai, NNArchiveEntry));
    py::enum_<NNArchiveEntry::Compression> archiveEntryCompression(nnArchiveEntry, "Compression");

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

    archiveEntryCompression.value("AUTO", NNArchiveEntry::Compression::AUTO)
        .value("RAW_FS", NNArchiveEntry::Compression::RAW_FS)
        .value("TAR", NNArchiveEntry::Compression::TAR)
        .value("TAR_GZ", NNArchiveEntry::Compression::TAR_GZ)
        .value("TAR_XZ", NNArchiveEntry::Compression::TAR_XZ);

    // Bind NNArchive
    nnArchive.def(py::init<const dai::Path&, NNArchiveEntry::Compression>(),
                  py::arg("path"),
                  py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                  DOC(dai, NNArchive, NNArchive));
}

