#include "NNArchiveEntryBindings.hpp"

// depthai
#include "depthai/nn_archive/NNArchiveEntry.hpp"

void NNArchiveEntryBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<NNArchiveEntry> nnArchiveEntry(m, "NNArchiveEntry", DOC(dai, NNArchiveEntry));
    py::enum_<NNArchiveEntry::Compression> archiveEntryCompression(nnArchiveEntry, "Compression");
    py::enum_<NNArchiveEntry::Seek> archiveEntrySeek(nnArchiveEntry, "Seek");

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

    archiveEntrySeek.value("SET", NNArchiveEntry::Seek::SET)
        .value("CUR", NNArchiveEntry::Seek::CUR)
        .value("END", NNArchiveEntry::Seek::END);

}
