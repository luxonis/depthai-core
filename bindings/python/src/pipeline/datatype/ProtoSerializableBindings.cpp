#include "DatatypeBindings.hpp"
#include "depthai/utility/ProtoSerializable.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_protoserializable(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ProtoSerializable, std::shared_ptr<ProtoSerializable>> protoSerializable(m, "ProtoSerializable", DOC(dai, ProtoSerializable));

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

#ifdef DEPTHAI_ENABLE_PROTOBU
    protoSerializable.def("save", &ProtoSerializable::save, py::arg("path"), py::arg("metadataOnly") = false)
        .def("load", &ProtoSerializable::load, py::arg("path"));
#else
    protoSerializable
        .def(
            "save",
            [](py::object&, const std::filesystem::path& path, bool metadataOnly) { throw std::runtime_error("save() requires Protobuf support"); },
            py::arg("path"),
            py::arg("metadataOnly") = false)
        .def("load", [](py::object&, const std::filesystem::path& path) { throw std::runtime_error("load() requires Protobuf support"); }, py::arg("path"));
#endif
}
