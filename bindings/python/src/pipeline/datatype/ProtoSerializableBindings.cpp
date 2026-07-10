#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/utility/ProtoSerializable.hpp"

void bind_protoserializable(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ProtoSerializable, std::shared_ptr<ProtoSerializable>> protoSerializable(m, "ProtoSerializable");

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

#ifdef DEPTHAI_ENABLE_PROTOBUF
    protoSerializable.def("save", &ProtoSerializable::save, py::arg("path"), py::arg("metadataOnly") = false)
        .def("load", &ProtoSerializable::load, py::arg("path"));
#endif
}
