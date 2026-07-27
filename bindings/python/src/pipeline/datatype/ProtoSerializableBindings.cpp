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

#ifdef DEPTHAI_ENABLE_PROTOBUF
    // NOTE on the GIL: pybind11 holds the GIL for the whole duration of a bound C++ call
    // unless a gil_scoped_release call_guard is given. save() therefore blocks every other
    // Python thread for its entire encode+write (hundreds of ms for a large ImgFrame),
    // which makes a background saver thread run *instead of* the capture loop rather than
    // beside it. The saveGilReleased/saveStream* variants below drop the GIL for that work.
    //
    // Releasing it is safe here: the underlying code is pure core C++ and touches no Python
    // object or refcount, and the message is kept alive for the duration of the call by the
    // caller's reference. The caller must not mutate the message from another thread while
    // the save is in flight -- and for the consuming variant, must not use it afterwards.
    //
    // save() itself is deliberately left exactly as it was, so existing behaviour is
    // unchanged and it serves as an in-binary control when benchmarking the variants.
    protoSerializable.def("save", &ProtoSerializable::save, py::arg("path"), py::arg("metadataOnly") = false)
        .def("load", &ProtoSerializable::load, py::arg("path"))
        // Identical to save(), but releases the GIL for the encode+write so other Python
        // threads keep running. Isolates the effect of the GIL alone.
        .def("saveGilReleased",
             &ProtoSerializable::save,
             py::arg("path"),
             py::arg("metadataOnly") = false,
             py::call_guard<py::gil_scoped_release>())
        // As saveGilReleased(), and additionally encodes straight into the file instead of
        // building the encoded message in memory first: one less full copy of the payload.
        .def(
            "saveStreaming",
            [](ProtoSerializable& self, const std::filesystem::path& path, bool metadataOnly) { self.saveStream(path, metadataOnly, false); },
            py::arg("path"),
            py::arg("metadataOnly") = false,
            py::call_guard<py::gil_scoped_release>())
        // As saveStreaming(), and additionally releases the payload buffer as soon as it has
        // been copied into the protobuf message, i.e. before the slow encode+write. Peak
        // memory per in-flight save drops from two payload copies to one.
        // DESTRUCTIVE: the message has no payload afterwards. Only for fire-and-forget saves.
        .def(
            "saveConsuming",
            [](ProtoSerializable& self, const std::filesystem::path& path, bool metadataOnly) { self.saveStream(path, metadataOnly, true); },
            py::arg("path"),
            py::arg("metadataOnly") = false,
            py::call_guard<py::gil_scoped_release>());
#else
    protoSerializable
        .def(
            "save",
            [](py::object&, const std::filesystem::path& path, bool metadataOnly) { throw std::runtime_error("save() requires Protobuf support"); },
            py::arg("path"),
            py::arg("metadataOnly") = false)
        .def("load", [](py::object&, const std::filesystem::path& path) { throw std::runtime_error("load() requires Protobuf support"); }, py::arg("path"))
        .def(
            "saveGilReleased",
            [](py::object&, const std::filesystem::path& path, bool metadataOnly) { throw std::runtime_error("saveGilReleased() requires Protobuf support"); },
            py::arg("path"),
            py::arg("metadataOnly") = false)
        .def(
            "saveStreaming",
            [](py::object&, const std::filesystem::path& path, bool metadataOnly) { throw std::runtime_error("saveStreaming() requires Protobuf support"); },
            py::arg("path"),
            py::arg("metadataOnly") = false)
        .def(
            "saveConsuming",
            [](py::object&, const std::filesystem::path& path, bool metadataOnly) { throw std::runtime_error("saveConsuming() requires Protobuf support"); },
            py::arg("path"),
            py::arg("metadataOnly") = false);
#endif
}
