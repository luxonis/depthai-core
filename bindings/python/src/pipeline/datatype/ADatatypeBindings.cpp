#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/StreamMessageParser.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

namespace dai {
class PyADataType : public ADatatype {
   public:
    using ADatatype::ADatatype;
};
}  // namespace dai

void bind_adatatype(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ADatatype, PyADataType, std::shared_ptr<ADatatype>> adatatype(m, "ADatatype", DOC(dai, ADatatype));

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

    adatatype.def(py::init<>(), DOC(dai, ADatatype, ADatatype))
        .def(
            "serialize",
            [](const ADatatype& message) {
                std::vector<std::uint8_t> serialized;
                const auto metadata = StreamMessageParser::serializeMetadata(message);
                if(message.data) {
                    const auto dataSpan = message.data->getData();
                    serialized.reserve(dataSpan.size() + metadata.size());
                    serialized.insert(serialized.end(), dataSpan.begin(), dataSpan.end());
                    serialized.insert(serialized.end(), metadata.begin(), metadata.end());
                } else {
                    serialized = metadata;
                }
                if(serialized.empty()) {
                    return py::bytes();
                }
                return py::bytes(reinterpret_cast<const char*>(serialized.data()), serialized.size());
            },
            "Serialize message to bytes (data + metadata)")
        .def_static(
            "deserialize",
            [](py::buffer data) {
                std::string bytes = data.cast<std::string>();
                streamPacketDesc_t packet{};
                packet.data = reinterpret_cast<std::uint8_t*>(bytes.data());
                packet.length = bytes.size();
                packet.fd = -1;
                return StreamMessageParser::parseMessage(&packet);
            },
            py::arg("data"),
            "Deserialize bytes (data + metadata) to ADatatype");
    // Message
    // adatatype
    // .def("getRaw", &ADatatype::getRaw);
}
