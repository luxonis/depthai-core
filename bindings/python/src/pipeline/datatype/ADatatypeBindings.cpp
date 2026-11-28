#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/ADatatype.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

namespace dai {
class PyADataType : public ADatatype, public py::trampoline_self_life_support {
   public:
    using ADatatype::ADatatype;
};
}  // namespace dai

void bind_adatatype(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ADatatype, PyADataType, py::smart_holder> adatatype(m, "ADatatype", DOC(dai, ADatatype));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    adatatype.def(py::init<>(), DOC(dai, ADatatype, ADatatype)).def("getDatatype", &dai::ADatatype::getDatatype, DOC(dai, ADatatype, getDatatype));
}
