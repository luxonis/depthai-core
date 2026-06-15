#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/MissingDataMessage.hpp"

void bind_empty_message(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<MissingDataMessage, Py<MissingDataMessage>, Buffer, std::shared_ptr<MissingDataMessage>> missingDataMessage(m, "MissingDataMessage");

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

    missingDataMessage.def(py::init<>()).def("__repr__", &MissingDataMessage::str);

    m.attr("EmptyMessage") = m.attr("MissingDataMessage");
}
