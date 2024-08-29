#include "RemoteConnectorBindings.hpp"

#include "depthai/remote_connection/RemoteConnector.hpp"

void RemoteConnectorBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    py::class_<RemoteConnector>(m, "RemoteConnector")
        .def(py::init<>())
        .def("initServer", &RemoteConnector::initServer, py::arg("address") = "0.0.0.0", py::arg("port") = 8765);
}