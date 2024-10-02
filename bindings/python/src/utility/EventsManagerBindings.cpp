#include "EventsManagerBindings.hpp"

// depthai
#include resources #include "depthai/utility/EventsManager.hpp"


void EventsManagerBindings::bind(pybind11::module& m, void* pCallstack){
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

    py::class_<EventsManager>(m, "EventsManager")
        .def(py::init<const std::string&, const std::string&, const std::string&>(), py::arg("sessionToken") = "token", py::arg("agentToken") = "token", py::arg("deviceSerialNumber") = "serial")
		.def("sendEvent", &EventsManager::sendEvent, py::arg("name"), py::arg("data"), py::arg("tags"))
		.def("sendSnap", &EventsManager::sendSnap, py::arg("name"), py::arg("data"), py::arg("tags"));
}


