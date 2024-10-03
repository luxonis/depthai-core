#include "EventsManagerBindings.hpp"

// depthai
#include "depthai/utility/EventsManager.hpp"

void EventsManagerBindings::bind(pybind11::module& m, void* pCallstack) {
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

    py::class_<utility::FileData>(m, "FileData")
		.def(py::init<>())
        .def_readwrite("data", &utility::FileData::data)
        .def_readwrite("fileName", &utility::FileData::fileName)
        .def_readwrite("fileUrl", &utility::FileData::fileUrl)
        .def_readwrite("mimeType", &utility::FileData::mimeType);

    py::class_<utility::EventsManager>(m, "EventsManager")
        .def(py::init<const std::string&>(), py::arg("deviceSerialNumber") = "serial")
        .def("setUrl", &utility::EventsManager::setUrl, py::arg("url"))
        .def("setSourceAppId", &utility::EventsManager::setSourceAppId, py::arg("sourceAppId"))
        .def("setSourceAppIdentifier", &utility::EventsManager::setSourceAppIdentifier, py::arg("sourceAppIdentifier"))
        .def("setToken", &utility::EventsManager::setToken, py::arg("token"))
        .def("setQueueSize", &utility::EventsManager::setQueueSize, py::arg("queueSize"))
        .def("setPublishInterval", &utility::EventsManager::setPublishInterval, py::arg("publishInterval"))
		.def("setLogResponse", &utility::EventsManager::setLogResponse, py::arg("logResponse"))
        .def("sendEvent",
             &utility::EventsManager::sendEvent,
             py::arg("name"),
             py::arg("data"),
             py::arg("tags"),
             py::arg("files") = std::vector<utility::FileData>(),
             py::arg("daiMsg") = nullptr)
        .def("sendSnap",
             &utility::EventsManager::sendSnap,
             py::arg("name"),
             py::arg("data"),
             py::arg("tags"),
             py::arg("files") = std::vector<utility::FileData>(),
             py::arg("daiMsg") = nullptr);
}
