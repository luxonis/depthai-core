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

    py::class_<utility::EventData, std::shared_ptr<utility::EventData>>(m, "EventData")
		.def(py::init<const std::string&, const std::string&, const std::string&>(), py::arg("data"), py::arg("fileName"), py::arg("mimeType"))
		.def(py::init<const std::string&>(), py::arg("fileUrl"))
		.def(py::init<const std::shared_ptr<ImgFrame>&, const std::string&>(), py::arg("imgFrame"), py::arg("fileName"))
		.def(py::init<const std::shared_ptr<NNData>&, const std::string&>(), py::arg("nnData"), py::arg("fileName"));

    py::class_<utility::EventsManager>(m, "EventsManager")
        .def(py::init<>())
        .def("setUrl", &utility::EventsManager::setUrl, py::arg("url"))
        .def("setSourceAppId", &utility::EventsManager::setSourceAppId, py::arg("sourceAppId"))
        .def("setSourceAppIdentifier", &utility::EventsManager::setSourceAppIdentifier, py::arg("sourceAppIdentifier"))
        .def("setToken", &utility::EventsManager::setToken, py::arg("token"))
        .def("setQueueSize", &utility::EventsManager::setQueueSize, py::arg("queueSize"))
        .def("setPublishInterval", &utility::EventsManager::setPublishInterval, py::arg("publishInterval"))
		.def("setLogResponse", &utility::EventsManager::setLogResponse, py::arg("logResponse"))
		.def("setDeviceSerialNumber", &utility::EventsManager::setDeviceSerialNumber, py::arg("deviceSerialNumber"))
        .def("sendEvent",
             &utility::EventsManager::sendEvent,
			 py::arg("name"),
			 py::arg("imgFrame").none(true) = nullptr,
			 py::arg("data") = std::vector<std::shared_ptr<utility::EventData>>(),
			 py::arg("tags") = std::vector<std::string>(),
			 py::arg("extraData") = std::unordered_map<std::string, std::string>(),
			 py::arg("deviceSerialNo") = "")
        .def("sendSnap",
             &utility::EventsManager::sendSnap,
			 py::arg("name"),
			 py::arg("imgFrame").none(true) = nullptr,
			 py::arg("data") = std::vector<std::shared_ptr<utility::EventData>>(),
			 py::arg("tags") = std::vector<std::string>(),
			 py::arg("extraData") = std::unordered_map<std::string, std::string>(),
			 py::arg("deviceSerialNo") = "");
}
