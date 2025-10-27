#include "EventsManagerBindings.hpp"

// depthai
#ifdef DEPTHAI_ENABLE_EVENTS_MANAGER
    #include "depthai/utility/EventsManager.hpp"
#endif

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

#ifdef DEPTHAI_ENABLE_EVENTS_MANAGER
    using namespace dai::utility;
    py::class_<EventData, std::shared_ptr<utility::EventData>>(m, "EventData")
        .def(py::init<const std::string&, const std::string&, const std::string&>(), py::arg("data"), py::arg("fileName"), py::arg("mimeType"))
        .def(py::init<std::string>(), py::arg("fileUrl"))
        .def(py::init<const std::shared_ptr<ImgFrame>&, std::string>(), py::arg("imgFrame"), py::arg("fileName"))
        .def(py::init<const std::shared_ptr<EncodedFrame>&, std::string>(), py::arg("encodedFrame"), py::arg("fileName"))
        .def(py::init<const std::shared_ptr<NNData>&, std::string>(), py::arg("nnData"), py::arg("fileName"));

    py::class_<EventsManager>(m, "EventsManager")
        .def(py::init<>())
        .def(py::init<std::string, bool, float>(), py::arg("url"), py::arg("uploadCachedOnStart") = false, py::arg("publishInterval") = 10.0)
        .def("setUrl", &EventsManager::setUrl, py::arg("url"), DOC(dai, utility, EventsManager, setUrl))
        .def("setSourceAppId", &EventsManager::setSourceAppId, py::arg("sourceAppId"), DOC(dai, utility, EventsManager, setSourceAppId))
        .def("setSourceAppIdentifier",
             &EventsManager::setSourceAppIdentifier,
             py::arg("sourceAppIdentifier"),
             DOC(dai, utility, EventsManager, setSourceAppIdentifier))
        .def("setToken", &EventsManager::setToken, py::arg("token"), DOC(dai, utility, EventsManager, setToken))
        .def("setQueueSize", &EventsManager::setQueueSize, py::arg("queueSize"), DOC(dai, utility, EventsManager, setQueueSize))
        .def("setLogResponse", &EventsManager::setLogResponse, py::arg("logResponse"), DOC(dai, utility, EventsManager, setLogResponse))
        .def("setDeviceSerialNumber",
             &EventsManager::setDeviceSerialNumber,
             py::arg("deviceSerialNumber"),
             DOC(dai, utility, EventsManager, setDeviceSerialNumber))
        .def("setVerifySsl", &EventsManager::setVerifySsl, py::arg("verifySsl"), DOC(dai, utility, EventsManager, setVerifySsl))
        .def("setCacheDir", &EventsManager::setCacheDir, py::arg("cacheDir"), DOC(dai, utility, EventsManager, setCacheDir))
        .def("setCacheIfCannotSend",
             &EventsManager::setCacheIfCannotSend,
             py::arg("cacheIfCannotUpload"),
             DOC(dai, utility, EventsManager, setCacheIfCannotSend))
        .def("checkConnection", &EventsManager::checkConnection, DOC(dai, utility, EventsManager, checkConnection))
        .def("uploadCachedData", &EventsManager::uploadCachedData, DOC(dai, utility, EventsManager, uploadCachedData))
        .def("sendEvent",
             &EventsManager::sendEvent,
             py::arg("name"),
             py::arg("imgFrame").none(true) = nullptr,
             py::arg("data") = std::vector<std::shared_ptr<EventData>>(),
             py::arg("tags") = std::vector<std::string>(),
             py::arg("extraData") = std::unordered_map<std::string, std::string>(),
             py::arg("deviceSerialNo") = "",
             DOC(dai, utility, EventsManager, sendEvent))
        .def("sendSnap",
             &EventsManager::sendSnap,
             py::arg("name"),
             py::arg("imgFrame").none(true) = nullptr,
             py::arg("data") = std::vector<std::shared_ptr<EventData>>(),
             py::arg("tags") = std::vector<std::string>(),
             py::arg("extraData") = std::unordered_map<std::string, std::string>(),
             py::arg("deviceSerialNo") = "",
             DOC(dai, utility, EventsManager, sendSnap));
#endif
}
