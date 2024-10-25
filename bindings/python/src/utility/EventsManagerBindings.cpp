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
        .def(py::init<std::string>(), py::arg("fileUrl"))
        .def(py::init<const std::shared_ptr<ImgFrame>&, std::string>(), py::arg("imgFrame"), py::arg("fileName"))
        .def(py::init<const std::shared_ptr<EncodedFrame>&, std::string>(), py::arg("encodedFrame"), py::arg("fileName"))
        .def(py::init<const std::shared_ptr<NNData>&, std::string>(), py::arg("nnData"), py::arg("fileName"));

    py::class_<utility::EventsManager>(m, "EventsManager")
        .def(py::init<>())
        .def(py::init<std::string, bool, float>(), py::arg("url"), py::arg("uploadCachedOnStart") = false, py::arg("publishInterval") = 10.0)
        .def("setUrl", &utility::EventsManager::setUrl, py::arg("url"), DOC(dai, EventsManager, setUrl))
        .def("setSourceAppId", &utility::EventsManager::setSourceAppId, py::arg("sourceAppId"), DOC(dai, EventsManager, setSourceAppId))
        .def("setSourceAppIdentifier",
             &utility::EventsManager::setSourceAppIdentifier,
             py::arg("sourceAppIdentifier"),
             DOC(dai, EventsManager, setSourceAppIdentifier))
        .def("setToken", &utility::EventsManager::setToken, py::arg("token"), DOC(dai, EventsManager, setToken))
        .def("setQueueSize", &utility::EventsManager::setQueueSize, py::arg("queueSize"), DOC(dai, EventsManager, setQueueSize))
        .def("setLogResponse", &utility::EventsManager::setLogResponse, py::arg("logResponse"), DOC(dai, EventsManager, setLogResponse))
        .def("setDeviceSerialNumber",
             &utility::EventsManager::setDeviceSerialNumber,
             py::arg("deviceSerialNumber"),
             DOC(dai, EventsManager, setDeviceSerialNumber))
        .def("setVerifySsl", &utility::EventsManager::setVerifySsl, py::arg("verifySsl"), DOC(dai, EventsManager, setVerifySsl))
        .def("setCacheDir", &utility::EventsManager::setCacheDir, py::arg("cacheDir"), DOC(dai, EventsManager, setCacheDir))
        .def("setCacheIfCannotSend",
             &utility::EventsManager::setCacheIfCannotSend,
             py::arg("cacheIfCannotUpload"),
             DOC(dai, EventsManager, setCacheIfCannotSend))
        .def("checkConnection", &utility::EventsManager::checkConnection, DOC(dai, EventsManager, checkConnection))
        .def("uploadCachedData", &utility::EventsManager::uploadCachedData, DOC(dai, EventsManager, uploadCachedData))
        .def("sendEvent",
             &utility::EventsManager::sendEvent,
             py::arg("name"),
             py::arg("imgFrame").none(true) = nullptr,
             py::arg("data") = std::vector<std::shared_ptr<utility::EventData>>(),
             py::arg("tags") = std::vector<std::string>(),
             py::arg("extraData") = std::unordered_map<std::string, std::string>(),
             py::arg("deviceSerialNo") = "",
             DOC(dai, EventsManager, sendEvent))
        .def("sendSnap",
             &utility::EventsManager::sendSnap,
             py::arg("name"),
             py::arg("imgFrame").none(true) = nullptr,
             py::arg("data") = std::vector<std::shared_ptr<utility::EventData>>(),
             py::arg("tags") = std::vector<std::string>(),
             py::arg("extraData") = std::unordered_map<std::string, std::string>(),
             py::arg("deviceSerialNo") = "",
             DOC(dai, EventsManager, sendSnap));
}
