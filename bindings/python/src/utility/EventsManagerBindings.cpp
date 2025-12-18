#include "EventsManagerBindings.hpp"

// depthai
#ifdef DEPTHAI_ENABLE_EVENTS_MANAGER
    #include "depthai/utility/EventsManager.hpp"
#endif

void EventsManagerBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::enum_<dai::utility::SendSnapCallbackStatus> sendSnapCallbackStatus(m, "SendSnapCallbackStatus", DOC(dai, SendSnapCallbackStatus));

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
    py::class_<FileGroup, std::shared_ptr<utility::FileGroup>>(m, "FileGroup")
        .def(py::init<>())
        .def("addFile",
             static_cast<void (FileGroup::*)(std::string, std::string, std::string)>(&FileGroup::addFile),
             py::arg("fileName"),
             py::arg("data"),
             py::arg("mimeType"),
             DOC(dai, utility, FileGroup, addFile))
        .def("addFile",
             static_cast<void (FileGroup::*)(std::string, std::filesystem::path)>(&FileGroup::addFile),
             py::arg("fileName"),
             py::arg("filePath"),
             DOC(dai, utility, FileGroup, addFile))
        .def("addFile",
             static_cast<void (FileGroup::*)(const std::optional<std::string>&, const std::shared_ptr<ImgFrame>&)>(&FileGroup::addFile),
             py::arg("fileName"),
             py::arg("imgFrame"),
             DOC(dai, utility, FileGroup, addFile))
        .def("addFile",
             static_cast<void (FileGroup::*)(const std::optional<std::string>&, const std::shared_ptr<EncodedFrame>&)>(&FileGroup::addFile),
             py::arg("fileName"),
             py::arg("encodedFrame"),
             DOC(dai, utility, FileGroup, addFile))
        //.def("addFile",
        //     static_cast<void (FileGroup::*)(std::string, const std::shared_ptr<NNData>&)>(&FileGroup::addFile),
        //     py::arg("fileName"),
        //     py::arg("nnData"),
        //     DOC(dai, utility, FileGroup, addFile))
        .def("addFile",
             static_cast<void (FileGroup::*)(const std::optional<std::string>&, const std::shared_ptr<ImgDetections>&)>(&FileGroup::addFile),
             py::arg("fileName"),
             py::arg("imgDetections"),
             DOC(dai, utility, FileGroup, addFile))
        .def("addImageDetectionsPair",
             static_cast<void (FileGroup::*)(const std::optional<std::string>&, const std::shared_ptr<ImgFrame>&, const std::shared_ptr<ImgDetections>&)>(
                 &FileGroup::addImageDetectionsPair),
             py::arg("fileName"),
             py::arg("imgFrame"),
             py::arg("imgDetections"),
             DOC(dai, utility, FileGroup, addImageDetectionsPair))
        .def("addImageDetectionsPair",
             static_cast<void (FileGroup::*)(const std::optional<std::string>&, const std::shared_ptr<EncodedFrame>&, const std::shared_ptr<ImgDetections>&)>(
                 &FileGroup::addImageDetectionsPair),
             py::arg("fileName"),
             py::arg("encodedFrame"),
             py::arg("imgDetections"),
             DOC(dai, utility, FileGroup, addImageDetectionsPair));
    //.def("addImageNNDataPair",
    //     static_cast<void (FileGroup::*)(std::string, const std::shared_ptr<ImgFrame>&, const std::shared_ptr<NNData>&)>(&FileGroup::addImageNNDataPair),
    //     py::arg("fileName"),
    //     py::arg("imgFrame"),
    //     py::arg("nnData"),
    //     DOC(dai, utility, FileGroup, addImageNNDataPair))
    //.def(
    //    "addImageNNDataPair",
    //    static_cast<void (FileGroup::*)(std::string, const std::shared_ptr<EncodedFrame>&, const std::shared_ptr<NNData>&)>(&FileGroup::addImageNNDataPair),
    //    py::arg("fileName"),
    //    py::arg("encodedFrame"),
    //    py::arg("nnData"),
    //    DOC(dai, utility, FileGroup, addImageNNDataPair));

    sendSnapCallbackStatus.value("SUCCESS", SendSnapCallbackStatus::SUCCESS)
        .value("FILE_BATCH_PREPARATION_FAILED", SendSnapCallbackStatus::FILE_BATCH_PREPARATION_FAILED)
        .value("GROUP_CONTAINS_REJECTED_FILES", SendSnapCallbackStatus::GROUP_CONTAINS_REJECTED_FILES)
        .value("FILE_UPLOAD_FAILED", SendSnapCallbackStatus::FILE_UPLOAD_FAILED)
        .value("SEND_EVENT_FAILED", SendSnapCallbackStatus::SEND_EVENT_FAILED)
        .value("EVENT_REJECTED", SendSnapCallbackStatus::EVENT_REJECTED);

    py::class_<SendSnapCallbackResult>(m, "SendSnapCallbackResult")
        .def(py::init<>())
        .def_readonly("snapName", &SendSnapCallbackResult::snapName)
        .def_readonly("snapTimestamp", &SendSnapCallbackResult::snapTimestamp)
        .def_readonly("snapID", &SendSnapCallbackResult::snapID)
        .def_readonly("snapPayload", &SendSnapCallbackResult::snapPayload)
        .def_readonly("uploadStatus", &SendSnapCallbackResult::uploadStatus);

    py::class_<EventsManager>(m, "EventsManager")
        .def(py::init<>())
        .def(py::init<bool>(), py::arg("uploadCachedOnStart") = false)
        .def("setToken", &EventsManager::setToken, py::arg("token"), DOC(dai, utility, EventsManager, setToken))
        .def("setLogResponse", &EventsManager::setLogResponse, py::arg("logResponse"), DOC(dai, utility, EventsManager, setLogResponse))
        .def("setVerifySsl", &EventsManager::setVerifySsl, py::arg("verifySsl"), DOC(dai, utility, EventsManager, setVerifySsl))
        .def("setCacheDir", &EventsManager::setCacheDir, py::arg("cacheDir"), DOC(dai, utility, EventsManager, setCacheDir))
        .def("setCacheIfCannotSend",
             &EventsManager::setCacheIfCannotSend,
             py::arg("cacheIfCannotUpload"),
             DOC(dai, utility, EventsManager, setCacheIfCannotSend))
        .def("sendEvent",
             &EventsManager::sendEvent,
             py::arg("name"),
             py::arg("tags") = std::vector<std::string>(),
             py::arg("extras") = std::unordered_map<std::string, std::string>(),
             py::arg("associateFiles") = std::vector<std::string>(),
             DOC(dai, utility, EventsManager, sendEvent))
        .def("sendSnap",
             static_cast<bool (EventsManager::*)(const std::string&,
                                                 const std::shared_ptr<FileGroup>,
                                                 const std::vector<std::string>&,
                                                 const std::unordered_map<std::string, std::string>&,
                                                 const std::function<void(SendSnapCallbackResult)> successCallback,
                                                 const std::function<void(SendSnapCallbackResult)> failureCallback)>(&EventsManager::sendSnap),
             py::arg("name"),
             py::arg("fileGroup") = std::shared_ptr<FileGroup>(),
             py::arg("tags") = std::vector<std::string>(),
             py::arg("extras") = std::unordered_map<std::string, std::string>(),
             py::arg("successCallback") = py::none(),
             py::arg("failureCallback") = py::none(),
             DOC(dai, utility, EventsManager, sendSnap))
        .def("sendSnap",
             static_cast<bool (EventsManager::*)(const std::string&,
                                                 const std::optional<std::string>&,
                                                 const std::shared_ptr<ImgFrame>,
                                                 const std::optional<std::shared_ptr<ImgDetections>>&,
                                                 const std::vector<std::string>&,
                                                 const std::unordered_map<std::string, std::string>&,
                                                 const std::function<void(SendSnapCallbackResult)> successCallback,
                                                 const std::function<void(SendSnapCallbackResult)> failureCallback)>(&EventsManager::sendSnap),
             py::arg("name"),
             py::arg("fileName"),
             py::arg("imgFrame"),
             py::arg("imgDetections"),
             py::arg("tags") = std::vector<std::string>(),
             py::arg("extras") = std::unordered_map<std::string, std::string>(),
             py::arg("successCallback") = py::none(),
             py::arg("failureCallback") = py::none(),
             DOC(dai, utility, EventsManager, sendSnap));
#endif
}
