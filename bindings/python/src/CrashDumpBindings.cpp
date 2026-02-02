#include "CrashDumpBindings.hpp"

// depthai
#include "depthai/device/CrashDump.hpp"
#include "depthai/device/CrashDumpManager.hpp"
#include "depthai/device/DeviceBase.hpp"

// pybind11_json
#include "pybind11_json/pybind11_json.hpp"

// std
#include <filesystem>

// Helpers to allow users to work with the `extra` attribute as if it was a regular python dict
// The gist of this is to use an extra python dictionary that gets synced with the C++ extra json
// dict whenever it's needed. This is needed as otherwise the `extra` json dict generally is returned
// as a copy (and not a reference)
static void syncExtraToNative(py::handle self, dai::CrashDump& dump) {
    if(py::hasattr(self, "_extra_dict")) {
        dump.extra = pyjson::to_json(self.attr("_extra_dict"));
    }
}

static py::dict getExtraDict(py::handle self) {
    if(!py::hasattr(self, "_extra_dict")) {
        auto& dump = self.cast<dai::CrashDump&>();
        self.attr("_extra_dict") = pyjson::from_json(dump.extra);
    }
    return self.attr("_extra_dict");
}

void CrashDumpBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    // Base CrashDump class
    py::class_<CrashDump, std::shared_ptr<CrashDump>> crashDump(m, "CrashDump", DOC(dai, CrashDump), py::dynamic_attr());

    // CrashDumpRVC2 and its nested types
    py::class_<CrashDumpRVC2, CrashDump, std::shared_ptr<CrashDumpRVC2>> crashDumpRVC2(m, "CrashDumpRVC2", DOC(dai, CrashDumpRVC2));
    py::class_<CrashDumpRVC2::CrashReportCollection> crashReportCollection(crashDumpRVC2, "CrashReportCollection", DOC(dai, CrashDumpRVC2, CrashReportCollection));
    py::class_<CrashDumpRVC2::CrashReport> crashReport(crashDumpRVC2, "CrashReport", DOC(dai, CrashDumpRVC2, CrashReport));
    py::class_<CrashDumpRVC2::CrashReport::ErrorSourceInfo> errorSourceInfo(crashReport, "ErrorSourceInfo", DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo));
    py::class_<CrashDumpRVC2::CrashReport::ErrorSourceInfo::AssertContext> assertContext(
        errorSourceInfo, "AssertContext", DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, AssertContext));
    py::class_<CrashDumpRVC2::CrashReport::ErrorSourceInfo::TrapContext> trapContext(
        errorSourceInfo, "TrapContext", DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, TrapContext));
    py::class_<CrashDumpRVC2::CrashReport::ThreadCallstack> threadCallstack(crashReport, "ThreadCallstack", DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack));
    py::class_<CrashDumpRVC2::CrashReport::ThreadCallstack::CallstackContext> callstackContext(
        threadCallstack, "CallstackContext", DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, CallstackContext));

    // CrashDumpRVC4
    py::class_<CrashDumpRVC4, CrashDump, std::shared_ptr<CrashDumpRVC4>> crashDumpRVC4(m, "CrashDumpRVC4", DOC(dai, CrashDumpRVC4));

    // CrashDumpManager
    py::class_<CrashDumpManager> crashDumpManager(m, "CrashDumpManager", DOC(dai, CrashDumpManager));

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

    // Bind CrashDump (base class - abstract)
    crashDump.def("getPlatform", &CrashDump::getPlatform, DOC(dai, CrashDump, getPlatform))
        .def("getCrashDumpVersion", &CrashDump::getCrashDumpVersion, DOC(dai, CrashDump, getCrashDumpVersion))
        .def(
            "toTar",
            [](py::object self, const std::filesystem::path& tarPath) {
                auto& dump = self.cast<CrashDump&>();
                syncExtraToNative(self, dump);
                dump.toTar(tarPath);
            },
            py::arg("tarPath"),
            DOC(dai, CrashDump, toTar))
        .def(
            "fromTar",
            [](py::object self, const std::filesystem::path& tarPath) {
                auto& dump = self.cast<CrashDump&>();
                dump.fromTar(tarPath);
                // Invalidate cached dict so it gets lazily re-synced from C++ on next access
                if(py::hasattr(self, "_extra_dict")) py::delattr(self, "_extra_dict");
            },
            py::arg("tarPath"),
            DOC(dai, CrashDump, fromTar))
        .def(
            "toBytes",
            [](py::object self) -> py::bytes {
                auto& dump = self.cast<CrashDump&>();
                syncExtraToNative(self, dump);
                auto bytes = dump.toBytes();
                return py::bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size());
            },
            DOC(dai, CrashDump, toBytes))
        .def_static(
            "fromBytes",
            [](const std::vector<uint8_t>& bytes) -> std::shared_ptr<CrashDump> { return CrashDump::fromBytes(bytes); },
            py::arg("bytes"),
            DOC(dai, CrashDump, fromBytes))
        .def_static(
            "load",
            [](const std::filesystem::path& tarPath) -> std::shared_ptr<CrashDump> { return CrashDump::load(tarPath); },
            py::arg("tarPath"),
            DOC(dai, CrashDump, load))
        .def(
            "__getitem__",
            [](py::object self, const std::string& key) -> py::object {
                auto d = getExtraDict(self);
                if(!d.contains(key)) throw py::key_error(key);
                return d[py::str(key)];
            },
            py::arg("key"))
        .def(
            "__setitem__",
            [](py::object self, const std::string& key, py::handle value) {
                getExtraDict(self)[py::str(key)] = value;
                auto& dump = self.cast<CrashDump&>();
                syncExtraToNative(self, dump);
            },
            py::arg("key"),
            py::arg("value"))
        .def(
            "__contains__", [](py::object self, const std::string& key) -> bool { return getExtraDict(self).contains(key); }, py::arg("key"))
        .def(
            "__delitem__",
            [](py::object self, const std::string& key) {
                auto d = getExtraDict(self);
                if(!d.contains(key)) throw py::key_error(key);
                PyDict_DelItemString(d.ptr(), key.c_str());
                auto& dump = self.cast<CrashDump&>();
                syncExtraToNative(self, dump);
            },
            py::arg("key"))
        .def("__len__", [](py::object self) -> size_t { return py::len(getExtraDict(self)); })
        .def_property(
            "extra",
            [](py::object self) -> py::dict { return getExtraDict(self); },
            [](py::object self, py::dict value) {
                self.attr("_extra_dict") = value;
                auto& dump = self.cast<CrashDump&>();
                syncExtraToNative(self, dump);
            })
        // Public members
        .def_readwrite("depthaiVersion", &CrashDump::depthaiVersion)
        .def_readwrite("depthaiVersionMajor", &CrashDump::depthaiVersionMajor)
        .def_readwrite("depthaiVersionMinor", &CrashDump::depthaiVersionMinor)
        .def_readwrite("depthaiVersionPatch", &CrashDump::depthaiVersionPatch)
        .def_readwrite("depthaiVersionPreReleaseType", &CrashDump::depthaiVersionPreReleaseType)
        .def_readwrite("depthaiVersionPreReleaseVersion", &CrashDump::depthaiVersionPreReleaseVersion)
        .def_readwrite("depthaiVersionBuildInfo", &CrashDump::depthaiVersionBuildInfo)
        .def_readwrite("depthaiCommitHash", &CrashDump::depthaiCommitHash)
        .def_readwrite("depthaiCommitDatetime", &CrashDump::depthaiCommitDatetime)
        .def_readwrite("depthaiBuildDatetime", &CrashDump::depthaiBuildDatetime)
        .def_readwrite("depthaiDeviceVersion", &CrashDump::depthaiDeviceVersion)
        .def_readwrite("depthaiBootloaderVersion", &CrashDump::depthaiBootloaderVersion)
        .def_readwrite("depthaiDeviceRVC3Version", &CrashDump::depthaiDeviceRVC3Version)
        .def_readwrite("depthaiDeviceRVC4Version", &CrashDump::depthaiDeviceRVC4Version)
        .def_readwrite("crashdumpTimestamp", &CrashDump::crashdumpTimestamp)
        .def_readwrite("deviceId", &CrashDump::deviceId)
        .def_readwrite("osPlatform", &CrashDump::osPlatform);

    // Bind CrashDumpRVC2
    crashDumpRVC2.def(py::init<>())
        .def(py::init<const std::filesystem::path&>(), py::arg("tarFile"))
        .def_readwrite("crashReports", &CrashDumpRVC2::crashReports);

    // Bind CrashDumpRVC2::CrashReportCollection
    crashReportCollection.def(py::init<>())
        .def_readwrite("crashReports", &CrashDumpRVC2::CrashReportCollection::crashReports, DOC(dai, CrashDumpRVC2, CrashReportCollection, crashReports))
        .def_readwrite("depthaiCommitHash", &CrashDumpRVC2::CrashReportCollection::depthaiCommitHash, DOC(dai, CrashDumpRVC2, CrashReportCollection, depthaiCommitHash))
        .def_readwrite("deviceId", &CrashDumpRVC2::CrashReportCollection::deviceId, DOC(dai, CrashDumpRVC2, CrashReportCollection, deviceId));

    // Bind CrashDumpRVC2::CrashReport
    crashReport.def(py::init<>())
        .def_readwrite("processor", &CrashDumpRVC2::CrashReport::processor, DOC(dai, CrashDumpRVC2, CrashReport, processor))
        .def_readwrite("errorSource", &CrashDumpRVC2::CrashReport::errorSource, DOC(dai, CrashDumpRVC2, CrashReport, errorSource))
        .def_readwrite("crashedThreadId", &CrashDumpRVC2::CrashReport::crashedThreadId, DOC(dai, CrashDumpRVC2, CrashReport, crashedThreadId))
        .def_readwrite("errorSourceInfo", &CrashDumpRVC2::CrashReport::errorSourceInfo, DOC(dai, CrashDumpRVC2, CrashReport, errorSourceInfo))
        .def_readwrite("threadCallstack", &CrashDumpRVC2::CrashReport::threadCallstack, DOC(dai, CrashDumpRVC2, CrashReport, threadCallstack));

    errorSourceInfo.def(py::init<>())
        .def_readwrite(
            "assertContext", &CrashDumpRVC2::CrashReport::ErrorSourceInfo::assertContext, DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, assertContext))
        .def_readwrite("trapContext", &CrashDumpRVC2::CrashReport::ErrorSourceInfo::trapContext, DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, trapContext))
        .def_readwrite("errorId", &CrashDumpRVC2::CrashReport::ErrorSourceInfo::errorId, DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, errorId));

    assertContext.def(py::init<>())
        .def_readwrite("fileName",
                       &CrashDumpRVC2::CrashReport::ErrorSourceInfo::AssertContext::fileName,
                       DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, AssertContext, fileName))
        .def_readwrite("functionName",
                       &CrashDumpRVC2::CrashReport::ErrorSourceInfo::AssertContext::functionName,
                       DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, AssertContext, functionName))
        .def_readwrite(
            "line", &CrashDumpRVC2::CrashReport::ErrorSourceInfo::AssertContext::line, DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, AssertContext, line));

    trapContext.def(py::init<>())
        .def_readwrite("trapNumber",
                       &CrashDumpRVC2::CrashReport::ErrorSourceInfo::TrapContext::trapNumber,
                       DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, TrapContext, trapNumber))
        .def_readwrite("trapAddress",
                       &CrashDumpRVC2::CrashReport::ErrorSourceInfo::TrapContext::trapAddress,
                       DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, TrapContext, trapAddress))
        .def_readwrite("trapName",
                       &CrashDumpRVC2::CrashReport::ErrorSourceInfo::TrapContext::trapName,
                       DOC(dai, CrashDumpRVC2, CrashReport, ErrorSourceInfo, TrapContext, trapName));

    threadCallstack.def(py::init<>())
        .def_readwrite("threadId", &CrashDumpRVC2::CrashReport::ThreadCallstack::threadId, DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, threadId))
        .def_readwrite("threadName", &CrashDumpRVC2::CrashReport::ThreadCallstack::threadName, DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, threadName))
        .def_readwrite("stackBottom", &CrashDumpRVC2::CrashReport::ThreadCallstack::stackBottom, DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, stackBottom))
        .def_readwrite("stackTop", &CrashDumpRVC2::CrashReport::ThreadCallstack::stackTop, DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, stackTop))
        .def_readwrite("stackPointer", &CrashDumpRVC2::CrashReport::ThreadCallstack::stackPointer, DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, stackPointer))
        .def_readwrite("instructionPointer",
                       &CrashDumpRVC2::CrashReport::ThreadCallstack::instructionPointer,
                       DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, instructionPointer))
        .def_readwrite("threadStatus", &CrashDumpRVC2::CrashReport::ThreadCallstack::threadStatus, DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, threadStatus))
        .def_readwrite("callStack", &CrashDumpRVC2::CrashReport::ThreadCallstack::callStack, DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, callStack));

    callstackContext.def(py::init<>())
        .def_readwrite("callSite",
                       &CrashDumpRVC2::CrashReport::ThreadCallstack::CallstackContext::callSite,
                       DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, CallstackContext, callSite))
        .def_readwrite("calledTarget",
                       &CrashDumpRVC2::CrashReport::ThreadCallstack::CallstackContext::calledTarget,
                       DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, CallstackContext, calledTarget))
        .def_readwrite("framePointer",
                       &CrashDumpRVC2::CrashReport::ThreadCallstack::CallstackContext::framePointer,
                       DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, CallstackContext, framePointer))
        .def_readwrite("context",
                       &CrashDumpRVC2::CrashReport::ThreadCallstack::CallstackContext::context,
                       DOC(dai, CrashDumpRVC2, CrashReport, ThreadCallstack, CallstackContext, context));

    // Bind CrashDumpRVC4
    crashDumpRVC4.def(py::init<>())
        .def(py::init<const std::filesystem::path&>(), py::arg("tarFile"))
        .def_readwrite("data", &CrashDumpRVC4::data)
        .def_readwrite("filename", &CrashDumpRVC4::filename);

    // Bind CrashDumpManager
    crashDumpManager.def(py::init<DeviceBase*>(), py::arg("device"), py::keep_alive<1, 2>(), DOC(dai, CrashDumpManager, CrashDumpManager))
        .def(
            "collectCrashDump",
            [](CrashDumpManager& self, bool clear) -> std::shared_ptr<CrashDump> {
                py::gil_scoped_release release;
                return self.collectCrashDump(clear);
            },
            py::arg("clear") = true,
            DOC(dai, CrashDumpManager, collectCrashDump))
        .def(
            "hasCrashDump",
            [](CrashDumpManager& self) {
                py::gil_scoped_release release;
                return self.hasCrashDump();
            },
            DOC(dai, CrashDumpManager, hasCrashDump));
}
