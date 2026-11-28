#include "Script.hpp"

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

#include <chrono>
#include <iostream>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Script.hpp"

namespace py = pybind11;
extern "C" void depthai_bindings_init();

namespace dai {
namespace gate {

static std::atomic<bool> cpythonInitialized = false;

py::dict copyPyDict(const py::dict& original) {
    py::dict copy;
    for(auto item : original) {
        copy[item.first] = item.second;
    }
    return copy;
}

void Script::buildStage1() {
    if(!cpythonInitialized.exchange(true)) {
        py::initialize_interpreter();
        depthai_bindings_init();

        {
            auto depthaiModule = py::module_::import("depthai");
            depthaiModule.attr("__device_id__") = std::string("serialtodo");
            depthaiModule.attr("__version__") = std::string(DEPTHAI_DEVICE_KB_FINAL_VERSION);
            depthaiModule.attr("__device_rvc4_version__") = std::string(DEPTHAI_DEVICE_KB_FINAL_VERSION);

            auto globals = py::globals();  // Scoped: these objects must be released before the GIL is released for the rest of the process

            globals.attr("update")(depthaiModule.attr("__dict__"));
        }

        py::gil_scoped_release guard{};
        guard.disarm();
    }
}

void Script::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");
    py::gil_scoped_acquire guard{};

    auto globalsScript = copyPyDict(py::globals());
    globalsScript["node"] = std::static_pointer_cast<::dai::node::Script>(shared_from_this());

    auto script = loadResource(properties.scriptUri);
    std::string buffer(reinterpret_cast<const char*>(script.data()), script.size());
    auto expr = (script.data()[0] == '\n') ? py::str(py::module_::import("textwrap").attr("dedent")(buffer)) : py::str(buffer);

    try {
        py::exec(expr, globalsScript, globalsScript);
    } catch(const py::error_already_set& e) {
        logger->error("Script threw an exception: {}", e.what());
    }
}

}  // namespace gate
}  // namespace dai
