#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_objecttrackerconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawPointCloudConfig, RawBuffer, std::shared_ptr<RawPointCloudConfig>> rawConfig(m, "RawPointCloudConfig", DOC(dai, RawPointCloudConfig));
    py::class_<ObjectTrackerConfig, Py<ObjectTrackerConfig>, Buffer, std::shared_ptr<ObjectTrackerConfig>> config(m, "ObjectTrackerConfig", DOC(dai, ObjectTrackerConfig));

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

    // // Metadata / raw
    // rawConfig
    //     .def(py::init<>())
    //     .def_readwrite("sparse", &RawPointCloudConfig::sparse, DOC(dai, RawPointCloudConfig, sparse))
    //     .def_readwrite("transformationMatrix", &RawPointCloudConfig::transformationMatrix, DOC(dai, RawPointCloudConfig, transformationMatrix))
    //     ;

    // Message
    config.def(py::init<>())
        .def("__repr__", &ObjectTrackerConfig::str)

        // .def("set", &ObjectTrackerConfig::set, py::arg("config"), DOC(dai, ObjectTrackerConfig, set))
        // .def("get", &ObjectTrackerConfig::get, DOC(dai, ObjectTrackerConfig, get))
        .def("forceRemoveID", &ObjectTrackerConfig::forceRemoveID, DOC(dai, ObjectTrackerConfig, forceRemoveID))
        .def("forceRemoveIDs", &ObjectTrackerConfig::forceRemoveIDs, DOC(dai, ObjectTrackerConfig, forceRemoveIDs))
    ;
    // add aliases
}
