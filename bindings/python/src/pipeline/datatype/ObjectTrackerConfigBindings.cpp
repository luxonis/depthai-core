#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_objecttrackerconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ObjectTrackerConfig, Py<ObjectTrackerConfig>, Buffer, py::smart_holder> config(m, "ObjectTrackerConfig", DOC(dai, ObjectTrackerConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    config.def(py::init<>())
        .def_readwrite("trackletIdsToRemove", &dai::ObjectTrackerConfig::trackletIdsToRemove, DOC(dai, ObjectTrackerConfig, trackletIdsToRemove))
        .def("__repr__", &ObjectTrackerConfig::str)

        .def("forceRemoveID", &ObjectTrackerConfig::forceRemoveID, DOC(dai, ObjectTrackerConfig, forceRemoveID))
        .def("forceRemoveIDs", &ObjectTrackerConfig::forceRemoveIDs, DOC(dai, ObjectTrackerConfig, forceRemoveIDs));
}
