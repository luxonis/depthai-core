#include "RemoteConnectorBindings.hpp"

#include <pybind11/functional.h>

#include "depthai/remote_connection/RemoteConnector.hpp"
#include "depthai/pipeline/InputQueue.hpp"

void RemoteConnectorBindings::bind(pybind11::module& m, void* pCallstack) {
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

    py::class_<RemoteConnector>(m, "RemoteConnector")
        .def(py::init<const std::string&, uint16_t>(), py::arg("address") = "0.0.0.0", py::arg("port") = 8765)
        .def("addTopic",
             py::overload_cast<const std::string&, dai::Node::Output&, const std::string&>(&RemoteConnector::addTopic),
             py::arg("topicName"),
             py::arg("output"),
             py::arg("group") = "")
        .def("addTopic",
             py::overload_cast<const std::string&, const std::string&, unsigned int, bool>(&RemoteConnector::addTopic),
             py::arg("topicName"),
             py::arg("group") = "",
             py::arg("maxSize") = 16,
             py::arg("blocking") = false)
        .def("registerPipeline", &RemoteConnector::registerPipeline, py::arg("pipeline"))
        .def("waitKey", &RemoteConnector::waitKey, py::arg("delay"));
}
