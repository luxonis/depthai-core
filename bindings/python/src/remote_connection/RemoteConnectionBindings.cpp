#include "RemoteConnectionBindings.hpp"

#include <pybind11/attr.h>
#include <pybind11/cast.h>
#include <pybind11/functional.h>
#include <pybind11/gil.h>

#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"

void RemoteConnectionBindings::bind(pybind11::module& m, void* pCallstack) {
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
#ifdef DEPTHAI_ENABLE_REMOTE_CONNECTION
    py::class_<RemoteConnection>(m, "RemoteConnection")
        .def(py::init<const std::string&, uint16_t, bool, uint16_t>(),
             py::arg("address") = RemoteConnection::DEFAULT_ADDRESS,
             py::arg("webSocketPort") = RemoteConnection::DEFAULT_WEBSOCKET_PORT,
             py::arg("serveFrontend") = true,
             py::arg("httpPort") = RemoteConnection::DEFAULT_HTTP_PORT,
             DOC(dai, RemoteConnection, RemoteConnection))
        .def("addTopic",
             py::overload_cast<const std::string&, dai::Node::Output&, const std::string&, bool>(&RemoteConnection::addTopic),
             py::arg("topicName"),
             py::arg("output"),
             py::arg("group") = "",
             py::arg("useVisualizationIfAvailable") = true,
             DOC(dai, RemoteConnection, addTopic))
        .def("addTopic",
             py::overload_cast<const std::string&, const std::string&, unsigned int, bool, bool>(&RemoteConnection::addTopic),
             py::arg("topicName"),
             py::arg("group") = "",
             py::arg("maxSize") = 16,
             py::arg("blocking") = false,
             py::arg("useVisualizationIfAvailable") = true,
             DOC(dai, RemoteConnection, addTopic))
        // Release the GIL
        .def("removeTopic",
             &RemoteConnection::removeTopic,
             py::arg("topicName"),
             py::call_guard<py::gil_scoped_release>(),
             DOC(dai, RemoteConnection, removeTopic))
        .def("registerPipeline", &RemoteConnection::registerPipeline, py::arg("pipeline"), DOC(dai, RemoteConnection, registerPipeline))
        .def("registerService", &RemoteConnection::registerService, py::arg("serviceName"), py::arg("callback"), DOC(dai, RemoteConnection, registerService))
        .def(
            "registerBinaryService",
            [](RemoteConnection& self, const std::string& serviceName, py::object callback) {
                self.registerBinaryService(serviceName, [callback](const std::vector<uint8_t>& data) -> std::vector<uint8_t> {
                    py::gil_scoped_acquire acquire;
                    py::object result = callback(py::bytes(reinterpret_cast<const char*>(data.data()), data.size()));
                    py::buffer_info buf(py::buffer(result).request());
                    return std::vector<uint8_t>(static_cast<uint8_t*>(buf.ptr), static_cast<uint8_t*>(buf.ptr) + buf.size);
                });
            },
            py::arg("serviceName"),
            py::arg("callback"),
            DOC(dai, RemoteConnection, registerBinaryService))
        .def("waitKey", &RemoteConnection::waitKey, py::arg("delay"), py::call_guard<py::gil_scoped_release>(), DOC(dai, RemoteConnection, waitKey));
#else
    // Define a placeholder class for RemoteConnection
    struct RemoteConnectionPlaceholder {
        RemoteConnectionPlaceholder(const std::string& /*address*/, uint16_t /*port*/, bool /*serveFrontend*/, uint16_t /*httpPort*/) {
            throw std::runtime_error("Remote connection is not enabled in this build.");
        }
    };
    py::class_<RemoteConnectionPlaceholder>(m, "RemoteConnection")
        .def(py::init<const std::string&, uint16_t, bool, uint16_t>(),
             py::arg("address") = "",
             py::arg("port") = 0,
             py::arg("serveFrontend") = true,
             py::arg("httpPort") = 0);
#endif
}