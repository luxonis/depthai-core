#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/gil.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>

// depthai
#include "MessageQueueBindings.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"

py::object messageQueueException;  // Needed to be able to catch in C++ after it's raised on the Python side
namespace dai {
namespace mq_utils {

inline int addCallbackUnlocked(MessageQueue& queue, std::function<void(std::string, std::shared_ptr<ADatatype>)> callback) {
    // Get unique id
    int uniqueId = queue.uniqueCallbackId++;

    // Assign callback directly without lock
    queue.callbacks[uniqueId] = std::move(callback);

    return uniqueId;
}

inline int addCallbackUnlocked(MessageQueue& queue, const std::function<void(std::shared_ptr<ADatatype>)>& callback) {
    return addCallbackUnlocked(queue, [callback](const std::string&, std::shared_ptr<ADatatype> message) { callback(std::move(message)); });
}

inline int addCallbackUnlocked(MessageQueue& queue, const std::function<void()>& callback) {
    return addCallbackUnlocked(queue, [callback](const std::string&, std::shared_ptr<ADatatype>) { callback(); });
}

}  // namespace mq_utils
}  // namespace dai

void MessageQueueBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace std::chrono;

    // Type definitions
    py::class_<MessageQueue, std::shared_ptr<MessageQueue>> messageQueue(m, "MessageQueue", DOC(dai, MessageQueue));
    constexpr auto QUEUE_EXCEPTION_NAME = "QueueException";
    py::register_exception<dai::MessageQueue::QueueException>(messageQueue, QUEUE_EXCEPTION_NAME);
    messageQueueException = messageQueue.attr(QUEUE_EXCEPTION_NAME);

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

    // To prevent blocking whole python interpreter, blocking functions like 'get' and 'send'
    // are pooled with a reasonable delay and check for python interrupt signal in between.

    // Bind DataOutputQueue
    auto addCallbackLambda = [](MessageQueue& q, py::function cb) -> int {
        pybind11::module inspectModule = pybind11::module::import("inspect");
        pybind11::object result = inspectModule.attr("signature")(cb).attr("parameters");
        auto numParams = pybind11::len(result);
        // Unlock the GIL before acquiring the mutex to avoid deadlocks
        py::gil_scoped_release release;
        std::unique_lock<std::mutex> lock(q.callbacksMtx);
        pybind11::gil_scoped_acquire
            gil;  // Order is important! Only acquire after the mutex is locked, to allow any callbacks to capture the GIL and finish the execution

        if(numParams == 2) {
            return dai::mq_utils::addCallbackUnlocked(q, [cb](std::string msg, std::shared_ptr<ADatatype> data) {
                pybind11::gil_scoped_acquire gil;
                cb(msg, data);
            });
        } else if(numParams == 1) {
            return dai::mq_utils::addCallbackUnlocked(q, [cb](std::shared_ptr<ADatatype> data) {
                pybind11::gil_scoped_acquire gil;
                cb(data);
            });
        } else if(numParams == 0) {
            return dai::mq_utils::addCallbackUnlocked(q, [cb]() {
                pybind11::gil_scoped_acquire gil;
                cb();
            });
        } else {
            throw py::value_error("Callback must take either zero, one or two arguments");
        }
    };

    messageQueue.def(py::init<std::string>(), py::arg("name"), DOC(dai, MessageQueue, MessageQueue))
        .def(py::init<std::string, unsigned int, bool>(),
             py::arg("name") = "",
             py::arg("maxSize") = 16,
             py::arg("blocking") = true,
             DOC(dai, MessageQueue, MessageQueue, 2))
        .def("getName", &MessageQueue::getName, DOC(dai, MessageQueue, getName))
        .def("getName", &MessageQueue::getName, DOC(dai, MessageQueue, getName))
        .def("setName", &MessageQueue::setName, py::arg("name"), DOC(dai, MessageQueue, setName))
        .def("isClosed", &MessageQueue::isClosed, DOC(dai, MessageQueue, isClosed))
        .def("close", &MessageQueue::close, DOC(dai, MessageQueue, close))
        .def("setBlocking", &MessageQueue::setBlocking, py::arg("blocking"), DOC(dai, MessageQueue, setBlocking))
        .def("getBlocking", &MessageQueue::getBlocking, DOC(dai, MessageQueue, getBlocking))
        .def("setMaxSize", &MessageQueue::setMaxSize, py::arg("maxSize"), DOC(dai, MessageQueue, setMaxSize))
        .def("getMaxSize", &MessageQueue::getMaxSize, DOC(dai, MessageQueue, getMaxSize))
        .def("getSize", &MessageQueue::getSize, DOC(dai, MessageQueue, getSize))
        .def("isFull", &MessageQueue::isFull, DOC(dai, MessageQueue, isFull))
        .def("addCallback", addCallbackLambda, py::arg("callback"), DOC(dai, MessageQueue, addCallback))
        .def(
            "removeCallback",
            [](MessageQueue& q, int id) {
                // Unlock the GIL before acquiring the mutex to avoid deadlocks
                py::gil_scoped_release release;

                std::unique_lock<std::mutex> lock(q.callbacksMtx);
                pybind11::gil_scoped_acquire
                    gil;  // Order is important! Only acquire after the mutex is locked, to allow any callbacks to capture the GIL and finish the execution
                // If callback with id 'callbackId' doesn't exists, return false
                if(q.callbacks.count(id) == 0) return false;
                // Otherwise erase and return true
                q.callbacks.erase(id);
                return true;
            },
            py::arg("callbackId"),
            DOC(dai, MessageQueue, removeCallback))
        .def("has", static_cast<bool (MessageQueue::*)()>(&MessageQueue::has), DOC(dai, MessageQueue, has))
        .def("tryGet", static_cast<std::shared_ptr<ADatatype> (MessageQueue::*)()>(&MessageQueue::tryGet), DOC(dai, MessageQueue, tryGet))
        .def(
            "get",
            [](MessageQueue& obj) {
                std::shared_ptr<ADatatype> d = nullptr;
                bool timedout = true;
                do {
                    {
                        // releases python GIL
                        py::gil_scoped_release release;
                        // block for 100ms
                        d = obj.get(milliseconds(100), timedout);
                    }
                    // reacquires python GIL for PyErr_CheckSignals call
                    // check if interrupt triggered in between
                    if(PyErr_CheckSignals() != 0) throw py::error_already_set();
                } while(timedout);
                return d;
            },
            DOC(dai, MessageQueue, get))
        .def("front", static_cast<std::shared_ptr<ADatatype> (MessageQueue::*)()>(&MessageQueue::front), DOC(dai, MessageQueue, front))
        .def(
            "get",
            [](MessageQueue& obj, milliseconds timeout) {
                std::shared_ptr<ADatatype> d = nullptr;
                bool timedout = true;
                milliseconds timeoutLeft = timeout;
                while(timedout && timeoutLeft.count() > 0) {
                    {
                        auto toSleep = std::min(milliseconds(100), timeoutLeft);
                        py::gil_scoped_release release;
                        d = obj.get(toSleep, timedout);
                        timeoutLeft -= toSleep;
                    }
                    if(PyErr_CheckSignals() != 0) throw py::error_already_set();
                }
                if(PyErr_CheckSignals() != 0) throw py::error_already_set();
                return d;
            },
            py::arg("timeout"),
            DOC(dai, MessageQueue, get))
        .def("tryGetAll", static_cast<std::vector<std::shared_ptr<ADatatype>> (MessageQueue::*)()>(&MessageQueue::tryGetAll), DOC(dai, MessageQueue, tryGetAll))
        .def(
            "getAll",
            [](MessageQueue& obj) {
                std::vector<std::shared_ptr<ADatatype>> vec;
                bool timedout = true;
                do {
                    {
                        py::gil_scoped_release release;
                        vec = obj.getAll(milliseconds(100), timedout);
                    }
                    if(PyErr_CheckSignals() != 0) throw py::error_already_set();
                } while(timedout);
                return vec;
            },
            DOC(dai, MessageQueue, getAll))
        .def(
            "send",
            [](MessageQueue& obj, const std::shared_ptr<ADatatype>& msg) {
                bool sent = false;
                do {
                    {
                        py::gil_scoped_release release;
                        sent = obj.send(msg, milliseconds(100));
                    }
                    if(PyErr_CheckSignals() != 0) throw py::error_already_set();
                } while(!sent);
            },
            py::arg("msg"),
            DOC(dai, MessageQueue, send))
        .def(
            "send",
            [](MessageQueue& obj, const std::shared_ptr<ADatatype>& msg, milliseconds timeout) {
                bool sent = false;
                milliseconds timeoutLeft = timeout;
                while(!sent && timeoutLeft.count() > 0) {
                    {
                        auto toSleep = std::min(milliseconds(100), timeoutLeft);
                        py::gil_scoped_release release;
                        sent = obj.send(msg, toSleep);
                        timeoutLeft -= toSleep;
                    }
                    if(PyErr_CheckSignals() != 0) throw py::error_already_set();
                }
                return sent;
            },
            py::arg("msg"),
            py::arg("timeout"),
            DOC(dai, MessageQueue, send))
        .def("trySend", &MessageQueue::trySend, py::arg("msg"), DOC(dai, MessageQueue, trySend));
}