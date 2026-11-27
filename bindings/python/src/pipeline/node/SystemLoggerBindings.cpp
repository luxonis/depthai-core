#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"

void bind_systemlogger(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<SystemLoggerProperties> systemLoggerProperties(m, "SystemLoggerProperties", DOC(dai, SystemLoggerProperties));
    auto systemLogger = ADD_NODE(SystemLogger);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    systemLoggerProperties.def_readwrite("rateHz", &SystemLoggerProperties::rateHz);

    systemLogger.def_readonly("out", &SystemLogger::out, DOC(dai, node, SystemLogger, out))
        .def("setRate", &SystemLogger::setRate, py::arg("hz"), DOC(dai, node, SystemLogger, setRate))
        .def("getRate", &SystemLogger::getRate, DOC(dai, node, SystemLogger, getRate));
}
