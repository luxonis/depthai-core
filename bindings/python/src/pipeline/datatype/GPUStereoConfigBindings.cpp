#include <memory>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_gpustereoconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<GPUStereoConfig, Py<GPUStereoConfig>, Buffer, std::shared_ptr<GPUStereoConfig>> gpuStereoConfig(m, "GPUStereoConfig", DOC(dai, GPUStereoConfig));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    gpuStereoConfig.def(py::init<>()).def_readwrite("confidenceThreshold", &GPUStereoConfig::confidenceThreshold);
}
