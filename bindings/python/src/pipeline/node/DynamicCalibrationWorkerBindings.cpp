#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DynamicCalibrationWorker.hpp"

void bind_dynamic_calibration_worker(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    namespace py = pybind11;

    // Node and Properties declare upfront
    py::class_<DynamicCalibrationWorkerProperties> DynamicCalibrationWorkerProperties(
        m, "DynamicCalibrationWorkerProperties", DOC(dai, DynamicCalibrationWorkerProperties));

    auto dynamicCalibrationWorker = ADD_NODE(DynamicCalibrationWorker);

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
    dynamicCalibrationWorker
        .def("build", &DynamicCalibrationWorker::build, py::arg("cameraLeft"), py::arg("cameraRight"), DOC(dai, node, DynamicCalibrationWorker, build))
        .def_readonly("output", &DynamicCalibrationWorker::output, DOC(dai, node, DynamicCalibrationWorker, output))
        .def_property(
            "initialConfig",
            [](DynamicCalibrationWorker& self) -> std::shared_ptr<DynamicCalibrationWorkerConfig> {
                // Explicitly returning the shared_ptr member
                return self.initialConfig;
            },
            [](DynamicCalibrationWorker& self, std::shared_ptr<DynamicCalibrationWorkerConfig> cfg) {
                // Assigning the shared_ptr directly
                self.initialConfig = cfg;
            },
            DOC(dai, node, DynamicCalibrationWorker, initialConfig));
}
