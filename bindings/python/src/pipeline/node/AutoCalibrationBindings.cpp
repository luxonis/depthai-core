#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AutoCalibration.hpp"

void bind_auto_calibration(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    namespace py = pybind11;

    // Node and Properties declare upfront
    py::class_<AutoCalibrationProperties> AutoCalibrationProperties(m, "AutoCalibrationProperties", DOC(dai, AutoCalibrationProperties));

    auto dynamicCalibrationWorker = ADD_NODE(AutoCalibration);

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
    dynamicCalibrationWorker.def("build", &AutoCalibration::build, py::arg("cameraLeft"), py::arg("cameraRight"), DOC(dai, node, AutoCalibration, build))
        .def_readonly("output", &AutoCalibration::output, DOC(dai, node, AutoCalibration, output))
        .def_property(
            "initialConfig",
            [](AutoCalibration& self) -> std::shared_ptr<AutoCalibrationConfig> {
                // Explicitly returning the shared_ptr member
                return self.initialConfig;
            },
            [](AutoCalibration& self, std::shared_ptr<AutoCalibrationConfig> cfg) {
                // Assigning the shared_ptr directly
                self.initialConfig = cfg;
            },
            DOC(dai, node, AutoCalibration, initialConfig));
}
