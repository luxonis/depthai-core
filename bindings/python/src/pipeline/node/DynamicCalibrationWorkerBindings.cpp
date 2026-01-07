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
    dynamicCalibrationWorker.def_property_readonly(
        "syncedInput", [](DynamicCalibrationWorker& node) { return &node.syncedInput; }, py::return_value_policy::reference_internal);
}
