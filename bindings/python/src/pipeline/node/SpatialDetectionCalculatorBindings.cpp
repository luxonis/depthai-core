#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SpatialDetectionCalculator.hpp"
#include "depthai/properties/SpatialDetectionCalculatorProperties.hpp"

void bind_spatialdetectioncalculator(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<SpatialDetectionCalculatorProperties> spatialDetectionCalculatorProperties(
        m, "SpatialDetectionCalculatorProperties", DOC(dai, SpatialDetectionCalculatorProperties));
    auto spatialDetectionCalculator = ADD_NODE(SpatialDetectionCalculator);

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

    // Properties

    spatialDetectionCalculatorProperties.def_readwrite("lowerThreshold", &SpatialDetectionCalculatorProperties::lowerThreshold);
    spatialDetectionCalculatorProperties.def_readwrite("upperThreshold", &SpatialDetectionCalculatorProperties::upperThreshold);
    spatialDetectionCalculatorProperties.def_readwrite("calculationAlgorithm", &SpatialDetectionCalculatorProperties::calculationAlgorithm);
    spatialDetectionCalculatorProperties.def_readwrite("measurementModel", &SpatialDetectionCalculatorProperties::measurementModel);
    spatialDetectionCalculatorProperties.def_readwrite("stepSize", &SpatialDetectionCalculatorProperties::stepSize);

    // Node
    spatialDetectionCalculator
        .def_readonly("inputDetections", &SpatialDetectionCalculator::inputDetections, DOC(dai, node, SpatialDetectionCalculator, inputDetections))
        .def_readonly("inputDepth", &SpatialDetectionCalculator::inputDepth, DOC(dai, node, SpatialDetectionCalculator, inputDepth))
        .def_readonly("out", &SpatialDetectionCalculator::out, DOC(dai, node, SpatialDetectionCalculator, out))
        .def_readonly("passthroughDepth", &SpatialDetectionCalculator::passthroughDepth, DOC(dai, node, SpatialDetectionCalculator, passthroughDepth))
        .def("setDepthThresholds",
             &SpatialDetectionCalculator::setDepthThresholds,
             py::arg("lowerThreshold") = 0,
             py::arg("upperThreshold") = 65535,
             DOC(dai, node, SpatialDetectionCalculator, setDepthThresholds))
        .def("setCalculationAlgorithm",
             &SpatialDetectionCalculator::setCalculationAlgorithm,
             py::arg("calculationAlgorithm") = SpatialDetectionCalculatorAlgorithm::AVERAGE,
             DOC(dai, node, SpatialDetectionCalculator, setCalculationAlgorithm))
        .def("setMeasurementModel",
             &SpatialDetectionCalculator::setMeasurementModel,
             py::arg("measurementModel") = SpatialDetectionsMeasurementModes::DETAILED,
             DOC(dai, node, SpatialDetectionCalculator, setMeasurementModel))
        .def("setStepSize",
             &SpatialDetectionCalculator::setStepSize,
             py::arg("stepSize") = SpatialDetectionCalculatorProperties::AUTO,
             DOC(dai, node, SpatialDetectionCalculator, setStepSize));
    // ALIAS
    daiNodeModule.attr("SpatialDetectionCalculator").attr("Properties") = SpatialDetectionCalculatorProperties;
}
