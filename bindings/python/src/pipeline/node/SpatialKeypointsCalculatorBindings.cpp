#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SpatialKeypointsCalculator.hpp"
#include "depthai/properties/SpatialKeypointsCalculatorProperties.hpp"

void bind_spatialkeypointscalculator(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::enum_<SpatialKeypointsCalculatorAlgorithm> spatialKeypointsCalculatorAlgorithm(
        m, "SpatialKeypointsCalculatorAlgorithm", DOC(dai, SpatialKeypointsCalculatorAlgorithm));
    py::enum_<SpatialKeypointsMeasurementModes> spatialKeypointsMeasurementModes(
        m, "SpatialKeypointsMeasurementModes", DOC(dai, SpatialKeypointsMeasurementModes));
    // Node and Properties declare upfront
    py::class_<SpatialKeypointsCalculatorProperties> spatialKeypointsCalculatorProperties(
        m, "SpatialKeypointsCalculatorProperties", DOC(dai, SpatialKeypointsCalculatorProperties));
    auto spatialKeypointsCalculator = ADD_NODE(SpatialKeypointsCalculator);

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

    // Metadata / raw
    spatialKeypointsCalculatorAlgorithm.value("AVERAGE", SpatialKeypointsCalculatorAlgorithm::AVERAGE)
        .value("MEAN", SpatialKeypointsCalculatorAlgorithm::MEAN)
        .value("MIN", SpatialKeypointsCalculatorAlgorithm::MIN)
        .value("MAX", SpatialKeypointsCalculatorAlgorithm::MAX)
        .value("MODE", SpatialKeypointsCalculatorAlgorithm::MODE)
        .value("MEDIAN", SpatialKeypointsCalculatorAlgorithm::MEDIAN);

    spatialKeypointsMeasurementModes.value("DETAILED", SpatialKeypointsMeasurementModes::DETAILED)
        .value("SEGMENTATION", SpatialKeypointsMeasurementModes::SEGMENTATION)
        .value("ELIPTICAL", SpatialKeypointsMeasurementModes::ELIPTICAL)
        .value("RECTANGLE", SpatialKeypointsMeasurementModes::RECTANGLE);

    // Properties

    spatialKeypointsCalculatorProperties.def_readwrite("lowerThreshold", &SpatialKeypointsCalculatorProperties::lowerThreshold);
    spatialKeypointsCalculatorProperties.def_readwrite("upperThreshold", &SpatialKeypointsCalculatorProperties::upperThreshold);
    spatialKeypointsCalculatorProperties.def_readwrite("calculationAlgorithm", &SpatialKeypointsCalculatorProperties::calculationAlgorithm);
    spatialKeypointsCalculatorProperties.def_readwrite("measurementModel", &SpatialKeypointsCalculatorProperties::measurementModel);
    spatialKeypointsCalculatorProperties.def_readwrite("stepSize", &SpatialKeypointsCalculatorProperties::stepSize);

    // Node
    spatialKeypointsCalculator
        .def_readonly("inputKeypoints", &SpatialKeypointsCalculator::inputKeypoints, DOC(dai, node, SpatialKeypointsCalculator, inputKeypoints))
        .def_readonly("inputDepth", &SpatialKeypointsCalculator::inputDepth, DOC(dai, node, SpatialKeypointsCalculator, inputDepth))
        .def_readonly("out", &SpatialKeypointsCalculator::out, DOC(dai, node, SpatialKeypointsCalculator, out))
        .def_readonly("passthroughDepth", &SpatialKeypointsCalculator::passthroughDepth, DOC(dai, node, SpatialKeypointsCalculator, passthroughDepth))
        .def("setDepthThresholds",
             &SpatialKeypointsCalculator::setDepthThresholds,
             py::arg("lowerThreshold") = 0,
             py::arg("upperThreshold") = 65535,
             DOC(dai, node, SpatialKeypointsCalculator, setDepthThresholds))
        .def("setCalculationAlgorithm",
             &SpatialKeypointsCalculator::setCalculationAlgorithm,
             py::arg("calculationAlgorithm") = SpatialKeypointsCalculatorAlgorithm::AVERAGE,
             DOC(dai, node, SpatialKeypointsCalculator, setCalculationAlgorithm))
        .def("setMeasurementModel",
             &SpatialKeypointsCalculator::setMeasurementModel,
             py::arg("measurementModel") = SpatialKeypointsMeasurementModes::DETAILED,
             DOC(dai, node, SpatialKeypointsCalculator, setMeasurementModel))
        .def("setStepSize",
             &SpatialKeypointsCalculator::setStepSize,
             py::arg("stepSize") = SpatialKeypointsCalculatorProperties::AUTO,
             DOC(dai, node, SpatialKeypointsCalculator, setStepSize));
    // ALIAS
    daiNodeModule.attr("SpatialKeypointsCalculator").attr("Properties") = spatialKeypointsCalculatorProperties;
}
