#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_spatiallocationcalculatorconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<SpatialLocationCalculatorConfigThresholds> spatialLocationCalculatorConfigThresholds(
        m, "SpatialLocationCalculatorConfigThresholds", DOC(dai, SpatialLocationCalculatorConfigThresholds));
    py::enum_<SpatialLocationCalculatorAlgorithm> spatialLocationCalculatorAlgorithm(
        m, "SpatialLocationCalculatorAlgorithm", DOC(dai, SpatialLocationCalculatorAlgorithm));
    py::class_<SpatialLocationCalculatorConfigData> spatialLocationCalculatorConfigData(
        m, "SpatialLocationCalculatorConfigData", DOC(dai, SpatialLocationCalculatorConfigData));
    py::class_<SpatialLocationCalculatorConfig, Py<SpatialLocationCalculatorConfig>, Buffer, std::shared_ptr<SpatialLocationCalculatorConfig>>
        spatialLocationCalculatorConfig(m, "SpatialLocationCalculatorConfig", DOC(dai, SpatialLocationCalculatorConfig));

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
    spatialLocationCalculatorConfigThresholds.def(py::init<>())
        .def_readwrite("lowerThreshold", &SpatialLocationCalculatorConfigThresholds::lowerThreshold)
        .def_readwrite("upperThreshold", &SpatialLocationCalculatorConfigThresholds::upperThreshold);

    spatialLocationCalculatorAlgorithm.value("AVERAGE", SpatialLocationCalculatorAlgorithm::AVERAGE)
        .value("MEAN", SpatialLocationCalculatorAlgorithm::MEAN)
        .value("MIN", SpatialLocationCalculatorAlgorithm::MIN)
        .value("MAX", SpatialLocationCalculatorAlgorithm::MAX)
        .value("MODE", SpatialLocationCalculatorAlgorithm::MODE)
        .value("MEDIAN", SpatialLocationCalculatorAlgorithm::MEDIAN);

    spatialLocationCalculatorConfigData.def(py::init<>())
        .def_readwrite("roi", &SpatialLocationCalculatorConfigData::roi, DOC(dai, SpatialLocationCalculatorConfigData, roi))
        .def_readwrite("depthThresholds", &SpatialLocationCalculatorConfigData::depthThresholds, DOC(dai, SpatialLocationCalculatorConfigData, depthThresholds))
        .def_readwrite("calculationAlgorithm",
                       &SpatialLocationCalculatorConfigData::calculationAlgorithm,
                       DOC(dai, SpatialLocationCalculatorConfigData, calculationAlgorithm));

    // Message
    spatialLocationCalculatorConfig.def(py::init<>())
        .def("__repr__", &SpatialLocationCalculatorConfig::str)
        .def_readwrite("globalStepSize", &SpatialLocationCalculatorConfig::globalStepSize)
        .def_readwrite("globalLowerThreshold", &SpatialLocationCalculatorConfig::globalLowerThreshold)
        .def_readwrite("globalUpperThreshold", &SpatialLocationCalculatorConfig::globalUpperThreshold)
        .def_readwrite("useKeypoints", &SpatialLocationCalculatorConfig::calculateSpatialKeypoints)
        .def_readwrite("useSegmentation", &SpatialLocationCalculatorConfig::useSegmentation)
        .def_readwrite("useKeypoints", &SpatialLocationCalculatorConfig::calculateSpatialKeypoints)
        // setters
        .def("setROIs", &SpatialLocationCalculatorConfig::setROIs, py::arg("ROIs"), DOC(dai, SpatialLocationCalculatorConfig, setROIs))
        .def("addROI", &SpatialLocationCalculatorConfig::addROI, py::arg("ROI"), DOC(dai, SpatialLocationCalculatorConfig, addROI))
        .def("getConfigData", &SpatialLocationCalculatorConfig::getConfigData, DOC(dai, SpatialLocationCalculatorConfig, getConfigData))
        .def("setDepthThresholds", &SpatialLocationCalculatorConfig::setDepthThresholds, DOC(dai, SpatialLocationCalculatorConfig, setDepthThresholds))
        .def("setCalculationAlgorithm", &SpatialLocationCalculatorConfig::setCalculationAlgorithm, DOC(dai, SpatialLocationCalculatorConfig, setCalculationAlgorithm))
        .def("setStepSize", &SpatialLocationCalculatorConfig::setStepSize, DOC(dai, SpatialLocationCalculatorConfig, setStepSize))
        .def("setKeypointRadius", &SpatialLocationCalculatorConfig::setKeypointRadius, DOC(dai, SpatialLocationCalculatorConfig, setKeypointRadius))
        .def("setCalculateSpatialKeypoints", &SpatialLocationCalculatorConfig::setCalculateSpatialKeypoints, DOC(dai, SpatialLocationCalculatorConfig, setCalculateSpatialKeypoints))
        .def("setUseSegmentation", &SpatialLocationCalculatorConfig::setUseSegmentation, DOC(dai, SpatialLocationCalculatorConfig, setUseSegmentation))
        .def("getDepthThresholds", &SpatialLocationCalculatorConfig::getDepthThresholds, DOC(dai, SpatialLocationCalculatorConfig, getDepthThresholds))
        .def("getCalculationAlgorithm", &SpatialLocationCalculatorConfig::getCalculationAlgorithm, DOC(dai, SpatialLocationCalculatorConfig, getCalculationAlgorithm))
        .def("getStepSize", &SpatialLocationCalculatorConfig::getStepSize, DOC(dai, SpatialLocationCalculatorConfig, getStepSize))
        .def("getKeypointRadius", &SpatialLocationCalculatorConfig::getKeypointRadius, DOC(dai, SpatialLocationCalculatorConfig, getKeypointRadius))
        .def("getCalculateSpatialKeypoints", &SpatialLocationCalculatorConfig::getCalculateSpatialKeypoints, DOC(dai, SpatialLocationCalculatorConfig, getCalculateSpatialKeypoints))
        .def("getUseSegmentation", &SpatialLocationCalculatorConfig::getUseSegmentation, DOC(dai, SpatialLocationCalculatorConfig, getUseSegmentation))
        ;
}
