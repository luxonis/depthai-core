#include "FilterParamsBindings.hpp"

#include "depthai/pipeline/FilterParams.hpp"

void FilterParamsBindings::bind(pybind11::module& m, void* pCallstack) {
    // dai.filters.params
    py::module_ filters = m.def_submodule("filters", "Parameters for filters");
    py::module_ params = filters.def_submodule("params", "Parameters for filters");

    // -- Median Filter --
    py::enum_<dai::filters::params::MedianFilter> medianFilter(params, "MedianFilter");
    medianFilter.value("MEDIAN_OFF", dai::filters::params::MedianFilter::MEDIAN_OFF)
        .value("KERNEL_3x3", dai::filters::params::MedianFilter::KERNEL_3x3)
        .value("KERNEL_5x5", dai::filters::params::MedianFilter::KERNEL_5x5)
        .value("KERNEL_7x7", dai::filters::params::MedianFilter::KERNEL_7x7)
        .def("__str__", [](const dai::filters::params::MedianFilter& self) { return py::str("MedianFilter({})").format(self); });

    // -- Spatial Filter --
    py::class_<dai::filters::params::SpatialFilter> spatialFilter(params, "SpatialFilter");
    spatialFilter.def(py::init<>())
        .def_readwrite("enable", &dai::filters::params::SpatialFilter::enable, DOC(dai, filters, params, SpatialFilter, enable))
        .def_readwrite(
            "holeFillingRadius", &dai::filters::params::SpatialFilter::holeFillingRadius, DOC(dai, filters, params, SpatialFilter, holeFillingRadius))
        .def_readwrite("alpha", &dai::filters::params::SpatialFilter::alpha, DOC(dai, filters, params, SpatialFilter, alpha))
        .def_readwrite("delta", &dai::filters::params::SpatialFilter::delta, DOC(dai, filters, params, SpatialFilter, delta))
        .def_readwrite("numIterations", &dai::filters::params::SpatialFilter::numIterations, DOC(dai, filters, params, SpatialFilter, numIterations))
        .def("__str__", [](const dai::filters::params::SpatialFilter& self) {
            return py::str("SpatialFilter(enable={}, holeFillingRadius={}, alpha={}, delta={}, numIterations={})")
                .format(self.enable, self.holeFillingRadius, self.alpha, self.delta, self.numIterations);
        });

    // -- Temporal Filter --
    py::class_<dai::filters::params::TemporalFilter> temporalFilter(params, "TemporalFilter", DOC(dai, filters, params, TemporalFilter));
    temporalFilter.def(py::init<>())
        .def_readwrite("enable", &dai::filters::params::TemporalFilter::enable, DOC(dai, filters, params, TemporalFilter, enable))
        .def_readwrite("persistencyMode", &dai::filters::params::TemporalFilter::persistencyMode, DOC(dai, filters, params, TemporalFilter, persistencyMode))
        .def_readwrite("alpha", &dai::filters::params::TemporalFilter::alpha, DOC(dai, filters, params, TemporalFilter, alpha))
        .def_readwrite("delta", &dai::filters::params::TemporalFilter::delta, DOC(dai, filters, params, TemporalFilter, delta))
        .def("__str__", [](const dai::filters::params::TemporalFilter& self) {
            return py::str("TemporalFilter(enable={}, persistencyMode={}, alpha={}, delta={})")
                .format(self.enable, self.persistencyMode, self.alpha, self.delta);
        });

    py::enum_<dai::filters::params::TemporalFilter::PersistencyMode> persistencyMode(
        temporalFilter, "PersistencyMode", DOC(dai, filters, params, TemporalFilter, PersistencyMode));
    persistencyMode
        .value("PERSISTENCY_OFF",
               dai::filters::params::TemporalFilter::PersistencyMode::PERSISTENCY_OFF,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, PERSISTENCY_OFF))
        .value("VALID_8_OUT_OF_8",
               dai::filters::params::TemporalFilter::PersistencyMode::VALID_8_OUT_OF_8,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, VALID_8_OUT_OF_8))
        .value("VALID_2_IN_LAST_3",
               dai::filters::params::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_3,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, VALID_2_IN_LAST_3))
        .value("VALID_2_IN_LAST_4",
               dai::filters::params::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_4,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, VALID_2_IN_LAST_4))
        .value("VALID_2_OUT_OF_8",
               dai::filters::params::TemporalFilter::PersistencyMode::VALID_2_OUT_OF_8,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, VALID_2_OUT_OF_8))
        .value("VALID_1_IN_LAST_2",
               dai::filters::params::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_2,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, VALID_1_IN_LAST_2))
        .value("VALID_1_IN_LAST_5",
               dai::filters::params::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_5,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, VALID_1_IN_LAST_5))
        .value("VALID_1_IN_LAST_8",
               dai::filters::params::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_8,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, VALID_1_IN_LAST_8))
        .value("PERSISTENCY_INDEFINITELY",
               dai::filters::params::TemporalFilter::PersistencyMode::PERSISTENCY_INDEFINITELY,
               DOC(dai, filters, params, TemporalFilter, PersistencyMode, PERSISTENCY_INDEFINITELY));

    // -- Speckle filter --
    py::class_<dai::filters::params::SpeckleFilter> speckleFilter(params, "SpeckleFilter");
    speckleFilter.def(py::init<>())
        .def_readwrite("enable", &dai::filters::params::SpeckleFilter::enable, DOC(dai, filters, params, SpeckleFilter, enable))
        .def_readwrite("speckleRange", &dai::filters::params::SpeckleFilter::speckleRange, DOC(dai, filters, params, SpeckleFilter, speckleRange))
        .def_readwrite(
            "differenceThreshold", &dai::filters::params::SpeckleFilter::differenceThreshold, DOC(dai, filters, params, SpeckleFilter, differenceThreshold))
        .def("__str__", [](const dai::filters::params::SpeckleFilter& self) {
            return py::str("SpeckleFilter(enable={}, speckleRange={}, differenceThreshold={})")
                .format(self.enable, self.speckleRange, self.differenceThreshold);
        });

    // Aliases for backward compatibility
    m.attr("MedianFilter") = medianFilter;
    m.attr("StereoDepthConfig").attr("MedianFilter") = medianFilter;
    m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpatialFilter") = spatialFilter;
    m.attr("StereoDepthConfig").attr("PostProcessing").attr("TemporalFilter") = temporalFilter;
    m.attr("StereoDepthConfig").attr("PostProcessing").attr("SpeckleFilter") = speckleFilter;

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
}