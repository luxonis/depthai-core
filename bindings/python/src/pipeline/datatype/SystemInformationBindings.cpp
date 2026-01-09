#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/SystemInformation.hpp"
#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_systeminformation(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawSystemInformation, RawBuffer, std::shared_ptr<RawSystemInformation>> rawSystemInformation(m, "RawSystemInformation", DOC(dai,
    // RawSystemInformation));
    py::class_<SystemInformation, Py<SystemInformation>, Buffer, std::shared_ptr<SystemInformation>> systemInformation(
        m, "SystemInformation", DOC(dai, SystemInformation));

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

    // // Metadata / raw
    // rawSystemInformation
    //     .def(py::init<>())
    //     .def_readwrite("ddrMemoryUsage", &RawSystemInformation::ddrMemoryUsage)
    //     .def_readwrite("cmxMemoryUsage", &RawSystemInformation::cmxMemoryUsage)
    //     .def_readwrite("leonCssMemoryUsage", &RawSystemInformation::leonCssMemoryUsage)
    //     .def_readwrite("leonMssMemoryUsage", &RawSystemInformation::leonMssMemoryUsage)
    //     .def_readwrite("leonCssCpuUsage", &RawSystemInformation::leonCssCpuUsage)
    //     .def_readwrite("leonMssCpuUsage", &RawSystemInformation::leonMssCpuUsage)
    //     .def_readwrite("chipTemperature", &RawSystemInformation::chipTemperature)
    //     ;

    // Message
    systemInformation.def(py::init<>())
        .def("__repr__", &SystemInformation::str)
        .def_property(
            "ddrMemoryUsage", [](SystemInformation& i) { return &i.ddrMemoryUsage; }, [](SystemInformation& i, MemoryInfo val) { i.ddrMemoryUsage = val; })
        .def_property(
            "cmxMemoryUsage", [](SystemInformation& i) { return &i.cmxMemoryUsage; }, [](SystemInformation& i, MemoryInfo val) { i.cmxMemoryUsage = val; })
        .def_property(
            "leonCssMemoryUsage",
            [](SystemInformation& i) { return &i.leonCssMemoryUsage; },
            [](SystemInformation& i, MemoryInfo val) { i.leonCssMemoryUsage = val; })
        .def_property(
            "leonMssMemoryUsage",
            [](SystemInformation& i) { return &i.leonMssMemoryUsage; },
            [](SystemInformation& i, MemoryInfo val) { i.leonMssMemoryUsage = val; })
        .def_property(
            "leonCssCpuUsage", [](SystemInformation& i) { return &i.leonCssCpuUsage; }, [](SystemInformation& i, CpuUsage val) { i.leonCssCpuUsage = val; })
        .def_property(
            "leonMssCpuUsage", [](SystemInformation& i) { return &i.leonMssCpuUsage; }, [](SystemInformation& i, CpuUsage val) { i.leonMssCpuUsage = val; })
        .def_property(
            "chipTemperature",
            [](SystemInformation& i) { return &i.chipTemperature; },
            [](SystemInformation& i, ChipTemperature val) { i.chipTemperature = val; });
}

void bind_systeminformationRVC4(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawSystemInformationRVC4, RawBuffer, std::shared_ptr<RawSystemInformationRVC4>> rawSystemInformationRVC4(
    // m, "RawSystemInformationRVC4", DOC(dai, RawSystemInformationRVC4));
    py::class_<SystemInformationRVC4, Py<SystemInformationRVC4>, Buffer, std::shared_ptr<SystemInformationRVC4>> systemInformationRVC4(
        m, "SystemInformationRVC4", DOC(dai, SystemInformationRVC4));

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

    // // Metadata / raw
    // rawSystemInformationRVC4.def(py::init<>())
    //     .def_readwrite("ddrMemoryUsage", &RawSystemInformationRVC4::ddrMemoryUsage)
    //     .def_readwrite("cpuAvgUsage", &RawSystemInformationRVC4::cpuAvgUsage)
    //     .def_readwrite("chipTemperature", &RawSystemInformationRVC4::chipTemperature)
    //     .def_readwrite("cpuUsages", &RawSystemInformationRVC4::cpuUsages);

    // Message
    systemInformationRVC4.def(py::init<>())
        .def_property(
            "ddrMemoryUsage",
            [](SystemInformationRVC4& i) { return &i.ddrMemoryUsage; },
            [](SystemInformationRVC4& i, MemoryInfo val) { i.ddrMemoryUsage = val; })
        .def_property(
            "processMemoryUsage",
            [](SystemInformationRVC4& i) { return i.processMemoryUsage; },
            [](SystemInformationRVC4& i, int64_t val) { i.processMemoryUsage = val; })
        .def_property(
            "cpuAvgUsage", [](SystemInformationRVC4& i) { return &i.cpuAvgUsage; }, [](SystemInformationRVC4& i, CpuUsage val) { i.cpuAvgUsage = val; })
        .def_property(
            "processCpuAvgUsage",
            [](SystemInformationRVC4& i) { return &i.processCpuAvgUsage; },
            [](SystemInformationRVC4& i, CpuUsage val) { i.processCpuAvgUsage = val; })
        .def_property(
            "cpuUsages", [](SystemInformationRVC4& i) { return &i.cpuUsages; }, [](SystemInformationRVC4& i, std::vector<CpuUsage> val) { i.cpuUsages = val; })
        .def_property(
            "chipTemperature",
            [](SystemInformationRVC4& i) { return &i.chipTemperature; },
            [](SystemInformationRVC4& i, ChipTemperatureRVC4 val) { i.chipTemperature = val; });
}
