#include "ZooBindings.hpp"

// depthai
#include "depthai/modelzoo/Zoo.hpp"

void ZooBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    m.def("getModelFromZoo",
          getModelFromZoo,
          py::arg("modelDescription"),
          py::arg("useCached") = true,
          py::arg("cacheDirectory") = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY,
          DOC(dai, getModelFromZoo));

    m.def("downloadModelsFromZoo",
          downloadModelsFromZoo,
          py::arg("path"),
          py::arg("cacheDirectory") = MODEL_ZOO_DEFAULT_CACHE_DIRECTORY,
          DOC(dai, downloadModelsFromZoo));
}