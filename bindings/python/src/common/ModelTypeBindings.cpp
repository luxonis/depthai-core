#include "ModelTypeBindings.hpp"

#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <depthai/common/ModelType.hpp>

void ModelTypeBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::enum_<model::ModelType> modelType(m, "ModelType", DOC(dai, model, ModelType));

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

    // Bind ModelType
    modelType.value("BLOB", model::ModelType::BLOB);
    modelType.value("SUPERBLOB", model::ModelType::SUPERBLOB);
    modelType.value("DLC", model::ModelType::DLC);
    modelType.value("NNARCHIVE", model::ModelType::NNARCHIVE);
    modelType.value("OTHER", model::ModelType::OTHER);

    m.def("readModelType", &model::readModelType, py::arg("modelPath"), DOC(dai, model, readModelType));
}
