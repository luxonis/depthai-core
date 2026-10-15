#pragma once

// pybind
#include "pybind11_common.hpp"

struct PipelineBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};

void bind_pipeline_schema(pybind11::module& m, void* pCallstack);
