#pragma once
#include "pybind11_common.hpp"

struct FilterParamsBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};