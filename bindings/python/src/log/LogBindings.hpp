#pragma once

#include "pybind11_common.hpp"

struct LogBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};
