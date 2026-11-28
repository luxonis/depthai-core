#pragma once

#include "pybind11_common.hpp"

struct CapabilityRangeBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};
