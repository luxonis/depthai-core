
#pragma once

#include "pybind11_common.hpp"

struct EventsManagerBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};
