#pragma once

#include "pybind11_common.hpp"

struct RemoteConnectionBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};
