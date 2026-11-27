#pragma once

#include "pybind11_common.hpp"

void bind_camera_stats(pybind11::module& m, void* pCallstack);
