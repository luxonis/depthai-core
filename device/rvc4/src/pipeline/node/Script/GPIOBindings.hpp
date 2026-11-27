#pragma once

#include <pybind11/pybind11.h>

namespace python {
namespace gpio {

void bind(pybind11::module_ m);

} // namespace gpio
} // namespace python



