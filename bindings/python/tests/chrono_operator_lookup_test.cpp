// Keep this order: the operator helper must not hide pybind11::int_ in chrono.h.
// clang-format off
#include <pybind11/operators.h>
#include <pybind11/chrono.h>
// clang-format on

#include "depthai_pybind11_tests.hpp"

TEST_SUBMODULE(chrono_operator_lookup, m) {
    m.def("roundtrip", [](std::chrono::system_clock::time_point value) { return value; });
}
