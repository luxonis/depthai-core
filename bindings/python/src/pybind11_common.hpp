#pragma once

#if(_MSC_VER >= 1910) || !defined(_MSC_VER)
    #ifndef HAVE_SNPRINTF
        #define HAVE_SNPRINTF
    #endif
#endif

#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <cstdint>
#include <depthai/common/Point2f.hpp>
#include <depthai/common/WorldConstraints.hpp>
#include <pybind11_json/pybind11_json.hpp>
#include <stack>
#include <utility/SpanBindings.hpp>

// Include docstring file
#include "docstring.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<uint8_t>);
PYBIND11_MAKE_OPAQUE(std::vector<dai::Point2f>);
// Lists of WorldConstraints entries are live references in Python, so that append() and friends change the constraints.
PYBIND11_MAKE_OPAQUE(std::vector<dai::WorldConstraints::MarkerDistance>);
PYBIND11_MAKE_OPAQUE(std::vector<dai::WorldConstraints::MarkerSocketDistance>);
PYBIND11_MAKE_OPAQUE(std::vector<dai::WorldConstraints::MarkerSize>);

namespace py = pybind11;

using StackFunction = void (*)(pybind11::module& m, void* pCallstack);
using Callstack = std::stack<StackFunction>;
