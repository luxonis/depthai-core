#pragma once

// pybind
#include "pybind11_common.hpp"

struct DatatypeBindings {
    static void addToCallstack(std::deque<StackFunction>& callstack);
 private:
    static void bind(pybind11::module& m, void* pCallstack);
};

namespace dai {
template <typename T>
// This is used so pybind detects the classes as overridable in python,
// which trigers the holders to keep the python part of the object alive
class Py : public T {
    public:
        using T::T;
};
}