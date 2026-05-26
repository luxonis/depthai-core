#pragma once

// pybind
#include <type_traits>

#include "depthai/pipeline/datatype/ImgAnnotations.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "pybind11_common.hpp"

struct DatatypeBindings {
    static void addToCallstack(std::deque<StackFunction>& callstack);

   private:
    static void bind(pybind11::module& m, void* pCallstack);
};

namespace dai {
template <typename T, typename Enable = void>
// This is used so pybind detects the classes as overridable in python,
// which trigers the holders to keep the python part of the object alive
class Py : public T {
   public:
    using T::T;
    dai::VisualizeType getVisualizationMessage() const override {
        PYBIND11_OVERLOAD(dai::VisualizeType,      /* Return type */
                          T,                       /* Parent class */
                          getVisualizationMessage, /* Name of function in C++ (must match Python name) */
        );
    }
};

template <typename T>
class Py<T, std::enable_if_t<std::is_base_of_v<dai::TransformableBuffer, T>>> : public T {
   public:
    using T::T;

    std::shared_ptr<dai::TransformableBuffer> transformTo(const dai::ImgTransformation& target) const override {
        PYBIND11_OVERLOAD(std::shared_ptr<dai::TransformableBuffer>, T, transformTo, target);
    }
};
}  // namespace dai
