#pragma once

#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

namespace dai {
namespace bindings {

// Converts raw mask bytes to a numpy array shaped (height, width) when the
// dimensions match the data size, flat otherwise. Empty data gives an empty array.
inline pybind11::array_t<std::uint8_t> toNumpyMask(const std::vector<std::uint8_t>& data, size_t width, size_t height) {
    namespace py = pybind11;
    if(data.empty()) {
        return py::array_t<std::uint8_t>();
    }

    const auto w = static_cast<py::ssize_t>(width);
    const auto h = static_cast<py::ssize_t>(height);
    const auto count = static_cast<py::ssize_t>(data.size());

    py::array_t<std::uint8_t> arr = (w > 0 && h > 0 && w * h == count) ? py::array_t<std::uint8_t>({h, w}) : py::array_t<std::uint8_t>(count);
    std::memcpy(arr.mutable_data(), data.data(), data.size());
    return arr;
}

// Overload for messages where the mask is optional: returns None when the mask is not set.
inline pybind11::object toNumpyMask(const std::optional<std::vector<std::uint8_t>>& data, size_t width, size_t height) {
    if(!data.has_value()) {
        return pybind11::none();
    }
    return toNumpyMask(*data, width, height);
}

}  // namespace bindings
}  // namespace dai
