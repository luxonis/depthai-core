#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstring>

#include "DatatypeBindings.hpp"
#include "depthai/beta/datatype/Map2D.hpp"
#include "pipeline/CommonBindings.hpp"

void bind_beta_map2d(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<beta::Map2D, Py<beta::Map2D>, Buffer, Transformable, std::shared_ptr<beta::Map2D>> map2D(betaModule, "Map2D", DOC(dai, beta, Map2D));

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    map2D.def(py::init<>())
        .def("__repr__", &beta::Map2D::str)
        .def(
            "getMap",
            [](const beta::Map2D& self) {
                const auto data = self.getMap();
                if(data.empty()) {
                    return py::array_t<float>();
                }
                const auto width = static_cast<py::ssize_t>(self.getWidth());
                const auto height = static_cast<py::ssize_t>(self.getHeight());
                py::array_t<float> arr({height, width});
                std::memcpy(arr.mutable_data(), data.data(), data.size() * sizeof(float));
                return arr;
            },
            DOC(dai, beta, Map2D, getMap))
        .def(
            "setMap",
            [](beta::Map2D& self, const py::array& map) {
                if(map.ndim() != 2) {
                    throw py::value_error("2D map must be a 2D array");
                }
                if(!map.dtype().is(py::dtype::of<float>())) {
                    throw py::value_error("2D map must be an array of floats");
                }
                py::array_t<float, py::array::c_style | py::array::forcecast> contiguous(map);
                const auto height = static_cast<size_t>(contiguous.shape(0));
                const auto width = static_cast<size_t>(contiguous.shape(1));
                self.setMap(dai::span<const float>(contiguous.data(), height * width), width, height);
            },
            py::arg("map"),
            DOC(dai, beta, Map2D, setMap))
        .def("getWidth", &beta::Map2D::getWidth, DOC(dai, beta, Map2D, getWidth))
        .def("getHeight", &beta::Map2D::getHeight, DOC(dai, beta, Map2D, getHeight))
        .def("transformTo", &beta::Map2D::transformTo, py::arg("target"), DOC(dai, beta, Map2D, transformTo))
        .def("getVisualizationMessage", &beta::Map2D::getVisualizationMessage, DOC(dai, beta, Map2D, getVisualizationMessage));
}
