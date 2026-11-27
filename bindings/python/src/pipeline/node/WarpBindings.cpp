#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Warp.hpp"

void bind_warp(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<Warp::Properties> warpProperties(m, "WarpProperties", DOC(dai, WarpProperties));
    auto warp = ADD_NODE(Warp);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    warpProperties.def(py::init<>())
        .def_readwrite("interpolation", &WarpProperties::interpolation, DOC(dai, WarpProperties, interpolation))
        .def_readwrite("meshHeight", &WarpProperties::meshHeight, DOC(dai, WarpProperties, meshHeight))
        .def_readwrite("meshUri", &WarpProperties::meshUri, DOC(dai, WarpProperties, meshUri))
        .def_readwrite("meshWidth", &WarpProperties::meshWidth, DOC(dai, WarpProperties, meshWidth))
        .def_readwrite("numFramesPool", &WarpProperties::numFramesPool, DOC(dai, WarpProperties, numFramesPool))
        .def_readwrite("outputFrameSize", &WarpProperties::outputFrameSize, DOC(dai, WarpProperties, outputFrameSize))
        .def_readwrite("outputHeight", &WarpProperties::outputHeight, DOC(dai, WarpProperties, outputHeight))
        .def_readwrite("outputWidth", &WarpProperties::outputWidth, DOC(dai, WarpProperties, outputWidth))
        .def_readwrite("warpHwIds", &WarpProperties::warpHwIds, DOC(dai, WarpProperties, warpHwIds));

    warp.def_readonly("inputImage", &Warp::inputImage, DOC(dai, node, Warp, inputImage))
        .def_readonly("out", &Warp::out, DOC(dai, node, Warp, out))

        .def("setOutputSize", py::overload_cast<int, int>(&Warp::setOutputSize), DOC(dai, node, Warp, setOutputSize))
        .def("setOutputSize", py::overload_cast<const std::tuple<int, int>&>(&Warp::setOutputSize), DOC(dai, node, Warp, setOutputSize, 2))

        .def("setNumFramesPool", &Warp::setNumFramesPool, DOC(dai, node, Warp, setNumFramesPool))
        .def("setMaxOutputFrameSize", &Warp::setMaxOutputFrameSize, DOC(dai, node, Warp, setMaxOutputFrameSize))

        .def("setWarpMesh", py::overload_cast<const std::vector<Point2f>&, int, int>(&Warp::setWarpMesh), DOC(dai, node, Warp, setWarpMesh))
        .def("setWarpMesh", py::overload_cast<const std::vector<std::pair<float, float>>&, int, int>(&Warp::setWarpMesh), DOC(dai, node, Warp, setWarpMesh))

        .def("setHwIds", &Warp::setHwIds, DOC(dai, node, Warp, setHwIds))
        .def("getHwIds", &Warp::getHwIds, DOC(dai, node, Warp, getHwIds))
        .def("setInterpolation", &Warp::setInterpolation, DOC(dai, node, Warp, setInterpolation))
        .def("getInterpolation", &Warp::getInterpolation, DOC(dai, node, Warp, getInterpolation));

    daiNodeModule.attr("Warp").attr("Properties") = warpProperties;
}
