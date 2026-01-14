#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/PointCloud.hpp"

void bind_pointcloud(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<PointCloudProperties> properties(m, "PointCloudProperties", DOC(dai, PointCloudProperties));
    auto node = ADD_NODE(PointCloud);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Properties
    properties.def_readwrite("initialConfig", &PointCloudProperties::initialConfig, DOC(dai, PointCloudProperties, initialConfig))
        .def_readwrite("numFramesPool", &PointCloudProperties::numFramesPool, DOC(dai, PointCloudProperties, numFramesPool));

    // Node
    node.def_readonly("inputConfig", &PointCloud::inputConfig, DOC(dai, node, PointCloud, inputConfig))
        .def_readonly("inputDepth", &PointCloud::inputDepth, DOC(dai, node, PointCloud, inputDepth))
        .def_readonly("outputPointCloud", &PointCloud::outputPointCloud, DOC(dai, node, PointCloud, outputPointCloud))
        .def_readonly("passthroughDepth", &PointCloud::passthroughDepth, DOC(dai, node, PointCloud, passthroughDepth))
        .def_readonly("initialConfig", &PointCloud::initialConfig, DOC(dai, node, PointCloud, initialConfig))
        .def("setNumFramesPool", &PointCloud::setNumFramesPool, DOC(dai, node, PointCloud, setNumFramesPool))
        .def("setRunOnHost", &PointCloud::setRunOnHost, DOC(dai, node, PointCloud, setRunOnHost))
        .def("setDepthUnit", &PointCloud::setDepthUnit, py::arg("depthUnit"), DOC(dai, node, PointCloud, setDepthUnit))
        .def("useCPU", &PointCloud::useCPU, DOC(dai, node, PointCloud, useCPU))
        .def("useCPUMT", &PointCloud::useCPUMT, py::arg("numThreads") = 2, DOC(dai, node, PointCloud, useCPUMT))
        .def("useGPU", &PointCloud::useGPU, py::arg("device") = 0, DOC(dai, node, PointCloud, useGPU))
        .def("setTargetCoordinateSystem", py::overload_cast<CameraBoardSocket, bool>(&PointCloud::setTargetCoordinateSystem), 
             py::arg("targetCamera"), py::arg("useSpecTranslation") = false,
             DOC(dai, node, PointCloud, setTargetCoordinateSystem))
        .def("setTargetCoordinateSystem", py::overload_cast<HousingCoordinateSystem, bool>(&PointCloud::setTargetCoordinateSystem),
             py::arg("housingCS"), py::arg("useSpecTranslation") = false,
             DOC(dai, node, PointCloud, setTargetCoordinateSystem, 2))
        ;
    
    // ALIAS
    daiNodeModule.attr("PointCloud").attr("Properties") = properties;
}
