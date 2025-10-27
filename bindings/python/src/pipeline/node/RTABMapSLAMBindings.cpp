#include <pybind11/eval.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/rtabmap/RTABMapSLAM.hpp"

extern py::handle daiNodeModule;

void bind_rtabmapslamnode(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto rtabmapSLAMNode = ADD_NODE_DERIVED(RTABMapSLAM, ThreadedHostNode);

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

    // RTABMapSLAM Node
    rtabmapSLAMNode.def_property_readonly(
                       "rect", [](RTABMapSLAM& node) { return &node.rect; }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "depth", [](RTABMapSLAM& node) { return &node.depth; }, py::return_value_policy::reference_internal)
        .def_readonly("features", &RTABMapSLAM::features, DOC(dai, node, RTABMapSLAM, features))
        .def_readonly("odom", &RTABMapSLAM::odom, DOC(dai, node, RTABMapSLAM, odom))
        .def_readonly("transform", &RTABMapSLAM::transform, DOC(dai, node, RTABMapSLAM, transform))
        .def_readonly("odomCorrection", &RTABMapSLAM::odomCorrection, DOC(dai, node, RTABMapSLAM, odomCorrection))
        .def_readonly("obstaclePCL", &RTABMapSLAM::obstaclePCL, DOC(dai, node, RTABMapSLAM, obstaclePCL))
        .def_readonly("groundPCL", &RTABMapSLAM::groundPCL, DOC(dai, node, RTABMapSLAM, groundPCL))
        .def_readonly("occupancyGridMap", &RTABMapSLAM::occupancyGridMap, DOC(dai, node, RTABMapSLAM, occupancyGridMap))
        .def_readonly("passthroughRect", &RTABMapSLAM::passthroughRect, DOC(dai, node, RTABMapSLAM, passthroughRect))
        .def_readonly("passthroughDepth", &RTABMapSLAM::passthroughDepth, DOC(dai, node, RTABMapSLAM, passthroughDepth))
        .def_readonly("passthroughFeatures", &RTABMapSLAM::passthroughFeatures, DOC(dai, node, RTABMapSLAM, passthroughFeatures))
        .def_readonly("passthroughOdom", &RTABMapSLAM::passthroughOdom, DOC(dai, node, RTABMapSLAM, passthroughOdom))
        .def("setParams", &RTABMapSLAM::setParams, py::arg("params"), DOC(dai, node, RTABMapSLAM, setParams))
        .def("setDatabasePath", &RTABMapSLAM::setDatabasePath, py::arg("path"), DOC(dai, node, RTABMapSLAM, setDatabasePath))
        .def("setLoadDatabaseOnStart", &RTABMapSLAM::setLoadDatabaseOnStart, py::arg("load"), DOC(dai, node, RTABMapSLAM, setLoadDatabaseOnStart))
        .def("setSaveDatabaseOnClose", &RTABMapSLAM::setSaveDatabaseOnClose, py::arg("save"), DOC(dai, node, RTABMapSLAM, setSaveDatabaseOnClose))
        .def("saveDatabase", &RTABMapSLAM::saveDatabase, DOC(dai, node, RTABMapSLAM, saveDatabase))
        .def(
            "setSaveDatabasePeriodically", &RTABMapSLAM::setSaveDatabasePeriodically, py::arg("save"), DOC(dai, node, RTABMapSLAM, setSaveDatabasePeriodically))
        .def("setSaveDatabasePeriod", &RTABMapSLAM::setSaveDatabasePeriod, py::arg("period"), DOC(dai, node, RTABMapSLAM, setSaveDatabasePeriod))
        .def("setPublishObstacleCloud", &RTABMapSLAM::setPublishObstacleCloud, py::arg("publish"), DOC(dai, node, RTABMapSLAM, setPublishObstacleCloud))
        .def("setPublishGroundCloud", &RTABMapSLAM::setPublishGroundCloud, py::arg("publish"), DOC(dai, node, RTABMapSLAM, setPublishGroundCloud))
        .def("setPublishGrid", &RTABMapSLAM::setPublishGrid, py::arg("publish"), DOC(dai, node, RTABMapSLAM, setPublishGrid))
        .def("setFreq", &RTABMapSLAM::setFreq, py::arg("f"), DOC(dai, node, RTABMapSLAM, setFreq))
        .def("setAlphaScaling", &RTABMapSLAM::setAlphaScaling, py::arg("alpha"), DOC(dai, node, RTABMapSLAM, setAlphaScaling))
        .def("setUseFeatures", &RTABMapSLAM::setUseFeatures, py::arg("useFeatures"), DOC(dai, node, RTABMapSLAM, setUseFeatures))
        .def("setLocalTransform", &RTABMapSLAM::setLocalTransform, py::arg("transform"), DOC(dai, node, RTABMapSLAM, setLocalTransform))
        .def("getLocalTransform", &RTABMapSLAM::getLocalTransform, DOC(dai, node, RTABMapSLAM, getLocalTransform))
        .def("triggerNewMap", &RTABMapSLAM::triggerNewMap, DOC(dai, node, RTABMapSLAM, triggerNewMap));
}