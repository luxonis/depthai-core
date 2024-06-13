#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/rtabmap/RTABMapVIO.hpp"

#include <pybind11/eval.h>

extern py::handle daiNodeModule;

void bind_rtabmapslamnode(pybind11::module& m, void* pCallstack){
    using namespace dai;
    using namespace dai::node;

    // declare upfront
    auto rtabmapSLAMNode = ADD_NODE(RTABMapSLAM);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // RTABMapSLAM Node
    rtabmapSLAMNode
        .def_readonly("rect", &RTABMapSLAM::rect, DOC(dai, node, RTABMapSLAM, rect))
        .def_readonly("depth", &RTABMapSLAM::depth, DOC(dai, node, RTABMapSLAM, depth))
        .def_readonly("features", &RTABMapSLAM::features, DOC(dai, node, RTABMapSLAM, features))
        .def_readonly("odom", &RTABMapSLAM::odom, DOC(dai, node, RTABMapSLAM, odom))
        .def_readonly("transform", &RTABMapSLAM::transform, DOC(dai, node, RTABMapSLAM, transform))
        .def_readonly("odomCorrection", &RTABMapSLAM::odomCorrection, DOC(dai, node, RTABMapSLAM, odomCorrection))
        .def_readonly("obstaclePCL", &RTABMapSLAM::obstaclePCL, DOC(dai, node, RTABMapSLAM, obstaclePCL))
        .def_readonly("groundPCL", &RTABMapSLAM::groundPCL, DOC(dai, node, RTABMapSLAM, groundPCL))
        .def_readonly("occupancyGridMap", &RTABMapSLAM::occupancyGridMap, DOC(dai, node, RTABMapSLAM, occupancyGridMap))
        .def_readonly("passtrough_rect", &RTABMapSLAM::passtrough_rect, DOC(dai, node, RTABMapSLAM, passtrough_rect))
        .def_readonly("passtrough_depth", &RTABMapSLAM::passtrough_depth, DOC(dai, node, RTABMapSLAM, passtrough_depth))
        .def_readonly("passtrough_features", &RTABMapSLAM::passtrough_features, DOC(dai, node, RTABMapSLAM, passtrough_features))
        .def_readonly("passtrough_odom", &RTABMapSLAM::passtrough_odom, DOC(dai, node, RTABMapSLAM, passtrough_odom))
        .def("build", &RTABMapSLAM::build)
        .def("setParams", &RTABMapSLAM::setParams, py::arg("params"), DOC(dai, node, RTABMapSLAM, setParams))
        .def("setDatabasePath", &RTABMapSLAM::setDatabasePath, py::arg("path"), DOC(dai, node, RTABMapSLAM, setDatabasePath))
        .def("setLoadDatabaseOnStart", &RTABMapSLAM::setLoadDatabaseOnStart, py::arg("load"), DOC(dai, node, RTABMapSLAM, setLoadDatabaseOnStart))
        .def("setSaveDatabaseOnClose", &RTABMapSLAM::setSaveDatabaseOnClose, py::arg("save"), DOC(dai, node, RTABMapSLAM, setSaveDatabaseOnClose))
        .def("saveDatabase", &RTABMapSLAM::saveDatabase, DOC(dai, node, RTABMapSLAM, saveDatabase))
        .def("setSaveDatabasePeriodically", &RTABMapSLAM::setSaveDatabasePeriodically, py::arg("save"), DOC(dai, node, RTABMapSLAM, setSaveDatabasePeriodically))
        .def("setSaveDatabasePeriod", &RTABMapSLAM::setSaveDatabasePeriod, py::arg("period"), DOC(dai, node, RTABMapSLAM, setSaveDatabasePeriod))
        .def("setPublishObstacleCloud", &RTABMapSLAM::setPublishObstacleCloud, py::arg("publish"), DOC(dai, node, RTABMapSLAM, setPublishObstacleCloud))
        .def("setPublishGroundCloud", &RTABMapSLAM::setPublishGroundCloud, py::arg("publish"), DOC(dai, node, RTABMapSLAM, setPublishGroundCloud))
        .def("setPublishGrid", &RTABMapSLAM::setPublishGrid, py::arg("publish"), DOC(dai, node, RTABMapSLAM, setPublishGridMap))
        .def("setFrequency", &RTABMapSLAM::setFrequency, py::arg("f"), DOC(dai, node, RTABMapSLAM, setFrequency))
        .def("setAlphaScaling", &RTABMapSLAM::setAlphaScaling, py::arg("alpha"), DOC(dai, node, RTABMapSLAM, setAlphaScaling))
        .def("setUseFeatures", &RTABMapSLAM::setUseFeatures, py::arg("useFeatures"), DOC(dai, node, RTABMapSLAM, setUseFeatures))
        .def("setLocalTransform", &RTABMapSLAM::setLocalTransform, py::arg("transform"), DOC(dai, node, RTABMapSLAM, setLocalTransform))
        .def("getLocalTransform", &RTABMapSLAM::getLocalTransform, DOC(dai, node, RTABMapSLAM, getLocalTransform))
        .def("triggerNewMap", &RTABMapSLAM::triggerNewMap, DOC(dai, node, RTABMapSLAM, triggerNewMap))
}