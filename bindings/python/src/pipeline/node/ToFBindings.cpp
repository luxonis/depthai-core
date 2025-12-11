#include "Common.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/ToF.hpp"

void bind_tof(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Node and Properties declare upfront
    py::class_<ToFProperties> tofProperties(m, "ToFProperties", DOC(dai, ToFProperties));
    auto tofBase = ADD_NODE(ToFBase);
    auto tof = ADD_NODE_DERIVED(ToF, DeviceNodeGroup);

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
    tofProperties.def_readwrite("initialConfig", &ToFProperties::initialConfig, DOC(dai, ToFProperties, initialConfig))
        .def_readwrite("numFramesPool", &ToFProperties::numFramesPool, DOC(dai, ToFProperties, numFramesPool))
        .def_readwrite("numShaves", &ToFProperties::numShaves, DOC(dai, ToFProperties, numShaves))
        .def_readwrite("warpHwIds", &ToFProperties::warpHwIds, DOC(dai, ToFProperties, warpHwIds));

    // ToFBase Node
    tofBase.def_readonly("inputConfig", &ToFBase::inputConfig, DOC(dai, node, ToFBase, inputConfig), DOC(dai, node, ToFBase, inputConfig))
        .def_readonly("depth", &ToFBase::depth, DOC(dai, node, ToFBase, depth), DOC(dai, node, ToFBase, depth))
        .def_readonly("amplitude", &ToFBase::amplitude, DOC(dai, node, ToFBase, amplitude), DOC(dai, node, ToFBase, amplitude))
        .def_readonly("intensity", &ToFBase::intensity, DOC(dai, node, ToFBase, intensity), DOC(dai, node, ToFBase, intensity))
        .def_readonly("phase", &ToFBase::phase, DOC(dai, node, ToFBase, phase), DOC(dai, node, ToFBase, phase))
        .def_readonly("initialConfig", &ToFBase::initialConfig, DOC(dai, node, ToFBase, initialConfig), DOC(dai, node, ToFBase, initialConfig))
        .def("build",
             &ToFBase::build,
             "boardSocket"_a = CameraBoardSocket::AUTO,
             "presetMode"_a = ImageFiltersPresetMode::TOF_MID_RANGE,
             "fps"_a = std::nullopt,
             DOC(dai, node, ToFBase, build))
        .def("getBoardSocket", &ToFBase::getBoardSocket, DOC(dai, node, ToFBase, getBoardSocket));

    // ToF Node (DeviceNodeGroup)
    tof.def_property_readonly(
           "rawDepth", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.rawDepth; }, DOC(dai, node, ToF, rawDepth))
        .def_property_readonly(
            "depth", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.depth; }, DOC(dai, node, ToF, depth))
        .def_property_readonly(
            "amplitude", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.amplitude; }, DOC(dai, node, ToF, amplitude))
        .def_property_readonly(
            "intensity", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.intensity; }, DOC(dai, node, ToF, intensity))
        .def_property_readonly(
            "phase", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.phase; }, DOC(dai, node, ToF, phase))
        .def_property_readonly(
            "tofBaseInputConfig",
            [](const ToF& self) -> const dai::DeviceNode::Input& { return self.tofBaseInputConfig; },
            DOC(dai, node, ToF, tofBaseInputConfig))
        .def_property_readonly(
            "imageFiltersInputConfig",
            [](const ToF& self) -> const dai::DeviceNode::Input& { return self.imageFiltersInputConfig; },
            DOC(dai, node, ToF, imageFiltersInputConfig))
        .def_property_readonly(
            "tofBaseNode", [](const ToF& self) -> const dai::node::ToFBase& { return self.tofBaseNode; }, DOC(dai, node, ToF, tofBaseNode))
        .def_property_readonly(
            "imageFiltersNode", [](const ToF& self) -> const dai::node::ImageFilters& { return self.imageFiltersNode; }, DOC(dai, node, ToF, imageFiltersNode))
        .def_static("create", &ToF::create, "device"_a, DOC(dai, node, ToF, create))
        .def("build",
             &ToF::build,
             "boardSocket"_a = CameraBoardSocket::AUTO,
             "presetMode"_a = ImageFiltersPresetMode::TOF_MID_RANGE,
             "fps"_a = std::nullopt,
             DOC(dai, node, ToF, build))
        .def("getInitialConfig", [&](const ToF& self) { return *self.tofBaseNode.initialConfig; })
        .def("setInitialConfig", [&](ToF& self, ToFConfig& config) { self.tofBaseNode.initialConfig = std::make_shared<ToFConfig>(config); });

    // ALIAS
    daiNodeModule.attr("ToFBase").attr("Properties") = tofProperties;
}
