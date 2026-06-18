#include "Common.hpp"
#include "depthai/common/ToFPreset.hpp"
#include "depthai/common/ToFSensorMode.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/ToF.hpp"

#include <unordered_set>
#include <cstdint>

namespace {

// imageFiltersPresetToToFPreset / toFPresetToImageFiltersPreset are shared from
// depthai/pipeline/datatype/ToFConfig.hpp (single source of truth for the mapping).

void warnOncePerToF(const dai::node::ToF& self, const char* key, const char* message) {
    static std::unordered_set<std::string> warned;
    const std::string id = std::string(key) + "@" + std::to_string(reinterpret_cast<std::uintptr_t>(&self));
    if(warned.insert(id).second) {
        pybind11::module_::import("warnings").attr("warn")(message);
    }
}

}  // namespace

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
        .def_readonly("rawInput", &ToFBase::rawInput, DOC(dai, node, ToFBase, rawInput), DOC(dai, node, ToFBase, rawInput))
        .def_readonly("depth", &ToFBase::depth, DOC(dai, node, ToFBase, depth), DOC(dai, node, ToFBase, depth))
        .def_readonly("amplitude", &ToFBase::amplitude, DOC(dai, node, ToFBase, amplitude), DOC(dai, node, ToFBase, amplitude))
        .def_readonly("intensity", &ToFBase::intensity, DOC(dai, node, ToFBase, intensity), DOC(dai, node, ToFBase, intensity))
        .def_readonly("confidence", &ToFBase::confidence, DOC(dai, node, ToFBase, confidence), DOC(dai, node, ToFBase, confidence))
        .def_readonly("phase", &ToFBase::phase, DOC(dai, node, ToFBase, phase), DOC(dai, node, ToFBase, phase))
        .def_readonly("raw", &ToFBase::raw, DOC(dai, node, ToFBase, raw), DOC(dai, node, ToFBase, raw))
        .def_readonly("initialConfig", &ToFBase::initialConfig, DOC(dai, node, ToFBase, initialConfig), DOC(dai, node, ToFBase, initialConfig))
        .def("build",
             py::overload_cast<CameraBoardSocket, ImageFiltersPresetMode, std::optional<float>>(&ToFBase::build),
             "boardSocket"_a = CameraBoardSocket::AUTO,
             "presetMode"_a = ImageFiltersPresetMode::TOF_MID_RANGE,
             "fps"_a = std::nullopt,
             DOC(dai, node, ToFBase, build))
        .def("getBoardSocket", &ToFBase::getBoardSocket, DOC(dai, node, ToFBase, getBoardSocket));

    // ToF Node (DeviceNodeGroup)
    tof.def_property_readonly(
           "rawDepth",
           [](const ToF& self) -> const dai::DeviceNode::Output& {
               const auto device = self.getDevice();
               if(device && device->getPlatform() == Platform::RVC4) {
                   warnOncePerToF(self, "rawDepth", "On RVC4 rawDepth is the same IPP output as depth; use tof.depth");
               }
               return self.rawDepth;
           },
           DOC(dai, node, ToF, rawDepth))
        .def_property_readonly(
            "rawInput", [](const ToF& self) -> const dai::DeviceNode::Input& { return self.rawInput; }, DOC(dai, node, ToF, rawInput))
        .def_property_readonly(
            "depth", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.depth; }, DOC(dai, node, ToF, depth))
        .def_property_readonly(
            "amplitude", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.amplitude; }, DOC(dai, node, ToF, amplitude))
        .def_property_readonly(
            "intensity", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.intensity; }, DOC(dai, node, ToF, intensity))
        .def_property_readonly(
            "confidence", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.confidence; }, DOC(dai, node, ToF, confidence))
        .def_property_readonly(
            "phase",
            [](const ToF& self) -> const dai::DeviceNode::Output& {
                const auto device = self.getDevice();
                if(device && device->getPlatform() == Platform::RVC4) {
                    warnOncePerToF(self, "phase", "tof.phase is not produced on RVC4 ToF");
                } else {
                    warnOncePerToF(self, "phase", "tof.phase is deprecated (kept for RVC2 backward compatibility)");
                }
                return self.phase;
            },
            DOC(dai, node, ToF, phase))
        .def_property_readonly(
            "raw", [](const ToF& self) -> const dai::DeviceNode::Output& { return self.raw; }, DOC(dai, node, ToF, raw))
        .def_property_readonly(
            "inputConfig",
            [](const ToF& self) -> const dai::DeviceNode::Input& { return self.inputConfig; },
            "Runtime ToFConfig input (decoder on RVC2, IPP on RVC4). On RVC2 this retunes the decoder, not the "
            "host ImageFilters that produce `depth` \xe2\x80\x94 use imageFiltersInputConfig (ImageFiltersConfig) for that.")
        .def_property_readonly(
            "initialConfig",
            [](ToF& self) -> std::shared_ptr<ToFConfig>& { return self.tofBaseNode.initialConfig; },
            py::return_value_policy::reference_internal,
            "Initial ToF config (IPP fields on RVC4, decoder fields on RVC2)")
        .def_property_readonly(
            "tofBaseInputConfig",
            [](const ToF& self) -> const dai::DeviceNode::Input& { return self.tofBaseInputConfig; },
            DOC(dai, node, ToF, tofBaseInputConfig))
        .def_property_readonly(
            "imageFiltersInputConfig",
            [](const ToF& self) -> const dai::DeviceNode::Input& { return self.imageFiltersInputConfig; },
            DOC(dai, node, ToF, imageFiltersInputConfig))
        .def_property_readonly(
            "tofBaseNode",
            [](const ToF& self) -> const dai::node::ToFBase& {
                const auto device = self.getDevice();
                if(device && device->getPlatform() == Platform::RVC4) {
                    warnOncePerToF(self, "tofBaseNode", "tofBaseNode is internal on RVC4; use tof.initialConfig, tof.inputConfig, and tof.rawInput directly");
                }
                return self.tofBaseNode;
            },
            "Internal ToF base node (RVC2 only; deprecated on RVC4)")
        .def_property_readonly(
            "imageFiltersNode",
            [](const ToF& self) -> const dai::node::ImageFilters& {
                const auto device = self.getDevice();
                if(device && device->getPlatform() == Platform::RVC4) {
                    warnOncePerToF(self, "imageFiltersNode", "ImageFilters is not used on RVC4 ToF");
                }
                return self.imageFiltersNode;
            },
            DOC(dai, node, ToF, imageFiltersNode))
        .def_static("create", &ToF::create, "device"_a, DOC(dai, node, ToF, create))
        .def("build",
             [](ToF& self,
                CameraBoardSocket boardSocket,
                py::object presetMode,
                py::object fps,
                py::object preset) -> std::shared_ptr<ToF> {
                 ToFBuildOptions options;
                 options.boardSocket = boardSocket;

                 if(!fps.is_none()) {
                     options.fps = fps.cast<float>();
                 }
                 if(!preset.is_none()) {
                     options.preset = preset.cast<ToFPreset>();
                 }

                 const bool hasPresetMode = !presetMode.is_none();
                 const bool hasPreset = !preset.is_none();

                 if(hasPresetMode && py::isinstance<ToFPreset>(presetMode)) {
                     throw std::runtime_error("Pass ToFPreset via preset= keyword, not as the second positional argument");
                 }

                 const auto device = self.getDevice();
                 const bool isRvc4 = device && device->getPlatform() == Platform::RVC4;

                 if(isRvc4) {
                     if(hasPresetMode && hasPreset) {
                         throw std::runtime_error("Specify either presetMode or preset on RVC4, not both");
                     }
                     if(hasPresetMode) {
                         options.preset = imageFiltersPresetToToFPreset(presetMode.cast<ImageFiltersPresetMode>());
                     }
                     return self.build(options);
                 }

                 if(hasPresetMode && hasPreset) {
                     throw std::runtime_error("Specify either presetMode or preset on RVC2, not both");
                 }

                 // Keep the unified build surface consistent across platforms: map ToFPreset to the
                 // equivalent ImageFiltersPresetMode on RVC2 instead of silently dropping it to MID_RANGE.
                 ImageFiltersPresetMode mode = ImageFiltersPresetMode::TOF_MID_RANGE;
                 if(hasPresetMode) {
                     mode = presetMode.cast<ImageFiltersPresetMode>();
                 } else if(hasPreset) {
                     mode = toFPresetToImageFiltersPreset(preset.cast<ToFPreset>());
                 }

                 return self.build(options.boardSocket, mode, options.fps);
             },
             py::arg("boardSocket") = CameraBoardSocket::AUTO,
             py::arg("presetMode") = py::none(),
             py::arg("fps") = py::none(),
             py::arg("preset") = py::none(),
             "Build ToF node for the connected device platform")
        .def("build",
             py::overload_cast<const ToFBuildOptions&>(&ToF::build),
             "options"_a,
             "Build ToF node using ToFBuildOptions")
        .def("getCamera", &ToF::getCamera, "Returns auto-created Camera on RVC4 after pipeline.start(), or None when rawInput was user-connected")
        .def("getBoardSocket", &ToF::getBoardSocket, "Board socket selected at build time")
        .def("getOutputResolution",
             &ToF::getOutputResolution,
             "RVC4: depth/amplitude/confidence output size (width, height) for the fixed F3_FULL capture")
        .def("getRawResolution",
             &ToF::getRawResolution,
             "RVC4: raw VD55H1 superframe size (width, height) for manual Camera.build(sensorResolution=...)")
        .def("getSensorResolution",
             &ToF::getSensorResolution,
             "Deprecated alias for getOutputResolution(); use getOutputResolution() or getRawResolution()")
        .def("getInitialConfig", [&](const ToF& self) { return *self.tofBaseNode.initialConfig; })
        .def("setInitialConfig", [&](ToF& self, ToFConfig& config) { self.tofBaseNode.initialConfig = std::make_shared<ToFConfig>(config); });

    // ALIAS
    daiNodeModule.attr("ToFBase").attr("Properties") = tofProperties;
}
