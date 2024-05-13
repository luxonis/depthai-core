
#include "PipelineBindings.hpp"

#include <pybind11/attr.h>
#include <pybind11/gil.h>

#include "node/NodeBindings.hpp"

// depthai
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

// depthai - nodes
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/BenchmarkOut.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/SPIOut.hpp"
#include "depthai/pipeline/node/SPIIn.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/Script.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/ObjectTracker.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/EdgeDetector.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai/pipeline/node/AprilTag.hpp"
#include "depthai/pipeline/node/DetectionParser.hpp"
#include "depthai/pipeline/node/UVC.hpp"
#include "depthai/pipeline/node/Warp.hpp"

// depthai/
#include "depthai/properties/GlobalProperties.hpp"
#include "utility/RecordReplay.hpp"
#include <memory>

std::shared_ptr<dai::Node> createNode(dai::Pipeline& p, py::object class_){
    auto nodeCreateMap = NodeBindings::getNodeCreateMap();
    for(auto& kv : nodeCreateMap){
        auto& node = kv.first;
        auto& create = kv.second;
        if(node.is(class_)){
            return create(p, class_);
        }
    }
    return nullptr;
}

void PipelineBindings::bind(pybind11::module& m, void* pCallstack){
    using namespace dai;

    // Type definitions
    py::class_<GlobalProperties> globalProperties(m, "GlobalProperties", DOC(dai, GlobalProperties));
    py::class_<utility::RecordConfig::VideoEncoding> recordVideoConfig(m, "VideoEncoding", DOC(dai, RecordConfig::VideoEncoding));
    py::class_<utility::RecordConfig> recordConfig(m, "RecordConfig", DOC(dai, RecordConfig));
    py::class_<Pipeline> pipeline(m, "Pipeline", DOC(dai, Pipeline, 2));

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


    // Bind global properties
    globalProperties
        .def_readwrite("leonOsFrequencyHz", &GlobalProperties::leonCssFrequencyHz)
        .def_readwrite("leonRtFrequencyHz", &GlobalProperties::leonMssFrequencyHz)
        .def_readwrite("pipelineName", &GlobalProperties::pipelineName)
        .def_readwrite("pipelineVersion", &GlobalProperties::pipelineVersion)
        .def_readwrite("cameraTuningBlobSize", &GlobalProperties::cameraTuningBlobSize, DOC(dai, GlobalProperties, cameraTuningBlobSize))
        .def_readwrite("cameraTuningBlobUri", &GlobalProperties::cameraTuningBlobUri, DOC(dai, GlobalProperties, cameraTuningBlobUri))
        .def_readwrite("xlinkChunkSize", &GlobalProperties::xlinkChunkSize, DOC(dai, GlobalProperties, xlinkChunkSize))
        .def_readwrite("sippBufferSize", &GlobalProperties::sippBufferSize, DOC(dai, GlobalProperties, sippBufferSize))
        .def_readwrite("sippDmaBufferSize", &GlobalProperties::sippDmaBufferSize, DOC(dai, GlobalProperties, sippDmaBufferSize))
        ;

    recordVideoConfig
        .def_readwrite("enabled", &utility::RecordConfig::VideoEncoding::enabled, DOC(dai, RecordConfig, VideoEncoding, enabled))
        .def_readwrite("bitrate", &utility::RecordConfig::VideoEncoding::bitrate, DOC(dai, RecordConfig, VideoEncoding, bitrate))
        .def_readwrite("profile", &utility::RecordConfig::VideoEncoding::profile, DOC(dai, RecordConfig, VideoEncoding, profile))
        .def_readwrite("lossless", &utility::RecordConfig::VideoEncoding::lossless, DOC(dai, RecordConfig, VideoEncoding, lossless))
        .def_readwrite("quality", &utility::RecordConfig::VideoEncoding::quality, DOC(dai, RecordConfig, VideoEncoding, quality));

    recordConfig
        .def_readwrite("outputDir", &utility::RecordConfig::outputDir, DOC(dai, RecordConfig, outputDir))
        .def_readwrite("videoEncoding", &utility::RecordConfig::videoEncoding, DOC(dai, RecordConfig, videoEncoding))
        .def_readwrite("compressionLevel", &utility::RecordConfig::compressionLevel, DOC(dai, RecordConfig, compressionLevel));

    // bind pipeline
    pipeline.def(py::init<bool>(), py::arg("createImplicitDevice") = true, DOC(dai, Pipeline, Pipeline))
        .def(py::init<std::shared_ptr<Device>>(), py::arg("defaultDevice"), DOC(dai, Pipeline, Pipeline))
        // Python only methods
        .def("__enter__",
             [](Pipeline& p) -> Pipeline& {
                 setImplicitPipeline(p);
                 return p;
             })
        .def("__exit__",
             [](Pipeline& d, py::object type, py::object value, py::object traceback) {
                 py::gil_scoped_release release;
                 delImplicitPipeline();
                 d.stop();
                 d.wait();
             })
        //.def(py::init<const Pipeline&>())
        .def("getDefaultDevice", [](Pipeline& p) -> py::object {
            auto device = p.getDefaultDevice();
            if(!device) return py::none();
            return py::cast(device);
        }, DOC(dai, Pipeline, getDefaultDevice))
        .def("getGlobalProperties", &Pipeline::getGlobalProperties, DOC(dai, Pipeline, getGlobalProperties))
        //.def("create", &Pipeline::create<node::XLinkIn>)
        .def("remove", &Pipeline::remove, py::arg("node"), DOC(dai, Pipeline, remove))
        .def("getAllNodes", static_cast<std::vector<std::shared_ptr<Node>> (Pipeline::*)() const>(&Pipeline::getAllNodes), DOC(dai, Pipeline, getAllNodes))
        // .def("getAllNodes", static_cast<std::vector<std::shared_ptr< Node>> (Pipeline::*)()>(&Pipeline::getAllNodes),
        // py::return_value_policy::reference_internal, DOC(dai, Pipeline, getAllNodes))
        .def("getNode", static_cast<std::shared_ptr<Node> (Pipeline::*)(Node::Id)>(&Pipeline::getNode), DOC(dai, Pipeline, getNode))
        // .def("getNode", static_cast<std::shared_ptr<Node> (Pipeline::*)(Node::Id) const>(&Pipeline::getNode), DOC(dai, Pipeline, getNode))
        // .def("getNode", static_cast<std::shared_ptr<Node> (Pipeline::*)(Node::Id)>(&Pipeline::getNode), py::return_value_policy::reference_internal, DOC(dai,
        // Pipeline, getNode)) .def("getConnections", &Pipeline::getConnections, DOC(dai, Pipeline, getConnections), DOC(dai, Pipeline, getConnections))
        // .def("getConnectionMap", &Pipeline::getConnectionMap, DOC(dai, Pipeline, getConnectionMap), py::return_value_policy::reference_internal, DOC(dai,
        // Pipeline, getConnectionMap)) .def("getNodeMap", &Pipeline::getNodeMap, DOC(dai, Pipeline, getNodeMap), py::return_value_policy::reference_internal,
        // DOC(dai, Pipeline, getNodeMap)) .def("link", &Pipeline::link, DOC(dai, Pipeline, link), DOC(dai, Pipeline, link)) .def("unlink", &Pipeline::unlink,
        // DOC(dai, Pipeline, unlink), DOC(dai, Pipeline, unlink))
        .def("getAssetManager",
             static_cast<const AssetManager& (Pipeline::*)() const>(&Pipeline::getAssetManager),
             py::return_value_policy::reference_internal,
             DOC(dai, Pipeline, getAssetManager))
        .def("getAssetManager",
             static_cast<AssetManager& (Pipeline::*)()>(&Pipeline::getAssetManager),
             py::return_value_policy::reference_internal,
             DOC(dai, Pipeline, getAssetManager))
        .def("setOpenVINOVersion", &Pipeline::setOpenVINOVersion, py::arg("version"), DOC(dai, Pipeline, setOpenVINOVersion))
        .def("getOpenVINOVersion", &Pipeline::getOpenVINOVersion, DOC(dai, Pipeline, getOpenVINOVersion))
        .def("getRequiredOpenVINOVersion", &Pipeline::getRequiredOpenVINOVersion, DOC(dai, Pipeline, getRequiredOpenVINOVersion))
        .def("setCameraTuningBlobPath", &Pipeline::setCameraTuningBlobPath, py::arg("path"), DOC(dai, Pipeline, setCameraTuningBlobPath))
        .def("setXLinkChunkSize", &Pipeline::setXLinkChunkSize, py::arg("sizeBytes"), DOC(dai, Pipeline, setXLinkChunkSize))
        .def("setSippBufferSize", &Pipeline::setSippBufferSize, py::arg("sizeBytes"), DOC(dai, Pipeline, setSippBufferSize))
        .def("setSippDmaBufferSize", &Pipeline::setSippDmaBufferSize, py::arg("sizeBytes"), DOC(dai, Pipeline, setSippDmaBufferSize))
        .def("setCalibrationData", &Pipeline::setCalibrationData, py::arg("calibrationDataHandler"), DOC(dai, Pipeline, setCalibrationData))
        .def("getCalibrationData", &Pipeline::getCalibrationData, DOC(dai, Pipeline, getCalibrationData))
        .def("getDeviceConfig", &Pipeline::getDeviceConfig, DOC(dai, Pipeline, getDeviceConfig))
        .def("serializeToJson", &Pipeline::serializeToJson, DOC(dai, Pipeline, serializeToJson))
        .def("setBoardConfig", &Pipeline::setBoardConfig, DOC(dai, Pipeline, setBoardConfig))
        .def("getBoardConfig", &Pipeline::getBoardConfig, DOC(dai, Pipeline, getBoardConfig))
        // 'Template' create function
        .def(
            "add",
            [](Pipeline& p, std::shared_ptr<Node> hostNode) {
                p.add(hostNode);
                return hostNode;
            },
            py::keep_alive<1, 2>())
        // 'Template' create function
        .def("create",
             [](dai::Pipeline& p, py::object class_, const py::args& args, const py::kwargs& kwargs) {
                 // Check if class_ is a subclass of HostNode
                 py::object issubclass = py::module::import("builtins").attr("issubclass");
                 py::object nodeClass = py::module::import("depthai").attr("node").attr("ThreadedHostNode");
                 auto isSubclass = issubclass(class_, nodeClass).cast<bool>();

                 // Check if the class is directly from bindings (__module__ == "depthai.node"). If so, the node comes from bindings,
                 // so we create in the same manner as device nodes.
                 auto isFromBindings = class_.attr("__module__").cast<std::string>() == "depthai.node";
                 if(isSubclass && !isFromBindings) {
                     std::shared_ptr<Node> hostNode = py::cast<std::shared_ptr<node::ThreadedHostNode>>(class_(*args, **kwargs));
                     // Node already adds itself to the pipeline in the constructor
                     // To be sure - check if it is already added
                     auto allNodes = p.getAllNodes();
                     auto found = false;
                     for(auto& n : allNodes) {
                         if(n == hostNode) {
                             found = true;
                             break;
                         }
                     }
                     if(!found) {
                         throw std::runtime_error("Node was not added to the pipeline in the constructor");
                     }
                     //  p.add(hostNode);
                     return hostNode;
                 }
                 // Otherwise create the node with `pipeline.create()` method
                 auto node = createNode(p, class_);
                 if(node == nullptr) {
                     throw std::invalid_argument(std::string(py::str(class_)) + " is not a subclass of depthai.node");
                 }
                 return node;
             }, py::keep_alive<1,0>())
        // TODO(themarpe) DEPRECATE, use pipeline.create([class name])
        // templated create<NODE> function
        .def("createXLinkIn", &Pipeline::create<node::XLinkIn>)
        .def("createXLinkOut", &Pipeline::create<node::XLinkOut>)
        .def("createNeuralNetwork", &Pipeline::create<node::NeuralNetwork>)
        .def("createColorCamera", &Pipeline::create<node::ColorCamera>)
        .def("createVideoEncoder", &Pipeline::create<node::VideoEncoder>)
        .def("createScript", &Pipeline::create<node::Script>)
        .def("createSPIOut", &Pipeline::create<node::SPIOut>)
        .def("createSPIIn", &Pipeline::create<node::SPIIn>)
        .def("createImageManip", &Pipeline::create<node::ImageManip>)
        .def("createMonoCamera", &Pipeline::create<node::MonoCamera>)
        .def("createStereoDepth", &Pipeline::create<node::StereoDepth>)
        .def("createMobileNetDetectionNetwork", &Pipeline::create<node::MobileNetDetectionNetwork>)
        .def("createYoloDetectionNetwork", &Pipeline::create<node::YoloDetectionNetwork>)
        .def("createSystemLogger", &Pipeline::create<node::SystemLogger>)
        .def("createSpatialLocationCalculator", &Pipeline::create<node::SpatialLocationCalculator>)
        .def("createMobileNetSpatialDetectionNetwork", &Pipeline::create<node::MobileNetSpatialDetectionNetwork>)
        .def("createYoloSpatialDetectionNetwork", &Pipeline::create<node::YoloSpatialDetectionNetwork>)
        .def("createObjectTracker", &Pipeline::create<node::ObjectTracker>)
        .def("createIMU", &Pipeline::create<node::IMU>)
        .def("createEdgeDetector", &Pipeline::create<node::EdgeDetector>)
        .def("createFeatureTracker", &Pipeline::create<node::FeatureTracker>)
        .def("createAprilTag", &Pipeline::create<node::AprilTag>)
        .def("createDetectionParser", &Pipeline::create<node::DetectionParser>)
        .def("createUVC", &Pipeline::create<node::UVC>)
        .def("createCamera", &Pipeline::create<node::Camera>)
        .def("createWarp", &Pipeline::create<node::Warp>)
        .def("start", &Pipeline::start)
        .def("wait",
             [](Pipeline& p) {
                 py::gil_scoped_release release;
                 p.wait();
             })
        .def("stop", &Pipeline::stop)
        .def("run",
             [](Pipeline& p) {
                 py::gil_scoped_release release;
                 p.run();
             })
        .def("isRunning", &Pipeline::isRunning)
        .def("processTasks", &Pipeline::processTasks)
        .def("enableHolisticRecord", &Pipeline::enableHolisticRecord, py::arg("recordConfig"), DOC(dai, Pipeline, enableHolisticRecord))
        .def("enableHolisticReplay", &Pipeline::enableHolisticReplay, py::arg("recordingPath"), DOC(dai, Pipeline, enableHolisticReplay));
    ;


}
