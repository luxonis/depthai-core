#include "NodeBindings.hpp"
#include "Common.hpp"

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/Camera.hpp"

void bind_camera(pybind11::module& m, void* pCallstack){

    using namespace dai;
    using namespace dai::node;
    using namespace pybind11::literals;

    // Node and Properties declare upfront
    py::class_<CameraProperties> cameraProperties(m, "CameraProperties", DOC(dai, CameraProperties));
    py::enum_<CameraProperties::WarpMeshSource> cameraPropertiesWarpMeshSource(cameraProperties, "WarpMeshSource", DOC(dai, CameraProperties, WarpMeshSource));
    py::enum_<CameraProperties::ColorOrder> cameraPropertiesColorOrder(cameraProperties, "ColorOrder", DOC(dai, CameraProperties, ColorOrder));
    auto camera = ADD_NODE(Camera);

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

    // // Camera Properties
    // cameraPropertiesSensorResolution
    //     .value("THE_1080_P", CameraProperties::SensorResolution::THE_1080_P)
    //     .value("THE_1200_P", CameraProperties::SensorResolution::THE_1200_P)
    //     .value("THE_4_K", CameraProperties::SensorResolution::THE_4_K)
    //     .value("THE_5_MP", CameraProperties::SensorResolution::THE_5_MP)
    //     .value("THE_12_MP", CameraProperties::SensorResolution::THE_12_MP)
    //     .value("THE_4000X3000", CameraProperties::SensorResolution::THE_4000X3000)
    //     .value("THE_13_MP", CameraProperties::SensorResolution::THE_13_MP)
    //     .value("THE_5312X6000", CameraProperties::SensorResolution::THE_5312X6000)
    //     .value("THE_48_MP", CameraProperties::SensorResolution::THE_48_MP)
    //     .value("THE_720_P", CameraProperties::SensorResolution::THE_720_P)
    //     .value("THE_800_P", CameraProperties::SensorResolution::THE_800_P)
    //     ;

    // Camera Properties - WarpMeshSource
    cameraPropertiesWarpMeshSource
        .value("AUTO", CameraProperties::WarpMeshSource::AUTO)
        .value("NONE", CameraProperties::WarpMeshSource::NONE)
        .value("CALIBRATION", CameraProperties::WarpMeshSource::CALIBRATION)
        .value("URI", CameraProperties::WarpMeshSource::URI)
        ;

    cameraPropertiesColorOrder
        .value("BGR", CameraProperties::ColorOrder::BGR)
        .value("RGB", CameraProperties::ColorOrder::RGB)
        ;

    cameraProperties
        .def_readwrite("initialControl", &CameraProperties::initialControl)
        .def_readwrite("boardSocket", &CameraProperties::boardSocket)
        .def_readwrite("cameraName", &CameraProperties::cameraName)
        .def_readwrite("imageOrientation", &CameraProperties::imageOrientation)
        .def_readwrite("colorOrder", &CameraProperties::colorOrder)
        .def_readwrite("interleaved", &CameraProperties::interleaved)
        .def_readwrite("fp16", &CameraProperties::fp16)

        .def_readwrite("previewHeight", &CameraProperties::previewHeight)
        .def_readwrite("previewWidth", &CameraProperties::previewWidth)
        .def_readwrite("videoHeight", &CameraProperties::videoHeight)
        .def_readwrite("videoWidth", &CameraProperties::videoWidth)
        .def_readwrite("stillHeight", &CameraProperties::stillHeight)
        .def_readwrite("stillWidth", &CameraProperties::stillWidth)
        .def_readwrite("resolutionHeight", &CameraProperties::resolutionHeight)
        .def_readwrite("resolutionWidth", &CameraProperties::resolutionWidth)

        .def_readwrite("fps", &CameraProperties::fps)
        .def_readwrite("isp3aFps", &CameraProperties::isp3aFps)
        .def_readwrite("sensorCropX", &CameraProperties::sensorCropX)
        .def_readwrite("sensorCropY", &CameraProperties::sensorCropY)
        .def_readwrite("previewKeepAspectRatio", &CameraProperties::previewKeepAspectRatio)
        .def_readwrite("ispScale", &CameraProperties::ispScale)
        .def_readwrite("sensorType", &CameraProperties::sensorType)

        .def_readwrite("numFramesPoolRaw", &CameraProperties::numFramesPoolRaw)
        .def_readwrite("numFramesPoolIsp", &CameraProperties::numFramesPoolIsp)
        .def_readwrite("numFramesPoolVideo", &CameraProperties::numFramesPoolVideo)
        .def_readwrite("numFramesPoolPreview", &CameraProperties::numFramesPoolPreview)
        .def_readwrite("numFramesPoolStill", &CameraProperties::numFramesPoolStill)

        .def_readwrite("warpMeshSource", &CameraProperties::warpMeshSource)
        .def_readwrite("warpMeshUri", &CameraProperties::warpMeshUri)
        .def_readwrite("warpMeshWidth", &CameraProperties::warpMeshWidth)
        .def_readwrite("warpMeshHeight", &CameraProperties::warpMeshHeight)
        .def_readwrite("calibAlpha", &CameraProperties::calibAlpha)
        .def_readwrite("warpMeshStepWidth", &CameraProperties::warpMeshStepWidth)
        .def_readwrite("warpMeshStepHeight", &CameraProperties::warpMeshStepHeight)
        .def_readwrite("rawPacked", &CameraProperties::rawPacked)
    ;

    // Camera node
    camera
        .def_readonly("inputControl", &Camera::inputControl, DOC(dai, node, Camera, inputControl))
        .def_readonly("initialControl", &Camera::initialControl, DOC(dai, node, Camera, initialControl))
#define CAMERA_ARGS \
        CameraBoardSocket boardSocket
#define CAMERA_PYARGS \
        py::arg("boardSocket") = CameraBoardSocket::AUTO
        // TODO (Zimamazim) Automatically fetch default arguments to avoid duplicity
#define CAMERA_CODE(OP) \
        self OP setBoardSocket(boardSocket);
        .def("build", [](Camera &self, CAMERA_ARGS) {
                self.build();
                CAMERA_CODE(.)
                return std::static_pointer_cast<Camera>(self.shared_from_this());
            },
            CAMERA_PYARGS
            )
        .def(py::init([](CAMERA_ARGS){
                auto self = getImplicitPipeline().create<Camera>();
                self->build();
                CAMERA_CODE(->)
                return self;
            }),
            CAMERA_PYARGS
            )
        .def("setBoardSocket", &Camera::setBoardSocket, py::arg("boardSocket"), DOC(dai, node, Camera, setBoardSocket))
        .def("getBoardSocket", &Camera::getBoardSocket, DOC(dai, node, Camera, getBoardSocket))
        .def("setCamera", &Camera::setCamera, py::arg("name"), DOC(dai, node, Camera, setCamera))
        .def("getCamera", &Camera::getCamera, DOC(dai, node, Camera, getCamera))
        .def("requestOutput", py::overload_cast<std::pair<uint32_t, uint32_t>, ImgFrame::Type, ImgResizeMode, uint32_t>(&Camera::requestOutput), "size"_a, "encoding"_a=dai::ImgFrame::Type::NV12, "resizeMode"_a=dai::ImgResizeMode::CROP, "fps"_a=30, py::return_value_policy::reference_internal, DOC(dai, node, Camera, requestOutput))
        .def("requestOutput",  py::overload_cast<const Capability&, bool>(&Camera::requestOutput), "capability"_a, "onHost"_a, py::return_value_policy::reference_internal, DOC(dai, node, Camera, requestOutput, 2))
        ;
    // ALIAS
    daiNodeModule.attr("Camera").attr("Properties") = cameraProperties;

}
