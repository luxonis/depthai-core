#include "NNArchiveBindings.hpp"

#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/NNArchiveBlob.hpp"
#include "depthai/nn_archive/NNArchiveConfig.hpp"
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/nn_archive/v1/Config.hpp"
#include "depthai/nn_archive/v1/ConfigVersion.hpp"
#include "depthai/nn_archive/v1/DataType.hpp"
#include "depthai/nn_archive/v1/InputType.hpp"
#include "depthai/nn_archive/v1/Metadata.hpp"
#include "depthai/nn_archive/v1/Model.hpp"
#include "depthai/nn_archive/v1/ObjectDetectionSubtypeYolo.hpp"
#include "depthai/nn_archive/v1/Output.hpp"
#include "depthai/nn_archive/v1/Outputs.hpp"
#include "depthai/nn_archive/v1/PreprocessingBlock.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<uint8_t>);

void NNArchiveBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::nn_archive;

    // NNArchive
    py::class_<NNArchive> nnArchive(m, "NNArchive", DOC(dai, NNArchive));

    // NNArchiveBlob
    py::class_<NNArchiveBlob> nnArchiveBlob(m, "NNArchiveBlob", DOC(dai, NNArchiveBlob));

    // NNArchiveConfig
    py::class_<NNArchiveConfig> nnArchiveConfig(m, "NNArchiveConfig", DOC(dai, NNArchiveConfig));

    // NNArchiveEntry
    py::class_<NNArchiveEntry> nnArchiveEntry(m, "NNArchiveEntry", DOC(dai, NNArchiveEntry));
    py::enum_<NNArchiveEntry::Compression> archiveEntryCompression(nnArchiveEntry, "Compression");
    py::enum_<NNArchiveEntry::Seek> archiveEntrySeek(nnArchiveEntry, "Seek");

    // NNArchive v1
    auto v1m = m.def_submodule("nn_archive").def_submodule("v1");
    py::enum_<v1::ConfigVersion> v1configVersion(v1m, "ConfigVersion", DOC(dai, nn_archive, v1, ConfigVersion));
    py::class_<v1::Config> v1config(v1m, "Config", DOC(dai, nn_archive, v1, Config));
    py::class_<v1::Model> v1model(v1m, "Model", DOC(dai, nn_archive, v1, Model));
    py::enum_<v1::ObjectDetectionSubtypeYolo> v1objectDetectionSubtypeYolo(
        v1m, "ObjectDetectionSubtypeYolo", DOC(dai, nn_archive, v1, ObjectDetectionSubtypeYolo));
    py::class_<v1::Head> v1head(v1m, "Head", DOC(dai, nn_archive, v1, Head));
    py::enum_<v1::DataType> v1dataType(v1m, "DataType", DOC(dai, nn_archive, v1, DataType));
    py::enum_<v1::InputType> v1inputType(v1m, "InputType", DOC(dai, nn_archive, v1, InputType));
    py::class_<v1::Input> v1input(v1m, "Input", DOC(dai, nn_archive, v1, Input));
    py::class_<v1::Metadata> v1metadata(v1m, "Metadata", DOC(dai, nn_archive, v1, Metadata));
    py::class_<v1::Output> v1output(v1m, "Output", DOC(dai, nn_archive, v1, Output));
    py::class_<v1::Outputs> v1outputs(v1m, "Outputs", DOC(dai, nn_archive, v1, Outputs));
    py::class_<v1::PreprocessingBlock> v1preprocessingBlock(v1m, "PreprocessingBlock", DOC(dai, nn_archive, v1, PreprocessingBlock));

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

    // Bind NNArchive
    nnArchive.def(py::init<const dai::Path&, NNArchiveEntry::Compression>(),
                  py::arg("path"),
                  py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                  DOC(dai, NNArchive, NNArchive));
    nnArchive.def(py::init<const std::vector<uint8_t>&, NNArchiveEntry::Compression>(),
                  py::arg("data"),
                  py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                  DOC(dai, NNArchive, NNArchive));
    nnArchive.def(py::init<const std::function<int()>&,
                           const std::function<std::shared_ptr<std::vector<uint8_t>>()>&,
                           const std::function<int64_t(int64_t, NNArchiveEntry::Seek)>&,
                           const std::function<int64_t(int64_t)>&,
                           const std::function<int()>&,
                           NNArchiveEntry::Compression>());
     nnArchive.def(py::init<const NNArchiveConfig&, const NNArchiveBlob&>(),
                  py::arg("config"),
                  py::arg("blob"),
                  DOC(dai, NNArchive, NNArchive));
     nnArchive.def("getConfig", &NNArchive::getConfig, DOC(dai, NNArchive, getConfig));
     nnArchive.def("getBlob", &NNArchive::getBlob, DOC(dai, NNArchive, getBlob));

    // Bind NNArchiveBlob
    nnArchiveBlob.def(py::init<const NNArchiveConfig&, const std::vector<uint8_t>&, NNArchiveEntry::Compression>(),
                      py::arg("config"),
                      py::arg("data"),
                      py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                      DOC(dai, NNArchiveBlob, NNArchiveBlob));
    nnArchiveBlob.def(py::init<const NNArchiveConfig&, const dai::Path&, NNArchiveEntry::Compression>(),
                      py::arg("config"),
                      py::arg("path"),
                      py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                      DOC(dai, NNArchiveBlob, NNArchiveBlob));
    nnArchiveBlob.def(py::init<const NNArchiveConfig&,
                               const std::function<int()>&,
                               const std::function<std::shared_ptr<std::vector<uint8_t>>()>&,
                               const std::function<int64_t(int64_t, NNArchiveEntry::Seek)>&,
                               const std::function<int64_t(int64_t)>&,
                               const std::function<int()>&,
                               NNArchiveEntry::Compression>(),
                      py::arg("config"),
                      py::arg("openCallback"),
                      py::arg("readCallback"),
                      py::arg("seekCallback"),
                      py::arg("skipCallback"),
                      py::arg("closeCallback"),
                      py::arg("compression") = NNArchiveEntry::Compression::AUTO);
    nnArchiveBlob.def("getOpenVINOBlob", &NNArchiveBlob::getOpenVINOBlob, DOC(dai, NNArchiveBlob, getOpenVINOBlob));

    // Bind NNArchiveConfig
    nnArchiveConfig.def(py::init<const dai::Path&, NNArchiveEntry::Compression>(),
                        py::arg("path"),
                        py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                        DOC(dai, NNArchiveConfig, NNArchiveConfig));
    nnArchiveConfig.def(py::init<const std::vector<uint8_t>&, NNArchiveEntry::Compression>(),
                        py::arg("data"),
                        py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                        DOC(dai, NNArchiveConfig, NNArchiveConfig));
    nnArchiveConfig.def(py::init<const std::function<int()>&,
                                 const std::function<std::shared_ptr<std::vector<uint8_t>>()>&,
                                 const std::function<int64_t(int64_t, NNArchiveEntry::Seek)>&,
                                 const std::function<int64_t(int64_t)>&,
                                 const std::function<int()>&,
                                 NNArchiveEntry::Compression>());
     nnArchiveConfig.def("getConfigV1", &NNArchiveConfig::getConfigV1, DOC(dai, NNArchiveConfig, getConfigV1));

    // Bind NNArchiveEntry
    archiveEntryCompression.value("AUTO", NNArchiveEntry::Compression::AUTO)
        .value("RAW_FS", NNArchiveEntry::Compression::RAW_FS)
        .value("TAR", NNArchiveEntry::Compression::TAR)
        .value("TAR_GZ", NNArchiveEntry::Compression::TAR_GZ)
        .value("TAR_XZ", NNArchiveEntry::Compression::TAR_XZ);
    archiveEntrySeek.value("SET", NNArchiveEntry::Seek::SET).value("CUR", NNArchiveEntry::Seek::CUR).value("END", NNArchiveEntry::Seek::END);

    // Bind NNArchive v1
    v1configVersion.
        value("THE_10", v1::ConfigVersion::THE_10);

    v1config.def(py::init<>());
    v1config.def(py::init<v1::ConfigVersion, v1::Model>(),
                 py::arg("configVersion"),
                 py::arg("model"),
                 DOC(dai, nn_archive, v1, Config, Config));
    v1config.def_readwrite("configVersion", &v1::Config::configVersion, DOC(dai, nn_archive, v1, Config, configVersion));
    v1config.def_readwrite("model", &v1::Config::model, DOC(dai, nn_archive, v1, Config, model));

    v1model.def(py::init<>());
    v1model.def(py::init<std::optional<std::vector<v1::Head>>, std::vector<v1::Input>, v1::Metadata, std::vector<v1::Output>>(),
                py::arg("heads"),
                py::arg("inputs"),
                py::arg("metadata"),
                py::arg("outputs"),
                DOC(dai, nn_archive, v1, Model, Model));
    v1model.def_readwrite("heads", &v1::Model::heads, DOC(dai, nn_archive, v1, Model, heads));
    v1model.def_readwrite("inputs", &v1::Model::inputs, DOC(dai, nn_archive, v1, Model, inputs));
    v1model.def_readwrite("metadata", &v1::Model::metadata, DOC(dai, nn_archive, v1, Model, metadata));
    v1model.def_readwrite("outputs", &v1::Model::outputs, DOC(dai, nn_archive, v1, Model, outputs));

    v1objectDetectionSubtypeYolo.
        value("YOLOV5", v1::ObjectDetectionSubtypeYolo::YOLOV5)
        .value("YOLOV6", v1::ObjectDetectionSubtypeYolo::YOLOV6)
        .value("YOLOV7", v1::ObjectDetectionSubtypeYolo::YOLOV7)
        .value("YOLOV8", v1::ObjectDetectionSubtypeYolo::YOLOV8);

    v1head.def(py::init<>());
    v1head.def_readwrite("classes", &v1::Head::classes, DOC(dai, nn_archive, v1, Head, classes));
    v1head.def_readwrite("family", &v1::Head::family, DOC(dai, nn_archive, v1, Head, family));
    v1head.def_readwrite("isSoftmax", &v1::Head::isSoftmax, DOC(dai, nn_archive, v1, Head, isSoftmax));
    v1head.def_readwrite("nClasses", &v1::Head::nClasses, DOC(dai, nn_archive, v1, Head, nClasses));
    v1head.def_readwrite("outputs", &v1::Head::outputs, DOC(dai, nn_archive, v1, Head, outputs));
    v1head.def_readwrite("anchors", &v1::Head::anchors, DOC(dai, nn_archive, v1, Head, anchors));
    v1head.def_readwrite("confThreshold", &v1::Head::confThreshold, DOC(dai, nn_archive, v1, Head, confThreshold));
    v1head.def_readwrite("iouThreshold", &v1::Head::iouThreshold, DOC(dai, nn_archive, v1, Head, iouThreshold));
    v1head.def_readwrite("maxDet", &v1::Head::maxDet, DOC(dai, nn_archive, v1, Head, maxDet));
    v1head.def_readwrite("nKeypoints", &v1::Head::nKeypoints, DOC(dai, nn_archive, v1, Head, nKeypoints));
    v1head.def_readwrite("nPrototypes", &v1::Head::nPrototypes, DOC(dai, nn_archive, v1, Head, nPrototypes));
    v1head.def_readwrite("prototypeOutputName", &v1::Head::prototypeOutputName, DOC(dai, nn_archive, v1, Head, prototypeOutputName));
    v1head.def_readwrite("subtype", &v1::Head::subtype, DOC(dai, nn_archive, v1, Head, subtype));
    v1head.def_readwrite("postprocessorPath", &v1::Head::postprocessorPath, DOC(dai, nn_archive, v1, Head, postprocessorPath));

    v1dataType.
        value("FLOAT16", v1::DataType::FLOAT16)
        .value("FLOAT32", v1::DataType::FLOAT32)
        .value("INT8", v1::DataType::INT8)
        .value("NV12", v1::DataType::NV12)
        .value("UINT8", v1::DataType::UINT8);
    v1inputType
        .value("IMAGE", v1::InputType::IMAGE)
        .value("RAW", v1::InputType::RAW);

    v1input.def(py::init<>());
    v1input.def(py::init<v1::DataType, v1::InputType, std::string, v1::PreprocessingBlock, std::vector<int64_t>>(),
                py::arg("dtype"),
                py::arg("inputType"),
                py::arg("name"),
                py::arg("preprocessing"),
                py::arg("shape"),
                DOC(dai, nn_archive, v1, Input, Input));
    v1input.def_readwrite("dtype", &v1::Input::dtype, DOC(dai, nn_archive, v1, Input, dtype));
    v1input.def_readwrite("inputType", &v1::Input::inputType, DOC(dai, nn_archive, v1, Input, inputType));
    v1input.def_readwrite("name", &v1::Input::name, DOC(dai, nn_archive, v1, Input, name));
    v1input.def_readwrite("preprocessing", &v1::Input::preprocessing, DOC(dai, nn_archive, v1, Input, preprocessing));
    v1input.def_readwrite("shape", &v1::Input::shape, DOC(dai, nn_archive, v1, Input, shape));

    v1metadata.def(py::init<>());
    v1metadata.def(py::init<std::string, std::string>(),
                   py::arg("name"),
                   py::arg("path"),
                   DOC(dai, nn_archive, v1, Metadata, Metadata));
    v1metadata.def_readwrite("name", &v1::Metadata::name, DOC(dai, nn_archive, v1, Metadata, name));
    v1metadata.def_readwrite("path", &v1::Metadata::path, DOC(dai, nn_archive, v1, Metadata, path));

    v1output.def(py::init<>());
    v1output.def(py::init<v1::DataType, std::string>(),
                 py::arg("dtype"),
                 py::arg("name"),
                 DOC(dai, nn_archive, v1, Output, Output));
    v1output.def_readwrite("dtype", &v1::Output::dtype, DOC(dai, nn_archive, v1, Output, dtype));
    v1output.def_readwrite("name", &v1::Output::name, DOC(dai, nn_archive, v1, Output, name));

    v1outputs.def(py::init<>());
    v1outputs.def_readwrite("predictions", &v1::Outputs::predictions, DOC(dai, nn_archive, v1, Outputs, predictions));
    v1outputs.def_readwrite("yoloOutputs", &v1::Outputs::yoloOutputs, DOC(dai, nn_archive, v1, Outputs, yoloOutputs));
    v1outputs.def_readwrite("boxes", &v1::Outputs::boxes, DOC(dai, nn_archive, v1, Outputs, boxes));
    v1outputs.def_readwrite("scores", &v1::Outputs::scores, DOC(dai, nn_archive, v1, Outputs, scores));
    v1outputs.def_readwrite("maskOutputs", &v1::Outputs::maskOutputs, DOC(dai, nn_archive, v1, Outputs, maskOutputs));
    v1outputs.def_readwrite("protos", &v1::Outputs::protos, DOC(dai, nn_archive, v1, Outputs, protos));

    v1preprocessingBlock.def(py::init<>());
    v1preprocessingBlock.def_readwrite("interleavedToPlanar", &v1::PreprocessingBlock::interleavedToPlanar, DOC(dai, nn_archive, v1, PreprocessingBlock, interleavedToPlanar));
    v1preprocessingBlock.def_readwrite("mean", &v1::PreprocessingBlock::mean, DOC(dai, nn_archive, v1, PreprocessingBlock, mean));
    v1preprocessingBlock.def_readwrite("reverseChannels", &v1::PreprocessingBlock::reverseChannels, DOC(dai, nn_archive, v1, PreprocessingBlock, reverseChannels));
    v1preprocessingBlock.def_readwrite("scale", &v1::PreprocessingBlock::scale, DOC(dai, nn_archive, v1, PreprocessingBlock, scale));
}
