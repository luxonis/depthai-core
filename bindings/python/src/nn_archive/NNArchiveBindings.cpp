#include "NNArchiveBindings.hpp"

#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/nn_archive/NNArchiveEntry.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"

// v1 nn_archive bindings
#include "depthai/nn_archive/v1/Config.hpp"
#include "depthai/nn_archive/v1/DataType.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/nn_archive/v1/Input.hpp"
#include "depthai/nn_archive/v1/InputType.hpp"
#include "depthai/nn_archive/v1/Metadata.hpp"
#include "depthai/nn_archive/v1/MetadataClass.hpp"
#include "depthai/nn_archive/v1/Model.hpp"
#include "depthai/nn_archive/v1/Output.hpp"
#include "depthai/nn_archive/v1/PreprocessingBlock.hpp"

void NNArchiveBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::nn_archive;

    // NNArchive
    py::class_<NNArchive> nnArchive(m, "NNArchive", DOC(dai, NNArchive));
    py::class_<NNArchiveOptions> nnArchiveOptions(m, "NNArchiveOptions", DOC(dai, NNArchiveOptions));

    // NNArchiveVersionedConfig
    py::class_<NNArchiveVersionedConfig> nnArchiveVersionedConfig(m, "NNArchiveVersionedConfig", DOC(dai, NNArchiveVersionedConfig));
    py::enum_<NNArchiveConfigVersion> nnArchiveConfigVersion(m, "NNArchiveConfigVersion", DOC(dai, NNArchiveConfigVersion));

    // NNArchiveEntry
    py::class_<NNArchiveEntry> nnArchiveEntry(m, "NNArchiveEntry", DOC(dai, NNArchiveEntry));
    py::enum_<NNArchiveEntry::Compression> archiveEntryCompression(nnArchiveEntry, "Compression");
    py::enum_<NNArchiveEntry::Seek> archiveEntrySeek(nnArchiveEntry, "Seek");

    // NNArchive v1
    auto v1m = m.def_submodule("nn_archive").def_submodule("v1");
    py::class_<v1::Config> v1config(v1m, "Config", DOC(dai, nn_archive, v1, Config));
    py::class_<v1::Model> v1model(v1m, "Model", DOC(dai, nn_archive, v1, Model));
    py::class_<v1::Head> v1head(v1m, "Head", DOC(dai, nn_archive, v1, Head));
    py::enum_<v1::DataType> v1dataType(v1m, "DataType", DOC(dai, nn_archive, v1, DataType));
    py::enum_<v1::InputType> v1inputType(v1m, "InputType", DOC(dai, nn_archive, v1, InputType));
    py::class_<v1::Input> v1input(v1m, "Input", DOC(dai, nn_archive, v1, Input));
    py::class_<v1::Metadata> v1metadata(v1m, "Metadata", DOC(dai, nn_archive, v1, Metadata));
    py::class_<v1::MetadataClass> v1metadataClass(v1m, "MetadataClass", DOC(dai, nn_archive, v1, MetadataClass));
    py::class_<v1::Output> v1output(v1m, "Output", DOC(dai, nn_archive, v1, Output));
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
    nnArchive.def(py::init([](const std::string& archivePath, NNArchiveEntry::Compression compression, const std::string& extractFolder) {
                      NNArchiveOptions options;
                      options.compression(compression);
                      options.extractFolder(extractFolder);
                      return NNArchive(archivePath, options);
                  }),
                  py::arg("archivePath"),
                  py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                  py::arg("extractFolder") = "/tmp/",
                  DOC(dai, NNArchive, NNArchive));
    nnArchive.def(
        py::init<const std::string&, NNArchiveOptions>(), py::arg("archivePath"), py::arg("options") = NNArchiveOptions(), DOC(dai, NNArchive, NNArchive));
    nnArchive.def("getBlob", &NNArchive::getBlob, DOC(dai, NNArchive, getBlob));
    nnArchive.def("getSuperBlob", &NNArchive::getSuperBlob, DOC(dai, NNArchive, getBlob));
    nnArchive.def("getModelPath", &NNArchive::getModelPath, DOC(dai, NNArchive, getModelPath));
    nnArchive.def("getConfig", &NNArchive::getConfig<NNArchiveConfig>, DOC(dai, NNArchive, getConfig));
    nnArchive.def("getConfigV1", &NNArchive::getConfig<v1::Config>, DOC(dai, NNArchive, getConfig));
    nnArchive.def("getModelType", &NNArchive::getModelType, DOC(dai, NNArchive, getModelType));
    nnArchive.def("getInputSize", &NNArchive::getInputSize, py::arg("index") = 0, DOC(dai, NNArchive, getInputSize));
    nnArchive.def("getInputWidth", &NNArchive::getInputWidth, py::arg("index") = 0, DOC(dai, NNArchive, getInputWidth));
    nnArchive.def("getInputHeight", &NNArchive::getInputHeight, py::arg("index") = 0, DOC(dai, NNArchive, getInputHeight));
    nnArchive.def("getSupportedPlatforms", &NNArchive::getSupportedPlatforms, DOC(dai, NNArchive, getSupportedPlatforms));

    // Bind NNArchive options
    nnArchiveOptions.def(py::init<>());
    nnArchiveOptions.def_property(
        "compression",
        [](const NNArchiveOptions& opt) { return opt.compression(); },
        [](NNArchiveOptions& opt, NNArchiveEntry::Compression compression) { opt.compression(compression); });
    nnArchiveOptions.def_property(
        "extractFolder",
        [](const NNArchiveOptions& opt) { return opt.extractFolder(); },
        [](NNArchiveOptions& opt, const std::string& extractFolder) { opt.extractFolder(extractFolder); });

    // Bind NNArchiveVersionedConfig
    nnArchiveVersionedConfig.def(py::init<const dai::Path&, NNArchiveEntry::Compression>(),
                                 py::arg("path"),
                                 py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                                 DOC(dai, NNArchiveVersionedConfig, NNArchiveVersionedConfig));
    nnArchiveVersionedConfig.def(py::init<const std::vector<uint8_t>&, NNArchiveEntry::Compression>(),
                                 py::arg("data"),
                                 py::arg("compression") = NNArchiveEntry::Compression::AUTO,
                                 DOC(dai, NNArchiveVersionedConfig, NNArchiveVersionedConfig));
    nnArchiveVersionedConfig.def(py::init([](const std::function<int()>& openCallback,
                                             const std::function<std::vector<uint8_t>()>& readCallback,
                                             const std::function<int64_t(int64_t, NNArchiveEntry::Seek)>& seekCallback,
                                             const std::function<int64_t(int64_t)>& skipCallback,
                                             const std::function<int()>& closeCallback,
                                             NNArchiveEntry::Compression compression) {
        auto readCallbackWrapper = [readCallback]() { return std::make_shared<std::vector<uint8_t>>(readCallback()); };
        return NNArchiveVersionedConfig(openCallback, readCallbackWrapper, seekCallback, skipCallback, closeCallback, compression);
    }));

    // Bind NNArchiveVersionedConfig.
    nnArchiveVersionedConfig.def("getConfig", &NNArchiveVersionedConfig::getConfig<NNArchiveConfig>, DOC(dai, NNArchiveVersionedConfig, getConfig));
    nnArchiveVersionedConfig.def("getConfigV1", &NNArchiveVersionedConfig::getConfig<v1::Config>, DOC(dai, NNArchiveVersionedConfig, getConfig));
    nnArchiveVersionedConfig.def("getVersion", &NNArchiveVersionedConfig::getVersion, DOC(dai, NNArchiveVersionedConfig, getVersion));

    // Bind NNArchiveConfigVersion
    nnArchiveConfigVersion.value("V1", NNArchiveConfigVersion::V1);

    archiveEntryCompression.value("AUTO", NNArchiveEntry::Compression::AUTO);
    archiveEntryCompression.value("RAW_FS", NNArchiveEntry::Compression::RAW_FS);
    archiveEntryCompression.value("TAR", NNArchiveEntry::Compression::TAR);
    archiveEntryCompression.value("TAR_GZ", NNArchiveEntry::Compression::TAR_GZ);
    archiveEntryCompression.value("TAR_XZ", NNArchiveEntry::Compression::TAR_XZ);

    archiveEntrySeek.value("SET", NNArchiveEntry::Seek::SET);
    archiveEntrySeek.value("CUR", NNArchiveEntry::Seek::CUR);
    archiveEntrySeek.value("END", NNArchiveEntry::Seek::END);

    v1config.def(py::init<>());
    v1config.def(py::init<std::string, v1::Model>(), py::arg("configVersion"), py::arg("model"));
    v1config.def_readwrite("configVersion", &v1::Config::configVersion, DOC(dai, nn_archive, v1, Config, configVersion));
    v1config.def_readwrite("model", &v1::Config::model, DOC(dai, nn_archive, v1, Config, model));

    v1model.def(py::init<>());
    v1model.def_readwrite("heads", &v1::Model::heads, DOC(dai, nn_archive, v1, Model, heads));
    v1model.def_readwrite("inputs", &v1::Model::inputs, DOC(dai, nn_archive, v1, Model, inputs));
    v1model.def_readwrite("metadata", &v1::Model::metadata, DOC(dai, nn_archive, v1, Model, metadata));
    v1model.def_readwrite("outputs", &v1::Model::outputs, DOC(dai, nn_archive, v1, Model, outputs));

    v1head.def(py::init<>());
    v1head.def_readwrite("name", &v1::Head::name, DOC(dai, nn_archive, v1, Head, name));
    v1head.def_readwrite("metadata", &v1::Head::metadata, DOC(dai, nn_archive, v1, Head, metadata));
    v1head.def_readwrite("outputs", &v1::Head::outputs, DOC(dai, nn_archive, v1, Head, outputs));
    v1head.def_readwrite("parser", &v1::Head::parser, DOC(dai, nn_archive, v1, Head, parser));

    v1dataType.value("BOOLEAN", v1::DataType::BOOLEAN);
    v1dataType.value("FLOAT16", v1::DataType::FLOAT16);
    v1dataType.value("FLOAT32", v1::DataType::FLOAT32);
    v1dataType.value("FLOAT64", v1::DataType::FLOAT64);
    v1dataType.value("INT4", v1::DataType::INT4);
    v1dataType.value("INT8", v1::DataType::INT8);
    v1dataType.value("INT16", v1::DataType::INT16);
    v1dataType.value("INT32", v1::DataType::INT32);
    v1dataType.value("INT64", v1::DataType::INT64);
    v1dataType.value("UINT4", v1::DataType::UINT4);
    v1dataType.value("UINT8", v1::DataType::UINT8);
    v1dataType.value("UINT16", v1::DataType::UINT16);
    v1dataType.value("UINT32", v1::DataType::UINT32);
    v1dataType.value("UINT64", v1::DataType::UINT64);
    v1dataType.value("STRING", v1::DataType::STRING);

    v1inputType.value("IMAGE", v1::InputType::IMAGE);
    v1inputType.value("RAW", v1::InputType::RAW);

    v1input.def(py::init<>());
    v1input.def_readwrite("dtype", &v1::Input::dtype, DOC(dai, nn_archive, v1, Input, dtype));
    v1input.def_readwrite("inputType", &v1::Input::inputType, DOC(dai, nn_archive, v1, Input, inputType));
    v1input.def_readwrite("layout", &v1::Input::layout, DOC(dai, nn_archive, v1, Input, layout));
    v1input.def_readwrite("name", &v1::Input::name, DOC(dai, nn_archive, v1, Input, name));
    v1input.def_readwrite("preprocessing", &v1::Input::preprocessing, DOC(dai, nn_archive, v1, Input, preprocessing));
    v1input.def_readwrite("shape", &v1::Input::shape, DOC(dai, nn_archive, v1, Input, shape));

    v1metadata.def(py::init<>());
    v1metadata.def_readwrite("postprocessorPath", &v1::Metadata::postprocessorPath, DOC(dai, nn_archive, v1, Metadata, postprocessorPath));
    v1metadata.def_readwrite("anchors", &v1::Metadata::anchors, DOC(dai, nn_archive, v1, Metadata, anchors));
    v1metadata.def_readwrite("classes", &v1::Metadata::classes, DOC(dai, nn_archive, v1, Metadata, classes));
    v1metadata.def_readwrite("confThreshold", &v1::Metadata::confThreshold, DOC(dai, nn_archive, v1, Metadata, confThreshold));
    v1metadata.def_readwrite("iouThreshold", &v1::Metadata::iouThreshold, DOC(dai, nn_archive, v1, Metadata, iouThreshold));
    v1metadata.def_readwrite("maxDet", &v1::Metadata::maxDet, DOC(dai, nn_archive, v1, Metadata, maxDet));
    v1metadata.def_readwrite("nClasses", &v1::Metadata::nClasses, DOC(dai, nn_archive, v1, Metadata, nClasses));
    v1metadata.def_readwrite("isSoftmax", &v1::Metadata::isSoftmax, DOC(dai, nn_archive, v1, Metadata, isSoftmax));
    v1metadata.def_readwrite("boxesOutputs", &v1::Metadata::boxesOutputs, DOC(dai, nn_archive, v1, Metadata, boxesOutputs));
    v1metadata.def_readwrite("scoresOutputs", &v1::Metadata::scoresOutputs, DOC(dai, nn_archive, v1, Metadata, scoresOutputs));
    v1metadata.def_readwrite("anglesOutputs", &v1::Metadata::anglesOutputs, DOC(dai, nn_archive, v1, Metadata, anglesOutputs));
    v1metadata.def_readwrite("keypointsOutputs", &v1::Metadata::keypointsOutputs, DOC(dai, nn_archive, v1, Metadata, keypointsOutputs));
    v1metadata.def_readwrite("maskOutputs", &v1::Metadata::maskOutputs, DOC(dai, nn_archive, v1, Metadata, maskOutputs));
    v1metadata.def_readwrite("nKeypoints", &v1::Metadata::nKeypoints, DOC(dai, nn_archive, v1, Metadata, nKeypoints));
    v1metadata.def_readwrite("nPrototypes", &v1::Metadata::nPrototypes, DOC(dai, nn_archive, v1, Metadata, nPrototypes));
    v1metadata.def_readwrite("protosOutputs", &v1::Metadata::protosOutputs, DOC(dai, nn_archive, v1, Metadata, protosOutputs));
    v1metadata.def_readwrite("subtype", &v1::Metadata::subtype, DOC(dai, nn_archive, v1, Metadata, subtype));
    v1metadata.def_readwrite("yoloOutputs", &v1::Metadata::yoloOutputs, DOC(dai, nn_archive, v1, Metadata, yoloOutputs));
    v1metadata.def_readwrite("extraParams", &v1::Metadata::extraParams, DOC(dai, nn_archive, v1, Metadata, extraParams));

    v1metadataClass.def(py::init<>());
    v1metadataClass.def_readwrite("name", &v1::MetadataClass::name, DOC(dai, nn_archive, v1, MetadataClass, name));
    v1metadataClass.def_readwrite("path", &v1::MetadataClass::path, DOC(dai, nn_archive, v1, MetadataClass, path));
    v1metadataClass.def_readwrite("precision", &v1::MetadataClass::precision, DOC(dai, nn_archive, v1, MetadataClass, precision));

    v1output.def(py::init<>());
    v1output.def_readwrite("dtype", &v1::Output::dtype, DOC(dai, nn_archive, v1, Output, dtype));
    v1output.def_readwrite("name", &v1::Output::name, DOC(dai, nn_archive, v1, Output, name));
    v1output.def_readwrite("layout", &v1::Output::layout, DOC(dai, nn_archive, v1, Output, layout));
    v1output.def_readwrite("shape", &v1::Output::shape, DOC(dai, nn_archive, v1, Output, shape));

    v1preprocessingBlock.def(py::init<>());
    v1preprocessingBlock.def_readwrite(
        "interleavedToPlanar", &v1::PreprocessingBlock::interleavedToPlanar, DOC(dai, nn_archive, v1, PreprocessingBlock, interleavedToPlanar));
    v1preprocessingBlock.def_readwrite("mean", &v1::PreprocessingBlock::mean, DOC(dai, nn_archive, v1, PreprocessingBlock, mean));
    v1preprocessingBlock.def_readwrite(
        "reverseChannels", &v1::PreprocessingBlock::reverseChannels, DOC(dai, nn_archive, v1, PreprocessingBlock, reverseChannels));
    v1preprocessingBlock.def_readwrite("scale", &v1::PreprocessingBlock::scale, DOC(dai, nn_archive, v1, PreprocessingBlock, scale));
    v1preprocessingBlock.def_readwrite("daiType", &v1::PreprocessingBlock::daiType, DOC(dai, nn_archive, v1, PreprocessingBlock, daiType));
}
