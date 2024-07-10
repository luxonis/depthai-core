//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     Generators.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
#include "helper.hpp"

#include "depthai/nn_archive/v1/Config.hpp"
#include "depthai/nn_archive/v1/Model.hpp"
#include "depthai/nn_archive/v1/Output.hpp"
#include "depthai/nn_archive/v1/MetadataClass.hpp"
#include "depthai/nn_archive/v1/Input.hpp"
#include "depthai/nn_archive/v1/PreprocessingBlock.hpp"
#include "depthai/nn_archive/v1/InputType.hpp"
#include "depthai/nn_archive/v1/DataType.hpp"
#include "depthai/nn_archive/v1/Head.hpp"
#include "depthai/nn_archive/v1/Metadata.hpp"
#include "depthai/nn_archive/v1/ObjectDetectionSubtypeYolo.hpp"
#include "depthai/nn_archive/v1/ConfigVersion.hpp"

namespace dai {
namespace nn_archive {
namespace v1 {
    void from_json(const json & j, Metadata & x);
    void to_json(json & j, const Metadata & x);

    void from_json(const json & j, Head & x);
    void to_json(json & j, const Head & x);

    void from_json(const json & j, PreprocessingBlock & x);
    void to_json(json & j, const PreprocessingBlock & x);

    void from_json(const json & j, Input & x);
    void to_json(json & j, const Input & x);

    void from_json(const json & j, MetadataClass & x);
    void to_json(json & j, const MetadataClass & x);

    void from_json(const json & j, Output & x);
    void to_json(json & j, const Output & x);

    void from_json(const json & j, Model & x);
    void to_json(json & j, const Model & x);

    void from_json(const json & j, Config & x);
    void to_json(json & j, const Config & x);

    void from_json(const json & j, ConfigVersion & x);
    void to_json(json & j, const ConfigVersion & x);

    void from_json(const json & j, ObjectDetectionSubtypeYolo & x);
    void to_json(json & j, const ObjectDetectionSubtypeYolo & x);

    void from_json(const json & j, DataType & x);
    void to_json(json & j, const DataType & x);

    void from_json(const json & j, InputType & x);
    void to_json(json & j, const InputType & x);

    inline void from_json(const json & j, Metadata& x) {
        x.postprocessorPath = get_stack_optional<std::string>(j, "postprocessor_path");
        x.anchors = get_stack_optional<std::vector<std::vector<std::vector<double>>>>(j, "anchors");
        x.classes = get_stack_optional<std::vector<std::string>>(j, "classes");
        x.confThreshold = get_stack_optional<double>(j, "conf_threshold");
        x.iouThreshold = get_stack_optional<double>(j, "iou_threshold");
        x.maxDet = get_stack_optional<int64_t>(j, "max_det");
        x.nClasses = get_stack_optional<int64_t>(j, "n_classes");
        x.isSoftmax = get_stack_optional<bool>(j, "is_softmax");
        x.boxesOutputs = get_stack_optional<std::string>(j, "boxes_outputs");
        x.scoresOutputs = get_stack_optional<std::string>(j, "scores_outputs");
        x.anglesOutputs = get_stack_optional<std::vector<std::string>>(j, "angles_outputs");
        x.keypointsOutputs = get_stack_optional<std::vector<std::string>>(j, "keypoints_outputs");
        x.maskOutputs = get_stack_optional<std::vector<std::string>>(j, "mask_outputs");
        x.nKeypoints = get_stack_optional<int64_t>(j, "n_keypoints");
        x.nPrototypes = get_stack_optional<int64_t>(j, "n_prototypes");
        x.protosOutputs = get_stack_optional<std::string>(j, "protos_outputs");
        x.subtype = get_stack_optional<ObjectDetectionSubtypeYolo>(j, "subtype");
        x.yoloOutputs = get_stack_optional<std::vector<std::string>>(j, "yolo_outputs");
    }

    inline void to_json(json & j, const Metadata & x) {
        j = json::object();
        j["postprocessor_path"] = x.postprocessorPath;
        j["anchors"] = x.anchors;
        j["classes"] = x.classes;
        j["conf_threshold"] = x.confThreshold;
        j["iou_threshold"] = x.iouThreshold;
        j["max_det"] = x.maxDet;
        j["n_classes"] = x.nClasses;
        j["is_softmax"] = x.isSoftmax;
        j["boxes_outputs"] = x.boxesOutputs;
        j["scores_outputs"] = x.scoresOutputs;
        j["angles_outputs"] = x.anglesOutputs;
        j["keypoints_outputs"] = x.keypointsOutputs;
        j["mask_outputs"] = x.maskOutputs;
        j["n_keypoints"] = x.nKeypoints;
        j["n_prototypes"] = x.nPrototypes;
        j["protos_outputs"] = x.protosOutputs;
        j["subtype"] = x.subtype;
        j["yolo_outputs"] = x.yoloOutputs;
    }

    inline void from_json(const json & j, Head& x) {
        x.metadata = j.at("metadata").get<Metadata>();
        x.outputs = get_stack_optional<std::vector<std::string>>(j, "outputs");
        x.parser = j.at("parser").get<std::string>();
    }

    inline void to_json(json & j, const Head & x) {
        j = json::object();
        j["metadata"] = x.metadata;
        j["outputs"] = x.outputs;
        j["parser"] = x.parser;
    }

    inline void from_json(const json & j, PreprocessingBlock& x) {
        x.interleavedToPlanar = get_stack_optional<bool>(j, "interleaved_to_planar");
        x.mean = get_stack_optional<std::vector<double>>(j, "mean");
        x.reverseChannels = get_stack_optional<bool>(j, "reverse_channels");
        x.scale = get_stack_optional<std::vector<double>>(j, "scale");
    }

    inline void to_json(json & j, const PreprocessingBlock & x) {
        j = json::object();
        j["interleaved_to_planar"] = x.interleavedToPlanar;
        j["mean"] = x.mean;
        j["reverse_channels"] = x.reverseChannels;
        j["scale"] = x.scale;
    }

    inline void from_json(const json & j, Input& x) {
        x.dtype = j.at("dtype").get<DataType>();
        x.inputType = j.at("input_type").get<InputType>();
        x.layout = get_stack_optional<std::string>(j, "layout");
        x.name = j.at("name").get<std::string>();
        x.preprocessing = j.at("preprocessing").get<PreprocessingBlock>();
        x.shape = j.at("shape").get<std::vector<int64_t>>();
    }

    inline void to_json(json & j, const Input & x) {
        j = json::object();
        j["dtype"] = x.dtype;
        j["input_type"] = x.inputType;
        j["layout"] = x.layout;
        j["name"] = x.name;
        j["preprocessing"] = x.preprocessing;
        j["shape"] = x.shape;
    }

    inline void from_json(const json & j, MetadataClass& x) {
        x.name = j.at("name").get<std::string>();
        x.path = j.at("path").get<std::string>();
        x.precision = get_stack_optional<DataType>(j, "precision");
    }

    inline void to_json(json & j, const MetadataClass & x) {
        j = json::object();
        j["name"] = x.name;
        j["path"] = x.path;
        j["precision"] = x.precision;
    }

    inline void from_json(const json & j, Output& x) {
        x.dtype = j.at("dtype").get<DataType>();
        x.name = j.at("name").get<std::string>();
    }

    inline void to_json(json & j, const Output & x) {
        j = json::object();
        j["dtype"] = x.dtype;
        j["name"] = x.name;
    }

    inline void from_json(const json & j, Model& x) {
        x.heads = get_stack_optional<std::vector<Head>>(j, "heads");
        x.inputs = j.at("inputs").get<std::vector<Input>>();
        x.metadata = j.at("metadata").get<MetadataClass>();
        x.outputs = j.at("outputs").get<std::vector<Output>>();
    }

    inline void to_json(json & j, const Model & x) {
        j = json::object();
        j["heads"] = x.heads;
        j["inputs"] = x.inputs;
        j["metadata"] = x.metadata;
        j["outputs"] = x.outputs;
    }

    inline void from_json(const json & j, Config& x) {
        x.configVersion = j.at("config_version").get<ConfigVersion>();
        x.model = j.at("model").get<Model>();
    }

    inline void to_json(json & j, const Config & x) {
        j = json::object();
        j["config_version"] = x.configVersion;
        j["model"] = x.model;
    }

    inline void from_json(const json & j, ConfigVersion & x) {
        if (j == "1.0") x = ConfigVersion::THE_10;
        else { throw std::runtime_error("Input JSON does not conform to schema!"); }
    }

    inline void to_json(json & j, const ConfigVersion & x) {
        switch (x) {
            case ConfigVersion::THE_10: j = "1.0"; break;
            default: throw std::runtime_error("Unexpected value in enumeration \"[object Object]\": " + std::to_string(static_cast<int>(x)));
        }
    }

    inline void from_json(const json & j, ObjectDetectionSubtypeYolo & x) {
        if (j == "yolov10") x = ObjectDetectionSubtypeYolo::YOLOV10;
        else if (j == "yolov5") x = ObjectDetectionSubtypeYolo::YOLOV5;
        else if (j == "yolov6") x = ObjectDetectionSubtypeYolo::YOLOV6;
        else if (j == "yolov6r2") x = ObjectDetectionSubtypeYolo::YOLOV6_R2;
        else if (j == "yolov7") x = ObjectDetectionSubtypeYolo::YOLOV7;
        else if (j == "yolov8") x = ObjectDetectionSubtypeYolo::YOLOV8;
        else { throw std::runtime_error("Input JSON does not conform to schema!"); }
    }

    inline void to_json(json & j, const ObjectDetectionSubtypeYolo & x) {
        switch (x) {
            case ObjectDetectionSubtypeYolo::YOLOV10: j = "yolov10"; break;
            case ObjectDetectionSubtypeYolo::YOLOV5: j = "yolov5"; break;
            case ObjectDetectionSubtypeYolo::YOLOV6: j = "yolov6"; break;
            case ObjectDetectionSubtypeYolo::YOLOV6_R2: j = "yolov6r2"; break;
            case ObjectDetectionSubtypeYolo::YOLOV7: j = "yolov7"; break;
            case ObjectDetectionSubtypeYolo::YOLOV8: j = "yolov8"; break;
            default: throw std::runtime_error("Unexpected value in enumeration \"[object Object]\": " + std::to_string(static_cast<int>(x)));
        }
    }

    inline void from_json(const json & j, DataType & x) {
        if (j == "float16") x = DataType::FLOAT16;
        else if (j == "float32") x = DataType::FLOAT32;
        else if (j == "int32") x = DataType::INT32;
        else if (j == "int8") x = DataType::INT8;
        else if (j == "uint8") x = DataType::UINT8;
        else { throw std::runtime_error("Input JSON does not conform to schema!"); }
    }

    inline void to_json(json & j, const DataType & x) {
        switch (x) {
            case DataType::FLOAT16: j = "float16"; break;
            case DataType::FLOAT32: j = "float32"; break;
            case DataType::INT32: j = "int32"; break;
            case DataType::INT8: j = "int8"; break;
            case DataType::UINT8: j = "uint8"; break;
            default: throw std::runtime_error("Unexpected value in enumeration \"[object Object]\": " + std::to_string(static_cast<int>(x)));
        }
    }

    inline void from_json(const json & j, InputType & x) {
        if (j == "image") x = InputType::IMAGE;
        else if (j == "raw") x = InputType::RAW;
        else { throw std::runtime_error("Input JSON does not conform to schema!"); }
    }

    inline void to_json(json & j, const InputType & x) {
        switch (x) {
            case InputType::IMAGE: j = "image"; break;
            case InputType::RAW: j = "raw"; break;
            default: throw std::runtime_error("Unexpected value in enumeration \"[object Object]\": " + std::to_string(static_cast<int>(x)));
        }
    }
}
}
}
