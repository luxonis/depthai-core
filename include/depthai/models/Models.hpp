#pragma once

#include <depthai/openvino/OpenVINO.hpp>
#include <string>
#include <variant>

namespace depthai {
namespace model {

enum class ModelType {
    UNKNOWN = 0,  // Unknown model type
    BLOB,
    SUPERBLOB,
    DLC,
    NNARCHIVE  // Helper type for NNArchive containing one of the above models
};

struct ModelSettings {
    std::string modelName = "";

    virtual bool isValid() const {
        return true;
    }
    virtual ~ModelSettings() = default;
};

struct BlobSettings : ModelSettings {};

struct SuperBlobSettings : BlobSettings {
    int numShaves = 6;
};

struct DlcSettings : ModelSettings {};

class Model {
   public:
    virtual ModelType type() const = 0;
    virtual ~Model() = default;
};

class BlobModel : public Model {
   public:
    ModelType type() const override {
        return ModelType::BLOB;
    }
    BlobModel(dai::OpenVINO::Blob model, BlobSettings settings) : model_(model), settings_(settings) {}

    const dai::OpenVINO::Blob& model() const {
        return model_;
    }

    const BlobSettings& settings() const {
        return settings_;
    }

   private:
    dai::OpenVINO::Blob model_;
    BlobSettings settings_;
};

class SuperBlobModel : public Model {
   public:
    ModelType type() const override {
        return ModelType::SUPERBLOB;
    }

    SuperBlobModel(dai::OpenVINO::SuperBlob model, SuperBlobSettings settings) : model_(model), settings_(settings) {}

    const dai::OpenVINO::SuperBlob& model() const {
        return model_;
    }

    const SuperBlobSettings& settings() const {
        return settings_;
    }

   private:
    dai::OpenVINO::SuperBlob model_;
    SuperBlobSettings settings_;
};

class DlcModel : public Model {
   public:
    ModelType type() const override {
        return ModelType::DLC;
    }

    DlcModel(const std::string& modelPath, DlcSettings settings) : modelPath_(modelPath), settings_(settings) {}

   private:
    std::string modelPath_;
    DlcSettings settings_;
};

using ModelVariant = std::variant<BlobModel, SuperBlobModel, DlcModel>;

// Helper functions
ModelType getModelType(const ModelVariant& model);

}  // namespace model
}  // namespace depthai
