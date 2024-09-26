#pragma once

#include <depthai/device/Device.hpp>
#include <depthai/openvino/OpenVINO.hpp>
#include <memory>
#include <string>
#include <variant>
#include <vector>
#include <optional>
#include <depthai/nn_archive/NNArchiveVersionedConfig.hpp>

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
    std::vector<dai::Platform> supportedPlatforms = {};

    virtual bool isValid() const {
        return true;
    }

    bool isSupported(dai::Platform platform) const {
        return std::find(supportedPlatforms.begin(), supportedPlatforms.end(), platform) != supportedPlatforms.end();
    }

    bool isSupported(const dai::Device& device) const {
        return isSupported(device.getPlatform());
    }

    virtual ~ModelSettings() = default;

    // Archive config for NNArchive models
    std::optional<dai::NNArchiveVersionedConfig> nnArchiveConfig;
};

struct BlobSettings : ModelSettings {
};

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

    BlobModel(std::shared_ptr<dai::OpenVINO::Blob> model, std::shared_ptr<BlobSettings> settings) : modelPtr_(model), settingsPtr_(settings) {}

    inline const dai::OpenVINO::Blob& model() const {
        return *modelPtr_;
    }

    inline const BlobSettings& settings() const {
        return *settingsPtr_;
    }

   private:
    std::shared_ptr<dai::OpenVINO::Blob> modelPtr_;
    std::shared_ptr<BlobSettings> settingsPtr_;
};

class SuperBlobModel : public Model {
   public:
    ModelType type() const override {
        return ModelType::SUPERBLOB;
    }

    SuperBlobModel(std::shared_ptr<dai::OpenVINO::SuperBlob> model, std::shared_ptr<SuperBlobSettings> settings) : modelPtr_(model), settingsPtr_(settings) {}

    inline const dai::OpenVINO::SuperBlob& model() const {
        return *modelPtr_;
    }

    inline const SuperBlobSettings& settings() const {
        return *settingsPtr_;
    }

   private:
    std::shared_ptr<dai::OpenVINO::SuperBlob> modelPtr_;
    std::shared_ptr<SuperBlobSettings> settingsPtr_;
};

class DlcModel : public Model {
   public:
    ModelType type() const override {
        return ModelType::DLC;
    }

    DlcModel(const std::string& modelPath, std::shared_ptr<DlcSettings> settings) : modelPath_(modelPath), settingsPtr_(settings) {}

    inline const std::string& modelPath() const {
        return modelPath_;
    }

    inline const DlcSettings& settings() const {
        return *settingsPtr_;
    }

   private:
    std::string modelPath_;
    std::shared_ptr<DlcSettings> settingsPtr_;
};

using ModelVariant = std::variant<BlobModel, SuperBlobModel, DlcModel>;

// Helper functions
ModelType getModelType(const ModelVariant& model);

}  // namespace model
}  // namespace depthai
