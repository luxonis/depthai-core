#pragma once

#include <vector>
#include <unordered_map>
#include <optional>

#include <depthai/models/Models.hpp>

namespace depthai {
namespace model {

class ModelLoader {
	public:

        ModelLoader() = default;

		// setters
		void loadModelFromPath(const std::string &path);
		void loadModelFromBytes(std::vector<uint8_t> bytes, ModelType type);

        bool isModelLoaded() const;
		
		ModelType getModelType() const;
		ModelVariant getModelVariant() const;
		
		static ModelType getModelTypeFromFilePath(const std::string &path); // Opens NNArchive and checks
		static std::unordered_map<ModelType, std::vector<std::string>> getSupportedModelFileExtensions();

	private:
			
		void loadBlobModel(std::vector<uint8_t> data);
		void loadBlobModel(const std::string &path);
		
		void loadSuperblobModel(std::vector<uint8_t> data);
		void loadSuperblobModel(const std::string &path);
		
		void loadDlcModel(std::vector<uint8_t> data);
		void loadDlcModel(const std::string &path);
		
		void loadNNArchive(std::vector<uint8_t> data);
		void loadNNArchive(const std::string &path);

        ModelType modelType_;
        std::optional<ModelVariant> modelVariant_;
		
};

ModelVariant load(const std::string& modelPath);
ModelVariant load(const std::vector<uint8_t>& data, ModelType type);

}
}