//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     NnArchiveConfig.cpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
#include "helper.hpp"

#include "ConfigVersion.hpp"
#include "Family.hpp"
#include "ObjectDetectionSubtypeYolo.hpp"
#include "Metadata.hpp"
#include "Head.hpp"
#include "DataType.hpp"
#include "InputType.hpp"
#include "PreprocessingBlock.hpp"
#include "Input.hpp"
#include "MetadataClass.hpp"
#include "Output.hpp"
#include "Model.hpp"
#include "NnArchiveConfig.hpp"
namespace dai {
namespace json_types {
}
}
