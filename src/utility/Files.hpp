#pragma once

#include <fmt/std.h>
#include <spdlog/spdlog.h>

#include <fstream>
#include <optional>

#include "utility/Platform.hpp"

namespace dai {

std::optional<std::string> saveFileToTemporaryDirectory(std::vector<uint8_t> data, std::string filename, std::filesystem::path path = "") {
    if(path.empty()) {
        path = platform::getTempPath();
    }
    path = dai::platform::joinPaths(path, filename);

    std::ofstream file(path, std::ios::binary);
    if(!file.is_open()) {
        spdlog::error("Couldn't open file {} for writing", path);
        return std::nullopt;
    }

    file.write(reinterpret_cast<char*>(data.data()), data.size());
    file.close();
    if(!file.good()) {
        spdlog::error("Couldn't write to file {}", path);
        return std::nullopt;
    }
    spdlog::debug("Saved file {} to {}", filename, path);
    return path.string();
}

}  // namespace dai
