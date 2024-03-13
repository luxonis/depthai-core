#pragma once

#include <spdlog/spdlog.h>

#include <fstream>
#include <optional>

#include "Platform.hpp"

namespace dai {

std::optional<std::string> saveFileToTemporaryDirectory(std::vector<uint8_t> data, std::string filename, std::string fpath = "") {
    if(fpath.empty()) {
        fpath = platform::getTempPath();
    }
    std::string path = std::string(fpath);
    if(path.back() != '/' && path.back() != '\\') {
        path += '/';
    }
    path += filename;

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
    return std::string(path);
}

}  // namespace dai
