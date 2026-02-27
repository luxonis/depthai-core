#pragma once

#include <fmt/format.h>

#if __has_include(<fmt/std.h>)
#include "utility/FmtCompatibility.hpp"
#else
#include <filesystem>
#include <string_view>

template <>
struct fmt::formatter<std::filesystem::path> : fmt::formatter<std::string_view> {
    template <typename FormatContext>
    auto format(const std::filesystem::path& path, FormatContext& ctx) const {
        const std::string pathString = path.string();
        return fmt::formatter<std::string_view>::format(pathString, ctx);
    }
};

#endif
