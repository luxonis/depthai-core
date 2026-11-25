#pragma once

#include <spdlog/fmt/fmt.h>

#if FMT_VERSION >= 90000
    #include <spdlog/fmt/ostr.h>

    #include "XLink/XLink.h"

    #include <depthai/common/TensorInfo.hpp>
#endif

#if FMT_VERSION >= 90000
template <>
struct fmt::formatter<XLinkError_t> : fmt::formatter<std::string> {
    auto format(XLinkError_t error, format_context& ctx) const -> decltype(ctx.out()) {
        return fmt::format_to(ctx.out(), "{}", static_cast<int32_t>(error));
    }
};

template <>
struct fmt::formatter<dai::TensorInfo::DataType> : fmt::formatter<std::string> {
    auto format(dai::TensorInfo::DataType type, format_context& ctx) const -> decltype(ctx.out()) {
        return fmt::format_to(ctx.out(), "{}", static_cast<int>(type));
    }
};

#endif
