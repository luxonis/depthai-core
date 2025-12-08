#pragma once

#include <spdlog/fmt/fmt.h>

#if FMT_VERSION >= 90000
    #include <spdlog/fmt/ostr.h>

    #include "depthai/common/CameraBoardSocket.hpp"
    #include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#endif

#if FMT_VERSION >= 90000
template <>
struct fmt::formatter<dai::CameraBoardSocket> : ostream_formatter {};

template <>
struct fmt::formatter<dai::DatatypeEnum> : fmt::formatter<std::string> {
    auto format(dai::DatatypeEnum my, format_context& ctx) const -> decltype(ctx.out()) {
        return fmt::format_to(ctx.out(), "{}", static_cast<int32_t>(my));
    }
};
#endif
