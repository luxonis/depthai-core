#pragma once

#include <spdlog/fmt/fmt.h>

#include "depthai/utility/Path.hpp"

#if FMT_VERSION >= 100000
#include <spdlog/fmt/ostr.h>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai-shared/datatype/DatatypeEnum.hpp"
#endif

namespace dai {
namespace utility {
static constexpr char path_convert_err[] = "<Unicode path not convertible>";
}
}  // namespace dai

template <>
struct fmt::formatter<dai::Path> : formatter<std::string> {
    // https://fmt.dev/latest/api.html#formatting-user-defined-types
    // https://fmt.dev/latest/syntax.html#format-specification-mini-language
    template <typename FormatContext>
    auto format(const dai::Path& p, FormatContext& ctx) {
        std::string output;
        try {
            output = p.string();
        } catch(const std::exception&) {
            output = dai::utility::path_convert_err;
        }
        return formatter<std::string>::format(output, ctx);
    }
};

#if FMT_VERSION >= 100000
template <>
struct fmt::formatter<dai::CameraBoardSocket> : ostream_formatter {};

template <>
struct fmt::formatter<dai::DatatypeEnum> : fmt::formatter<std::string> {
    auto format(dai::DatatypeEnum my, format_context& ctx) const -> decltype(ctx.out()) {
        return fmt::format_to(ctx.out(), "{}", static_cast<int32_t>(my));
    }
};
#endif
