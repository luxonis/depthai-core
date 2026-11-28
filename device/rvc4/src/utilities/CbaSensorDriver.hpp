#pragma once

#include <optional>
#include <string_view>

namespace dai {
namespace gate {

std::optional<std::string_view> cbaSensorDriverForBoardConf(std::string_view boardConf);  // e.g. nIR-C54M00-10 gives og05b1b; empty for boards not in the table

}  // namespace gate
}  // namespace dai
