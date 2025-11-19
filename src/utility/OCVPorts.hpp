#include <vector>

#include "depthai/common/RotatedRect.hpp"

namespace dai {
namespace utility {

dai::RotatedRect getOuterRotatedRect(const std::vector<std::array<float, 2>>& points);

}  // namespace utility
}  // namespace dai
