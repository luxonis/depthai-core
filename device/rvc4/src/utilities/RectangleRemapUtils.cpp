#include "RectangleRemapUtils.hpp"

// extern bool hasIMX214; // TMP

namespace dai {
namespace gate {
namespace RectangleRemapUtils {

void clampRect(dai::Rect& toClamp, float min, float max) {
    toClamp.x = std::max(min, toClamp.x);
    toClamp.x = std::min(max, toClamp.x);

    toClamp.y = std::max(min, toClamp.y);
    toClamp.y = std::min(max, toClamp.y);

    toClamp.width = std::max(min, toClamp.width);
    toClamp.width = std::min(max - toClamp.x, toClamp.width);

    toClamp.height = std::max(min, toClamp.height);
    toClamp.height = std::min(max - toClamp.y, toClamp.height);
}

double radians(double degrees) {
    return degrees * (M_PI / 180.0);
}

bool validRect(dai::Rect rect) {
    if(rect.topLeft().x < 0 || rect.topLeft().x > 1) return false;
    if(rect.bottomRight().x < 0 || rect.bottomRight().x > 1) return false;
    if(rect.topLeft().y < 0 || rect.topLeft().y > 1) return false;
    if(rect.bottomRight().y < 0 || rect.bottomRight().y > 1) return false;
    if(rect.topLeft().x >= rect.bottomRight().x) return false;
    if(rect.topLeft().y >= rect.bottomRight().y) return false;
    return true;
}

bool validFrameRemapping(std::shared_ptr<dai::ImgFrame> imgFrame) {
    if(imgFrame->getSourceHeight() == 0) return false;
    if(imgFrame->getSourceWidth() == 0) return false;
    if(imgFrame->getSourceHFov() == 0.0f) return false;
    return true;
}

dai::Rect scaleRect(dai::Rect toScale, float scaleFactor) {
    dai::Point2f topLeft = toScale.topLeft();
    dai::Point2f bottomRight = toScale.bottomRight();

    float middle_x = (bottomRight.x + topLeft.x) / 2;
    float middle_y = (bottomRight.y + topLeft.y) / 2;
    float width = toScale.width * scaleFactor;
    float height = toScale.height * scaleFactor;

    float xmin = std::max(0.f, middle_x - width / 2);
    float ymin = std::max(0.f, middle_y - height / 2);

    return dai::Rect(xmin, ymin, width, height);
}

}  // namespace RectangleRemapUtils
}  // namespace gate
}  // namespace dai
