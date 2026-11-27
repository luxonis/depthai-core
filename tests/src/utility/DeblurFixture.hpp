#pragma once
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "../../../device/rvc4/src/utilities/Deblur.hpp"
#include "../../../device/rvc4/src/utilities/OpticalFlowOutput.hpp"

namespace deblur_test {
inline std::shared_ptr<dai::ImgFrame> image(const cv::Mat& pixels, int sequence, int milliseconds) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(pixels.cols).setHeight(pixels.rows).setType(dai::ImgFrame::Type::GRAY8);
    frame->setData(std::vector<std::uint8_t>(pixels.data, pixels.data + pixels.total()));
    frame->setSequenceNum(sequence);
    frame->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(milliseconds)));
    frame->setTimestamp(frame->getTimestampDevice());
    return frame;
}
inline cv::Mat texture(int width, int height) {
    cv::Mat result(height, width, CV_8U);
    for(int y = 0; y < height; ++y)
        for(int x = 0; x < width; ++x)
            result.at<std::uint8_t>(y, x) =
                cv::saturate_cast<std::uint8_t>(128 + 35 * std::sin(x * 0.65) + 25 * std::cos(y * 0.38) + 20 * std::sin(x * 0.27 + y * 0.19));
    return result;
}
struct Fixture {
    std::shared_ptr<dai::ImgFrame> reference, target;
    std::shared_ptr<dai::MessageGroup> packet;
    cv::Mat sharp, blurred;
};
// Two independent moving regions and a stationary background, all generated.
inline Fixture make(int width = 256, int height = 128, int sequence = 10, int milliseconds = 1000) {
    Fixture result;
    result.sharp = texture(width, height);
    cv::Mat horizontal, vertical;
    cv::filter2D(result.sharp, horizontal, -1, dai::gate::deblur::linePsf({4, 0}));
    cv::filter2D(result.sharp, vertical, -1, dai::gate::deblur::linePsf({0, 3}));
    result.blurred = result.sharp.clone();
    const cv::Rect a(16, 16, width / 3, height - 32), b(width / 2, 16, width / 3, height - 32);
    horizontal(a).copyTo(result.blurred(a));
    vertical(b).copyTo(result.blurred(b));
    cv::Mat before = result.blurred.clone();
    std::vector<std::int16_t> f(width * height * 2, 0), back(f.size(), 0);
    std::vector<std::uint8_t> score(width * height, 255);
    for(const auto& region : std::vector<std::pair<cv::Rect, cv::Point>>{{a, {8, 0}}, {b, {0, 6}}}) {
        const auto rect = region.first;
        const auto motion = region.second;
        for(int y = rect.y; y < rect.y + rect.height; ++y)
            for(int x = rect.x; x < rect.x + rect.width; ++x) {
                const int sx = x - motion.x, sy = y - motion.y;
                before.at<std::uint8_t>(sy, sx) = result.blurred.at<std::uint8_t>(y, x);
                back[2 * (y * width + x)] = -16 * motion.x;
                back[2 * (y * width + x) + 1] = -16 * motion.y;
                f[2 * (sy * width + sx)] = 16 * motion.x;
                f[2 * (sy * width + sx) + 1] = 16 * motion.y;
            }
    }
    result.reference = image(before, sequence, milliseconds);
    result.target = image(result.blurred, sequence + 4, milliseconds + 40);
    result.packet = dai::gate::makeOpticalFlowOutput(f.data(), back.data(), score.data(), *result.reference, *result.target);
    return result;
}
inline dai::DeblurProperties config() {
    dai::DeblurProperties config;
    config.exposureTimeUs = 20000;  // 1/2 of actual 40 ms pair interval
    config.useOpenCL = false;
    return config;
}
}  // namespace deblur_test
