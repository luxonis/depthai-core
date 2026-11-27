#include "Deblur.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

#include "../../../../src/utility/DeblurValidation.hpp"
#include "DeblurSync.hpp"
#include "OpticalFlowInput.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai {
namespace gate {
namespace deblur {
void validateImage(const ImgFrame& image) {
    validateOpticalFlowInput(image, nullptr);
    if(image.getType() != ImgFrame::Type::GRAY8) throw std::invalid_argument("Deblur requires packed GRAY8 images");
}
cv::Mat copyImage(const ImgFrame& image) {
    validateImage(image);
    cv::Mat result(image.getHeight(), image.getWidth(), CV_8U);
    const auto data = image.getData();
    std::memcpy(result.data, data.data(), result.total());
    return result;
}
namespace {
cv::Mat tensor(const NNData& flow, const char* name, int width, int height, int channels, bool bytes) {
    const auto info = flow.getTensorInfo(name);
    const auto type = bytes ? TensorInfo::DataType::U8F : TensorInfo::DataType::FP32;
    const auto element = bytes ? 1U : 4U;
    const std::vector<unsigned int> dims{static_cast<unsigned int>(height), static_cast<unsigned int>(width), static_cast<unsigned int>(channels)};
    const std::vector<unsigned int> strides{static_cast<unsigned int>(width * channels) * element, static_cast<unsigned int>(channels) * element, element};
    if(!info || info->dataType != type || info->order != TensorInfo::StorageOrder::HWC || info->numDimensions != 3 || info->dims != dims
       || info->strides != strides || flow.batchSize != 1)
        throw std::invalid_argument("Invalid Deblur flow tensor layout");
    const auto data = flow.getData();
    const auto size = static_cast<std::size_t>(width) * height * channels * element;
    if(info->offset > data.size() || size > data.size() - info->offset) throw std::invalid_argument("Truncated Deblur flow tensor");
    cv::Mat result(height, width, CV_MAKETYPE(bytes ? CV_8U : CV_32F, channels));
    std::memcpy(result.data, data.data() + info->offset, size);
    return result;
}
float length(cv::Point2f p) {
    return std::hypot(p.x, p.y);
}
int reflect(int p, int n) {
    while(p < 0 || p >= n) {
        if(p < 0) p = -p;
        if(p >= n) p = 2 * n - 2 - p;
    }
    return p;
}
}  // namespace
cv::Mat linePsf(cv::Point2f vector) {
    if(!std::isfinite(vector.x) || !std::isfinite(vector.y) || length(vector) > 32) throw std::invalid_argument("Invalid Deblur PSF vector");
    const int radius = static_cast<int>(std::ceil(std::max(std::abs(vector.x), std::abs(vector.y)) / 2)) + 1;
    cv::Mat psf = cv::Mat::zeros(2 * radius + 1, 2 * radius + 1, CV_32F);
    const int count = std::max(1, static_cast<int>(std::ceil(length(vector) * 16)));
    for(int i = 0; i < count; ++i) {
        const float t = (i + 0.5f) / count - 0.5f;
        const float x = radius + t * vector.x, y = radius + t * vector.y;
        const int ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
        const float fx = x - ix, fy = y - iy;
        for(int dy = 0; dy < 2; ++dy)
            for(int dx = 0; dx < 2; ++dx) psf.at<float>(iy + dy, ix + dx) += (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy) / count;
    }
    return psf;
}
cv::Mat inverseKernel(cv::Point2f vector, const DeblurProperties& config) {
    dai::utility::validateDeblurConfig(config);
    constexpr int size = 128;
    const auto psf = linePsf(vector);
    cv::Mat padded = cv::Mat::zeros(size, size, CV_32F);
    for(int y = 0; y < psf.rows; ++y)
        for(int x = 0; x < psf.cols; ++x) padded.at<float>((y - psf.rows / 2 + size) % size, (x - psf.cols / 2 + size) % size) = psf.at<float>(y, x);
    cv::Mat spectrum;
    cv::dft(padded, spectrum, cv::DFT_COMPLEX_OUTPUT);
    for(int y = 0; y < size; ++y)
        for(int x = 0; x < size; ++x) {
            auto& v = spectrum.at<cv::Vec2f>(y, x);
            const float power = v.dot(v);
            v = cv::Vec2f(v[0], -v[1]) / (power + config.regularization);
            const float amp = std::sqrt(v.dot(v));
            if(amp > config.maxGain) v *= config.maxGain / amp;
        }
    spectrum.at<cv::Vec2f>(0, 0) = {1, 0};
    cv::Mat impulse;
    cv::idft(spectrum, impulse, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    const int r = static_cast<int>(config.kernelRadius), ks = 2 * r + 1;
    cv::Mat result(ks, ks, CV_32F);
    for(int y = 0; y < ks; ++y)
        for(int x = 0; x < ks; ++x) {
            const double wy = 0.5 - 0.5 * std::cos(2 * CV_PI * (y + 1) / (ks + 1)), wx = 0.5 - 0.5 * std::cos(2 * CV_PI * (x + 1) / (ks + 1));
            result.at<float>(y, x) = static_cast<float>(wy * wx) * impulse.at<float>((y - r + size) % size, (x - r + size) % size);
        }
    result.at<float>(r, r) += 1 - static_cast<float>(cv::sum(result)[0]);
    return result;
}
Field prepare(const ImgFrame& reference, const ImgFrame& target, const MessageGroup& packet, const DeblurProperties& config) {
    dai::utility::validateDeblurConfig(config);
    Field result;
    const auto current = copyImage(target), previous = copyImage(reference);
    current.convertTo(result.image, CV_32F);
    result.labels = cv::Mat(current.size(), CV_32S, cv::Scalar(-1));
    result.blend = cv::Mat::zeros(current.size(), CV_32F);
    const auto ref = member<Buffer>(packet, "reference"), cur = member<Buffer>(packet, "target");
    const auto flow = member<NNData>(packet, "flow");
    if(!ref || !cur || !flow || !sameFrame(*ref, reference) || !sameFrame(*cur, target) || !sameFrame(packet, target) || !sameFrame(*flow, target)
       || previous.size() != current.size())
        return result;
    const double dt = std::chrono::duration<double>(cur->getTimestampDevice() - ref->getTimestampDevice()).count();
    const double exposure = config.exposureTimeUs ? config.exposureTimeUs * 1e-6 : target.getExposureTime().count() * 1e-6;
    if(dt <= 0 || exposure <= 0 || exposure > dt || config.strength == 0) return result;
    const float ratio = static_cast<float>(exposure / dt);
    cv::Mat forward, backward, score;
    try {
        forward = tensor(*flow, "forward", current.cols, current.rows, 2, false);
        backward = tensor(*flow, "backward", current.cols, current.rows, 2, false);
        score = tensor(*flow, "confidence", current.cols, current.rows, 1, true);
    } catch(const std::invalid_argument&) {
        return result;
    }
    cv::Mat before, after, mean, meanSquare, variance;
    previous.convertTo(before, CV_32F);
    cv::GaussianBlur(before, before, {0, 0}, 1);
    cv::GaussianBlur(result.image, after, {0, 0}, 1);
    cv::boxFilter(result.image, mean, -1, {7, 7});
    cv::boxFilter(result.image.mul(result.image), meanSquare, -1, {7, 7});
    variance = meanSquare - mean.mul(mean);
    using Bin = std::pair<int, int>;
    std::map<Bin, int> counts;
    cv::Mat bins(current.size(), CV_32SC2, cv::Scalar(0, 0));
    cv::Mat valid = cv::Mat::zeros(current.size(), CV_8U);
    // Bilinear reference sampling avoids imposing integer precision on other flow producers.
    for(int y = 2; y < current.rows - 3; ++y)
        for(int x = 2; x < current.cols - 3; ++x) {
            const auto b = backward.at<cv::Vec2f>(y, x);
            const float mx = x + b[0], my = y + b[1];
            if(!std::isfinite(mx) || !std::isfinite(my) || mx < 2 || my < 2 || mx >= current.cols - 3 || my >= current.rows - 3) continue;
            const int ix = static_cast<int>(mx), iy = static_cast<int>(my);
            const float fx = mx - ix, fy = my - iy;
            cv::Vec2f f{0, 0};
            float confidence = 0, warped = 0;
            for(int dy = 0; dy < 2; ++dy)
                for(int dx = 0; dx < 2; ++dx) {
                    const float weight = (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy);
                    f += weight * forward.at<cv::Vec2f>(iy + dy, ix + dx);
                    confidence += weight * score.at<std::uint8_t>(iy + dy, ix + dx);
                    warped += weight * before.at<float>(iy + dy, ix + dx);
                }
            const float fb = std::hypot(f[0] + b[0], f[1] + b[1]);
            const float tolerance = config.forwardBackwardThreshold + 0.05f * std::hypot(b[0], b[1]);
            const float photo = std::abs(warped - after.at<float>(y, x));
            const cv::Point2f motion{-b[0] * ratio, -b[1] * ratio};
            const float blur = length(motion);
            if(!std::isfinite(fb) || fb >= tolerance || confidence < config.minConfidence || photo >= config.photometricThreshold
               || variance.at<float>(y, x) < config.minTextureVariance || blur < config.minBlur || blur > config.maxBlur)
                continue;
            const Bin bin{static_cast<int>(std::round(motion.x / config.motionQuantum)), static_cast<int>(std::round(motion.y / config.motionQuantum))};
            if(bin.first == 0 && bin.second == 0) continue;
            bins.at<cv::Vec2i>(y, x) = {bin.first, bin.second};
            valid.at<std::uint8_t>(y, x) = 255;
            result.blend.at<float>(y, x) = config.strength * (1 - fb / tolerance) * (1 - photo / config.photometricThreshold) * confidence / 255.f;
        }
    cv::erode(valid, valid, cv::Mat::ones(3, 3, CV_8U));
    for(int y = 0; y < current.rows; ++y)
        for(int x = 0; x < current.cols; ++x)
            if(valid.at<std::uint8_t>(y, x)) {
                const auto bin = bins.at<cv::Vec2i>(y, x);
                ++counts[{bin[0], bin[1]}];
            }
    std::vector<std::pair<Bin, int>> sorted(counts.begin(), counts.end());
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) { return a.second != b.second ? a.second > b.second : a.first < b.first; });
    std::map<Bin, int> lookup;
    for(const auto& entry : sorted) {
        if(entry.second < 16 || lookup.size() >= config.maxClusters) break;
        lookup[entry.first] = static_cast<int>(result.kernels.size());
        result.kernels.push_back(inverseKernel({entry.first.first * config.motionQuantum, entry.first.second * config.motionQuantum}, config));
    }
    cv::Mat support = cv::Mat::zeros(current.size(), CV_32F);
    for(int y = 0; y < current.rows; ++y)
        for(int x = 0; x < current.cols; ++x) {
            const auto bin = bins.at<cv::Vec2i>(y, x);
            const auto it = lookup.find({bin[0], bin[1]});
            if(valid.at<std::uint8_t>(y, x) && it != lookup.end()) {
                result.labels.at<int>(y, x) = it->second;
                support.at<float>(y, x) = 1;
            } else
                result.blend.at<float>(y, x) = 0;
        }
    cv::GaussianBlur(support, support, {0, 0}, 1);
    result.blend = result.blend.mul(support);
    return result;
}
cv::Mat filterCpu(const Field& field) {
    cv::Mat output = field.image.clone();
    for(int y = 0; y < output.rows; ++y)
        for(int x = 0; x < output.cols; ++x) {
            const int label = field.labels.at<int>(y, x);
            if(label < 0) continue;
            const auto& kernel = field.kernels.at(label);
            const int r = kernel.rows / 2;
            float sum = 0;
            for(int ky = 0; ky < kernel.rows; ++ky)
                for(int kx = 0; kx < kernel.cols; ++kx)
                    sum += kernel.at<float>(ky, kx) * field.image.at<float>(reflect(y + ky - r, output.rows), reflect(x + kx - r, output.cols));
            output.at<float>(y, x) = sum;
        }
    return output;
}
cv::Mat finish(const Field& field, const cv::Mat& filtered, const DeblurProperties& config) {
    cv::Mat output(field.image.size(), CV_8U);
    for(int y = 0; y < output.rows; ++y)
        for(int x = 0; x < output.cols; ++x) {
            const float source = field.image.at<float>(y, x), blend = field.blend.at<float>(y, x);
            float value = source;
            if(blend > 0 && std::isfinite(filtered.at<float>(y, x)))
                value += std::clamp(blend * (filtered.at<float>(y, x) - source), -config.maxCorrection, config.maxCorrection);
            output.at<std::uint8_t>(y, x) = cv::saturate_cast<std::uint8_t>(value);
        }
    return output;
}
}  // namespace deblur
}  // namespace gate
}  // namespace dai
