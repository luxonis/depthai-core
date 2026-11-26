#include "FixedBlender.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

#include "utility/ErrorMacros.hpp"

namespace dai {
namespace utilities {

namespace {

constexpr float MIN_WEIGHT = 1.0f / 256.0f;
constexpr int STRIP_ROWS = 32;
constexpr int MIN_PIECE_GAP = 32;

std::vector<cv::Range> pieceColumns(const cv::Mat& used) {
    cv::Mat columns;
    cv::reduce(used, columns, 0, cv::REDUCE_MAX);
    std::vector<cv::Range> ranges;
    int begin = -1;
    int lastUsed = -1;
    for(int x = 0; x < columns.cols; ++x) {
        if(columns.at<uint8_t>(0, x) == 0) continue;
        if(begin < 0) {
            begin = x;
        } else if(x - lastUsed > MIN_PIECE_GAP) {
            ranges.emplace_back(begin, lastUsed + 1);
            begin = x;
        }
        lastUsed = x;
    }
    if(begin >= 0) ranges.emplace_back(begin, lastUsed + 1);
    return ranges;
}

bool isUnitGain(const cv::Scalar& gain) {
    return std::abs(gain[0] - 1.0) < 1e-3 && std::abs(gain[1] - 1.0) < 1e-3 && std::abs(gain[2] - 1.0) < 1e-3;
}

}  // namespace

std::vector<cv::Scalar> estimateChannelGains(const std::vector<cv::Point>& corners, const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& masks) {
    const int count = static_cast<int>(images.size());
    DAI_CHECK_V(corners.size() == images.size() && masks.size() == images.size(), "Gain estimation needs a corner and a mask per image");
    struct Overlap {
        int i;
        int j;
        double pixels;
        cv::Scalar meanI;
        cv::Scalar meanJ;
    };
    std::vector<Overlap> overlaps;
    for(int i = 0; i < count; ++i) {
        for(int j = i + 1; j < count; ++j) {
            const cv::Rect overlap = cv::Rect(corners[i], images[i].size()) & cv::Rect(corners[j], images[j].size());
            if(overlap.area() == 0) continue;
            const cv::Rect inI = overlap - corners[i];
            const cv::Rect inJ = overlap - corners[j];
            const cv::Mat both = masks[i](inI) & masks[j](inJ);
            const int pixels = cv::countNonZero(both);
            if(pixels == 0) continue;
            overlaps.push_back({i, j, static_cast<double>(pixels), cv::mean(images[i](inI), both), cv::mean(images[j](inJ), both)});
        }
    }

    constexpr double PRIOR = 1e-3;
    const int rows = static_cast<int>(overlaps.size());
    double total = 1.0;
    for(const auto& overlap : overlaps) total += std::sqrt(overlap.pixels);
    std::vector<cv::Scalar> gains(count, cv::Scalar(1.0, 1.0, 1.0, 1.0));
    for(int channel = 0; channel < 3; ++channel) {
        cv::Mat A = cv::Mat::zeros(rows + count + 1, count, CV_64F);
        cv::Mat b = cv::Mat::zeros(A.rows, 1, CV_64F);
        for(int r = 0; r < rows; ++r) {
            const auto& overlap = overlaps[r];
            const double weight = std::sqrt(overlap.pixels) / std::max(1.0, 0.5 * (overlap.meanI[channel] + overlap.meanJ[channel]));
            A.at<double>(r, overlap.i) = weight * overlap.meanI[channel];
            A.at<double>(r, overlap.j) = -weight * overlap.meanJ[channel];
        }
        for(int i = 0; i < count; ++i) {
            A.at<double>(rows + i, i) = PRIOR * total;
            b.at<double>(rows + i) = PRIOR * total;
        }
        A.row(A.rows - 1).setTo(total);
        b.at<double>(A.rows - 1) = total * count;
        cv::Mat solution;
        cv::solve(A, b, solution, cv::DECOMP_SVD);
        for(int i = 0; i < count; ++i) gains[i][channel] = std::max(0.0, solution.at<double>(i));
    }
    return gains;
}

std::vector<cv::Mat> findSeamMasks(const cv::Size& canvas,
                                   const std::vector<cv::Point>& corners,
                                   const std::vector<cv::Mat>& images,
                                   const std::vector<cv::Mat>& masks,
                                   cv::detail::SeamFinder& finder,
                                   double megapixels) {
    DAI_CHECK_V(corners.size() == images.size() && masks.size() == images.size(), "Seam finding needs a corner and a mask per image");
    const double scale = megapixels > 0.0 ? std::min(1.0, std::sqrt(megapixels * 1e6 / static_cast<double>(canvas.area()))) : 1.0;
    std::vector<cv::Point> seamCorners;
    std::vector<cv::UMat> seamImages;
    std::vector<cv::UMat> seamMasks;
    for(size_t i = 0; i < images.size(); ++i) {
        const cv::Size size(std::max(1, static_cast<int>(std::lround(images[i].cols * scale))),
                            std::max(1, static_cast<int>(std::lround(images[i].rows * scale))));
        cv::Mat image;
        cv::Mat mask;
        cv::resize(images[i], image, size, 0, 0, cv::INTER_LINEAR_EXACT);
        cv::resize(masks[i], mask, size, 0, 0, cv::INTER_NEAREST);
        image.convertTo(image, CV_32F);
        seamCorners.emplace_back(static_cast<int>(std::lround(corners[i].x * scale)), static_cast<int>(std::lround(corners[i].y * scale)));
        seamImages.push_back(image.getUMat(cv::ACCESS_READ));
        seamMasks.push_back(mask.getUMat(cv::ACCESS_RW));
    }
    finder.find(seamImages, seamCorners, seamMasks);

    std::vector<cv::Mat> result;
    result.reserve(images.size());
    for(size_t i = 0; i < images.size(); ++i) {
        cv::Mat seamMask;
        cv::dilate(seamMasks[i].getMat(cv::ACCESS_READ), seamMask, cv::Mat());
        cv::resize(seamMask, seamMask, masks[i].size(), 0, 0, cv::INTER_LINEAR);
        result.push_back(seamMask & masks[i]);
    }
    return result;
}

void FixedBlender::prepare(const cv::Rect& canvasRect, const std::vector<Source>& input, int bandWidth) {
    DAI_CHECK_V(!input.empty(), "The fixed blender needs at least one source");
    DAI_CHECK_V(canvasRect.area() > 0, "The fixed blender needs a non-empty canvas");
    canvas = canvasRect;
    pieces.clear();
    strips.clear();
    const cv::Rect bounds(cv::Point(0, 0), canvas.size());
    const int kernel = std::max(1, bandWidth) | 1;
    const int margin = kernel / 2 + 1;

    std::vector<cv::Rect> regions(input.size());
    std::vector<cv::Mat> weights(input.size());
    cv::Mat sum = cv::Mat::zeros(canvas.size(), CV_32F);
    for(size_t i = 0; i < input.size(); ++i) {
        const auto& source = input[i];
        DAI_CHECK_V(source.mask.size() == source.seamMask.size() && source.map1.size() == source.mask.size(),
                    "Fixed blender source {} needs a seam mask and maps sized like its mask",
                    i);
        const cv::Rect placed = cv::Rect(source.tl - canvas.tl(), source.mask.size()) & bounds;
        cv::Rect region = placed;
        if(region.area() > 0) {
            const cv::Rect seam = cv::boundingRect(source.seamMask(placed - (source.tl - canvas.tl())));
            region = cv::Rect(seam.x + placed.x - margin, seam.y + placed.y - margin, seam.width + 2 * margin, seam.height + 2 * margin) & placed;
        }
        regions[i] = region;
        if(region.area() == 0) continue;

        const cv::Rect local = region - (source.tl - canvas.tl());
        cv::Mat weight;
        source.seamMask(local).convertTo(weight, CV_32F, 1.0 / 255.0);
        cv::blur(weight, weight, cv::Size(kernel, kernel), cv::Point(-1, -1), cv::BORDER_REPLICATE);
        // Distance to the nearest uncovered pixel, the border of the warped image included, ramps from 1 to half the band
        cv::Mat padded;
        cv::copyMakeBorder(source.mask, padded, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::Mat distance;
        cv::distanceTransform(padded, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
        cv::Mat fade = cv::min(distance(local + cv::Point(1, 1)) / std::max(1, kernel / 2), 1.0);
        weight = weight.mul(fade);
        weight.setTo(0.0f, source.mask(local) == 0);
        weights[i] = weight;
        sum(region) += weight;
    }

    cv::Mat kept = cv::Mat::zeros(canvas.size(), CV_32F);
    for(size_t i = 0; i < input.size(); ++i) {
        if(weights[i].empty()) continue;
        cv::Mat share;
        cv::divide(weights[i], sum(regions[i]), share);
        weights[i].setTo(0.0f, share < MIN_WEIGHT);
        kept(regions[i]) += weights[i];
    }

    cv::Mat blended = cv::Mat::zeros(canvas.size(), CV_8U);
    std::vector<cv::Mat> pieceWeights;
    for(size_t i = 0; i < input.size(); ++i) {
        if(weights[i].empty()) continue;
        cv::divide(weights[i], kept(regions[i]), weights[i]);
        cv::patchNaNs(weights[i], 0.0);  // 0/0 where no source has weight; float division by zero is not defined by OpenCV
        const cv::Mat used = weights[i] > 0;
        for(const auto& columns : pieceColumns(used)) {
            const cv::Rect band(columns.start, 0, columns.size(), used.rows);
            const cv::Rect rect = cv::boundingRect(used(band)) + band.tl();
            Piece piece;
            piece.source = i;
            piece.used = rect + regions[i].tl();
            piece.gain = input[i].gain;
            const cv::Rect inSource = piece.used - (input[i].tl - canvas.tl());
            piece.map1 = input[i].map1(inSource).clone();
            if(!input[i].map2.empty()) piece.map2 = input[i].map2(inSource).clone();
            pieceWeights.push_back(weights[i](rect).clone());
            piece.own = pieceWeights.back() >= 1.0f - MIN_WEIGHT;
            blended(piece.used) |= (pieceWeights.back() > 0) & ~piece.own;
            pieces.push_back(std::move(piece));
        }
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    for(int top = 0; top < canvas.height; top += STRIP_ROWS) {
        const cv::Rect band(0, top, canvas.width, std::min(STRIP_ROWS, canvas.height - top));
        const int count = cv::connectedComponentsWithStats(blended(band), labels, stats, centroids, 8, CV_32S);
        for(int label = 1; label < count; ++label) {
            const cv::Rect inBand(stats.at<int>(label, cv::CC_STAT_LEFT),
                                  stats.at<int>(label, cv::CC_STAT_TOP),
                                  stats.at<int>(label, cv::CC_STAT_WIDTH),
                                  stats.at<int>(label, cv::CC_STAT_HEIGHT));
            Strip strip;
            strip.rect = inBand + band.tl();
            strip.mask = labels(inBand) == label;
            for(size_t p = 0; p < pieces.size(); ++p) {
                const cv::Rect part = strip.rect & pieces[p].used;
                if(part.area() == 0) continue;
                cv::Mat weight;
                pieceWeights[p](part - pieces[p].used.tl()).convertTo(weight, CV_8U, 255.0);
                if(cv::countNonZero(weight & strip.mask(part - strip.rect.tl())) == 0) continue;
                cv::Mat weight3;
                cv::merge(std::vector<cv::Mat>{weight, weight, weight}, weight3);
                strip.parts.push_back({p, part - strip.rect.tl(), weight3});
            }
            strips.push_back(std::move(strip));
        }
    }
}

cv::Mat FixedBlender::blend(const std::vector<cv::Mat>& images, int borderMode) const {
    cv::Mat result = cv::Mat::zeros(canvas.size(), CV_8UC3);
    std::vector<cv::Mat> exposed(pieces.size());
    for(size_t i = 0; i < pieces.size(); ++i) {
        const auto& piece = pieces[i];
        DAI_CHECK_V(piece.source < images.size() && images[piece.source].type() == CV_8UC3,
                    "Fixed blender source {} needs a BGR image, got {} images",
                    piece.source,
                    images.size());
        cv::remap(images[piece.source], exposed[i], piece.map1, piece.map2, cv::INTER_LINEAR, borderMode);
        if(!isUnitGain(piece.gain)) cv::multiply(exposed[i], piece.gain, exposed[i]);
        exposed[i].copyTo(result(piece.used), piece.own);
    }

    for(const auto& strip : strips) {
        cv::Mat sum = cv::Mat::zeros(strip.rect.size(), CV_32FC3);
        for(const auto& part : strip.parts) {
            const cv::Rect inPiece = part.rect + strip.rect.tl() - pieces[part.piece].used.tl();
            cv::Mat region = sum(part.rect);
            cv::accumulateProduct(exposed[part.piece](inPiece), part.weight, region);
        }
        cv::Mat blended;
        sum.convertTo(blended, CV_8U, 1.0 / 255.0);
        blended.copyTo(result(strip.rect), strip.mask);
    }
    return result;
}

}  // namespace utilities
}  // namespace dai
