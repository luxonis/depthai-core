
#include "VppImpl.hpp"

#include <tbb/global_control.h>

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <set>
#include <thread>

namespace dai {
namespace gate {

uint8_t VppImpl::getRandomGrayValue() {
    thread_local uint32_t state = 0x12345678u ^ static_cast<uint32_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    state = state * 1664525u + 1013904223u;
    return static_cast<uint8_t>((state >> 16) & 0xFF);
}

void VppImpl::randomColorPatch(NonuniformPatch& patch) {
    for(int y = 0; y < patch.colors.rows; ++y) {
        uint8_t* colorsRow = patch.colors.ptr<uint8_t>(y);
        for(int x = 0; x < patch.colors.cols; ++x) {
            colorsRow[x] = getRandomGrayValue();
        }
    }
}

uint8_t VppImpl::getPatchColor(const cv::Mat& img, const UniformPatch& patch, VppConfig::PatchColoringType coloringType, cv::Mat& hist) {
    uint8_t color;
    switch(coloringType) {
        case VppConfig::RANDOM: {
            color = getRandomGrayValue();
            break;
        }

        case VppConfig::MAXDIST: {
            int startX = std::max(0, patch.startX);
            int startY = std::max(0, patch.startY);
            int endX = std::min(static_cast<int>(img.cols), patch.startX + static_cast<int>(patch.width));
            int endY = std::min(static_cast<int>(img.rows), patch.startY + static_cast<int>(patch.height));
            if(endY < 0 || endX < 0) {
                return 0;
            }
            // ROI inside the image
            cv::Mat patchROI = img(cv::Range(startY, endY), cv::Range(startX, endX));

            // Find a gray level farthest from the most common color
            color = findFarthestColor(patchROI, hist);
            break;
        }

        default: {
            color = 0;
            break;
        }
    }

    return color;
}

bool VppImpl::markOccupied(cv::Mat& occupancyMask, int startX, int startY, int width, int height, float dispDiff) {
    // Handle subpixel shift — if dispDiff > 0, patch extends by 1 pixel in width
    if(dispDiff > 0.0f) width += 1;

    // Compute bounds safely
    int endX = std::min(startX + width, occupancyMask.cols);
    int endY = std::min(startY + height, occupancyMask.rows);
    startX = std::max(startX, 0);
    startY = std::max(startY, 0);

    // If the region is completely outside, consider it occupied (invalid placement)
    if(endX <= 0 || endY <= 0 || startX >= occupancyMask.cols || startY >= occupancyMask.rows) return true;

    if(endX <= startX || endY <= startY) return true;
    auto region = occupancyMask(cv::Rect(startX, startY, endX - startX, endY - startY));
    if(cv::countNonZero(region) != 0) return true;

    // Rejected patches must not reserve pixels and block subsequent patches.
    region.setTo(255);
    return false;
}

uint8_t VppImpl::findFarthestColor(const cv::Mat& img, cv::Mat& hist, float sigma) {
    CV_Assert(img.type() == CV_8UC1);

    hist.setTo(0);

    // Manual histogram (faster than calcHist on small ROIs)
    uint16_t bins[256] = {};
    for(int y = 0; y < img.rows; ++y) {
        const uint8_t* row = img.ptr<uint8_t>(y);
        for(int x = 0; x < img.cols; ++x) ++bins[row[x]];
    }
    // copy & normalize
    const float invN = 1.0f / static_cast<float>(img.total());
    float* h = hist.ptr<float>(0);
    for(int i = 0; i < 256; ++i) h[i] = bins[i] * invN;

    // 1D blur only
    const int kernelSize = (int(6 * sigma + 1) | 1);
    cv::GaussianBlur(hist, hist, cv::Size(kernelSize, 1), sigma, 0, cv::BORDER_REPLICATE);

    // dominant → farthest (naive) — keep your logic or min-bin logic
    double maxVal;
    int maxIdx[2];
    float dominant;
    if(cv::countNonZero(hist) == 0) {
        dominant = 0;
    } else {
        cv::minMaxIdx(hist, nullptr, &maxVal, nullptr, maxIdx);
        dominant = float(maxIdx[0]);
    }

    return static_cast<uint8_t>(dominant < 128.f ? 255 : 0);
}

cv::Mat VppImpl::getDisparityMask(const cv::Mat& disparity, const cv::Mat& img, const cv::Mat& confidence) {
    if(!parameters.injectionParameters.useInjection) {
        if(allValidMask.size() != disparity.size()) {
            allValidMask.create(disparity.size(), CV_8UC1);
            allValidMask.setTo(255);
        }
        return allValidMask;
    }
    const cv::Mat energyMap = getEnergyMap(img);

    cv::Mat highTextureMask;
    cv::Mat lowConfidenceMask;
    cv::threshold(energyMap, highTextureMask, parameters.injectionParameters.textureThreshold, 255, cv::THRESH_BINARY_INV);
    cv::resize(highTextureMask, highTextureMask, disparity.size(), 0, 0, cv::INTER_NEAREST);
    highTextureMask.convertTo(highTextureMask, CV_8U);

    if(!confidence.empty()) {
        cv::threshold(confidence, lowConfidenceMask, parameters.injectionParameters.confidenceThreshold, 255, cv::THRESH_BINARY);
        lowConfidenceMask.convertTo(lowConfidenceMask, CV_8U);
    } else {
        lowConfidenceMask = cv::Mat::ones(energyMap.size(), CV_8U) * 255;
    }

    cv::Mat combinedMask;
    cv::bitwise_and(highTextureMask, lowConfidenceMask, combinedMask);

    if(parameters.injectionParameters.useMorphology) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});

        cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), parameters.injectionParameters.morphologyIterations);
        cv::morphologyEx(
            combinedMask, combinedMask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), std::max(1, parameters.injectionParameters.morphologyIterations / 2));
    }

    if(combinedMask.size() != disparity.size()) {
        cv::resize(combinedMask, combinedMask, disparity.size(), 0, 0, cv::INTER_NEAREST);
    }
    return combinedMask;
}

cv::Mat VppImpl::getEnergyMap(const cv::Mat& img) {
    // Compute Sobel gradients
    cv::Mat sobelx, sobely, energyMap;
    cv::Sobel(img, sobelx, CV_32F, 1, 0, parameters.injectionParameters.kernelSize);
    cv::Sobel(img, sobely, CV_32F, 0, 1, parameters.injectionParameters.kernelSize);
    cv::magnitude(sobelx, sobely, energyMap);
    return energyMap;
}

// --- Draw Patch ---
void VppImpl::drawPatch(cv::Mat& img, const UniformPatch& patch) {
    int startXnonzero = std::max(patch.startX, 0);
    int endX = std::min(patch.startX + patch.width, img.cols);
    int width = endX - startXnonzero;

    if(width <= 0) return;

    cv::Rect roi(startXnonzero, patch.startY, width, static_cast<int>(patch.height));
    cv::Mat imgROI = img(roi);
    // cv::Mat patchMat(imgROI.size(), CV_8UC1, cv::Scalar(patch.color));
    // cv::addWeighted(patchMat, blending, imgROI, 1.0 - blending, 0, imgROI);
    imgROI.convertTo(imgROI, -1, 1.0 - parameters.blending, parameters.blending * patch.color);
}

void VppImpl::drawPatch(cv::Mat& img, const UniformPatch& patch, float subpixelShift) {
    CV_Assert(img.type() == CV_8UC1);
    CV_Assert(subpixelShift >= 0.f && subpixelShift <= 1.f);

    const int imgRows = img.rows;
    const int imgCols = img.cols;
    const int patchRows = patch.height;
    const int patchCols = patch.width;
    // precompute weights
    const float blendWeightA = parameters.blending;
    const float blendWeightB = (1. - parameters.blending);
    const float subpixelWeightA = subpixelShift;
    const float subpixelWeightB = (1. - subpixelShift);

    for(int y = 0; y < patch.height; ++y) {
        const int iy = patch.startY + y;
        if(iy < 0 || iy >= imgRows) continue;

        uint8_t* imgRow = img.ptr<uint8_t>(iy);
        // first pixel
        int ix = patch.startX;
        if((ix >= 0 && ix < imgCols)) {
            uint8_t& imgPixel = imgRow[ix];
            float patchColor = subpixelWeightA * imgPixel + subpixelWeightB * static_cast<float>(patch.color);
            imgPixel = static_cast<uint8_t>(std::round(patchColor * blendWeightA + blendWeightB * static_cast<float>(imgPixel)));
        }

        for(int x = 1; x < patchCols; ++x) {
            ix = patch.startX + x;
            if(ix < 0 || ix > imgCols - 1) continue;
            uint8_t& imgPixel = imgRow[ix];
            imgRow[ix] = static_cast<uint8_t>(std::round(static_cast<float>(patch.color) * blendWeightA + blendWeightB * static_cast<float>(imgPixel)));
        }
        // last pixel
        ix = patch.startX + patchCols;
        if((ix >= 0 && ix < imgCols)) {
            uint8_t& imgPixel = imgRow[ix];
            float patchColor = subpixelWeightA * static_cast<float>(patch.color) + subpixelWeightB * imgPixel;
            imgPixel = static_cast<uint8_t>(std::round(patchColor * blendWeightA + blendWeightB * static_cast<float>(imgPixel)));
        }
    }
}

void VppImpl::drawPatch(cv::Mat& img, const NonuniformPatch& patch, float subpixelShift) {
    CV_Assert(img.type() == CV_8UC1);
    CV_Assert(subpixelShift >= 0.f && subpixelShift < 1.f);

    const int imgRows = img.rows;
    const int imgCols = img.cols;
    const int patchRows = patch.colors.rows;
    const int patchCols = patch.colors.cols;
    // precompute weights
    const float blendWeightA = parameters.blending;
    const float blendWeightB = (1. - parameters.blending);
    const float subpixelWeightA = subpixelShift;
    const float subpixelWeightB = (1. - subpixelShift);

    for(int y = 0; y < patchRows; ++y) {
        const int iy = patch.startY + y;
        if(iy < 0 || iy >= imgRows) continue;

        uint8_t* imgRow = img.ptr<uint8_t>(iy);
        const uint8_t* patchRow = patch.colors.ptr<uint8_t>(y);
        // first pixel
        int ix = patch.startX;
        if((ix >= 0 && ix < imgCols)) {
            uint8_t& imgPixel = imgRow[ix];
            float patchColor = subpixelWeightA * imgPixel + subpixelWeightB * static_cast<float>(patchRow[0]);
            imgPixel = static_cast<uint8_t>(std::round(patchColor * blendWeightA + blendWeightB * static_cast<float>(imgPixel)));
        }

        for(int x = 1; x < patchCols; ++x) {
            ix = patch.startX + x;
            if(ix < 0 || ix > imgCols - 1) continue;
            uint8_t& imgPixel = imgRow[ix];
            float patchColor = subpixelWeightA * static_cast<float>(patchRow[x - 1]) + subpixelWeightB * static_cast<float>(patchRow[x]);
            imgRow[ix] = static_cast<uint8_t>(std::round(static_cast<float>(patchColor) * blendWeightA + blendWeightB * static_cast<float>(imgPixel)));
        }
        // last pixel
        ix = patch.startX + patchCols;
        if((ix >= 0 && ix < imgCols)) {
            uint8_t& imgPixel = imgRow[ix];
            float patchColor = subpixelWeightA * static_cast<float>(patchRow[patchCols - 1]) + subpixelWeightB * imgPixel;
            imgPixel = static_cast<uint8_t>(std::round(patchColor * blendWeightA + blendWeightB * static_cast<float>(imgPixel)));
        }
    }
}

void VppImpl::drawPatch(cv::Mat& img, const NonuniformPatch& patch) {
    CV_Assert(img.type() == CV_8UC1);
    CV_Assert(patch.colors.type() == CV_8UC1);
    CV_Assert(patch.colors.rows > 0 && patch.colors.cols > 0);

    const int imgRows = img.rows;
    const int imgCols = img.cols;
    const int patchRows = patch.colors.rows;
    const int patchCols = patch.colors.cols;

    const float blendWeightA = parameters.blending;
    const float blendWeightB = (1.0f - parameters.blending);

    for(int y = 0; y < patchRows; ++y) {
        const int iy = patch.startY + y;
        if(iy < 0 || iy >= imgRows) continue;

        uint8_t* imgRow = img.ptr<uint8_t>(iy);
        const uint8_t* patchRow = patch.colors.ptr<uint8_t>(y);

        for(int x = 0; x < patchCols; ++x) {
            const int ix = patch.startX + x;
            if(ix < 0 || ix >= imgCols) continue;

            imgRow[ix] = static_cast<uint8_t>(std::round(patchRow[x] * blendWeightA + imgRow[ix] * blendWeightB));
        }
    }
}

// Preserve markOccupied's original fractional footprint, including clipping at x = -1.
bool VppImpl::markSinglePixelOccupied(uint8_t* occupied, int width, int startX, float subpixelShift) {
    const int first = std::max(0, startX);
    const int end = std::min(width, startX + (subpixelShift > 0.0f ? 2 : 1));
    if(end <= first) return true;
    if(occupied[first] || (end - first == 2 && occupied[first + 1])) return true;
    occupied[first] = 255;
    if(end - first == 2) occupied[first + 1] = 255;
    return false;
}

void VppImpl::drawSinglePixel(uint8_t& left, uint8_t* right, int width, int startX, float subpixelShift, uint8_t color) {
    // Keep the same operation order and rounding as the nonuniform drawPatch overloads.
    const float blendWeightA = parameters.blending;
    const float blendWeightB = 1.0f - parameters.blending;
    left = static_cast<uint8_t>(std::round(color * blendWeightA + left * blendWeightB));
    if(startX >= 0 && startX < width) {
        auto& pixel = right[startX];
        const float patchColor = subpixelShift * pixel + (1.0f - subpixelShift) * static_cast<float>(color);
        pixel = static_cast<uint8_t>(std::round(patchColor * blendWeightA + blendWeightB * static_cast<float>(pixel)));
    }
    const int endX = startX + 1;
    if(endX >= 0 && endX < width) {
        auto& pixel = right[endX];
        const float patchColor = subpixelShift * static_cast<float>(color) + (1.0f - subpixelShift) * pixel;
        pixel = static_cast<uint8_t>(std::round(patchColor * blendWeightA + blendWeightB * static_cast<float>(pixel)));
    }
}

void VppImpl::processSinglePixel(cv::Mat& imgLeft, cv::Mat& imgRight, const cv::Mat& disparity, const cv::Mat& mask) {
    cv::Mat usedRight = cv::Mat::zeros(imgRight.size(), CV_8UC1);
    cv::parallel_for_(cv::Range(0, disparity.rows), [&](const cv::Range& range) {
        for(int y = range.start; y < range.end; ++y) {
            auto* leftRow = imgLeft.ptr<uint8_t>(y);
            auto* rightRow = imgRight.ptr<uint8_t>(y);
            auto* occupied = usedRight.ptr<uint8_t>(y);
            const auto* dispRow = disparity.ptr<float>(y);
            const auto* maskRow = mask.ptr<uint8_t>(y);
            for(int x = 0; x < disparity.cols; ++x) {
                const float disp = dispRow[x];
                if(disp <= 0.0f || maskRow[x] == 0) continue;
                const int rounded = std::ceil(disp);
                const float subpixelShift = static_cast<float>(rounded) - disp;
                const int rightX = x - rounded;
                if(markSinglePixelOccupied(occupied, imgRight.cols, rightX, subpixelShift)) continue;
                drawSinglePixel(leftRow[x], rightRow, imgRight.cols, rightX, subpixelShift, getRandomGrayValue());
            }
        }
    });
}

// --- Process disparity map ---
void VppImpl::process(cv::Mat& imgLeft, cv::Mat& imgRight, const cv::Mat& disparity, const cv::Mat& confidence) {
    if(imgLeft.empty() || imgRight.empty() || disparity.empty()) {
        throw std::runtime_error("VPP node: Error: One or more input matrices are empty.\n");
    }

    // Ensure all are 1-channel grayscale
    CV_Assert(imgLeft.channels() == 1);
    CV_Assert(imgRight.channels() == 1);
    CV_Assert(disparity.channels() == 1);
    if(!confidence.empty()) {
        CV_Assert(confidence.channels() == 1);
    }

    CV_Assert(disparity.type() == CV_32F);
    if(!confidence.empty()) {
        CV_Assert(confidence.type() == CV_32F);
    }
    CV_Assert(imgLeft.type() == CV_8UC1);
    CV_Assert(imgRight.type() == CV_8UC1);
    const float ratioX = static_cast<float>(imgLeft.cols) / disparity.cols;
    const float ratioY = static_cast<float>(imgLeft.rows) / disparity.rows;

    const uint8_t patchSizeX = std::min(parameters.maxPatchSize, uint8_t(ratioX));
    const uint8_t patchSizeY = std::min(parameters.maxPatchSize, uint8_t(ratioY));

    const float blending = parameters.blending;
    const auto mask = getDisparityMask(disparity, imgLeft, confidence);

    tbb::global_control c(tbb::global_control::max_allowed_parallelism, parameters.maxNumThreads);
    // Full-resolution, nonuniform priors produce 1x1 patches. Avoid an OpenCV ROI
    // and generic patch machinery for every candidate; keep the same mask and scan order.
    if(disparity.size() == imgLeft.size() && imgLeft.size() == imgRight.size() && parameters.maxPatchSize > 0 && !parameters.uniformPatch) {
        processSinglePixel(imgLeft, imgRight, disparity, mask);
        return;
    }

    cv::Mat usedRight = cv::Mat::zeros(imgRight.size(), CV_8UC1);
    cv::parallel_for_(cv::Range(0, disparity.rows), [&](const cv::Range& range) {
        // {
        UniformPatch uniformPatch;
        cv::Mat hist = cv::Mat::zeros(256, 1, CV_32F);
        std::vector<NonuniformPatch> patches;
        patches.reserve(parameters.maxPatchSize * parameters.maxPatchSize);
        for(int j = 1; j <= parameters.maxPatchSize; ++j) {
            for(int i = 1; i <= parameters.maxPatchSize; ++i) {
                NonuniformPatch patch;
                patch.colors = cv::Mat(j, i, CV_8UC1);
                patches.push_back(patch);
            }
        }
        for(int y = range.start; y < range.end; ++y) {
            const float* dispRow = disparity.ptr<float>(y);
            const uint8_t* maskRow = mask.ptr<uint8_t>(y);
            const int startDisparityPixelY = std::ceil(y * ratioY);
            const int endDisparityPixelY = int((y + 1) * ratioY);
            for(int x = 0; x < disparity.cols; ++x) {
                const float disp = dispRow[x];
                // skip zero disparity
                if(disp <= 0.0f || maskRow[x] == 0) {
                    continue;
                }
                const int dispRounded = std::ceil(disp * ratioX);
                const float dispDiff = static_cast<float>(dispRounded) - disp * ratioX;

                const int startDisparityPixelX = std::ceil(x * ratioX);
                const int endDisparityPixelX = int((x + 1) * ratioX);

                const int dispartyPixelWidthX = endDisparityPixelX - startDisparityPixelX;
                const int dispartyPixelWidthY = endDisparityPixelY - startDisparityPixelY;

                const int patchWidthX = std::min(int(patchSizeX), dispartyPixelWidthX);
                const int patchWidthY = std::min(int(patchSizeY), dispartyPixelWidthY);

                const int startX = startDisparityPixelX + (dispartyPixelWidthX - patchWidthX) / 2;
                const int startY = startDisparityPixelY + (dispartyPixelWidthY - patchWidthY) / 2;

                if(startX - dispRounded + patchWidthX < 0 || patchWidthX <= 0 || patchWidthY <= 0) {
                    continue;
                }
                if(markOccupied(usedRight, startX - dispRounded, startY, patchWidthX, patchWidthY, dispDiff)) {
                    // if the imgRightution is already occupied continue
                    continue;
                }
                if(parameters.uniformPatch) {
                    uniformPatch.width = patchWidthX;
                    uniformPatch.height = patchWidthY;
                    uniformPatch.startY = startY;
                    uniformPatch.startX = startX;
                    uniformPatch.color = getPatchColor(imgLeft, uniformPatch, parameters.patchColoringType, hist);

                    drawPatch(imgLeft, uniformPatch);

                    uniformPatch.startX = startX - dispRounded;
                    drawPatch(imgRight, uniformPatch, dispDiff);

                } else {
                    auto& patch = patches[(patchWidthY - 1) * (parameters.maxPatchSize) + (patchWidthX - 1)];

                    patch.startY = startY;
                    patch.startX = startX;
                    randomColorPatch(patch);

                    drawPatch(imgLeft, patch);

                    patch.startX = startX - dispRounded;
                    drawPatch(imgRight, patch, dispDiff);
                }
            }
        }
    });
}

}  // namespace gate
}  // namespace dai
