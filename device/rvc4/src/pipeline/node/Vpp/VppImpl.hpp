#ifndef MAIN_H_
#define MAIN_H_

#include <opencv2/opencv.hpp>
#include <random>

#include "depthai/pipeline/datatype/VppConfig.hpp"

namespace dai {
namespace gate {
class VppImplTestable;
class VppImpl {
   public:
    void setConfig(VppConfig& parametersNew) {
        parameters = parametersNew;
    }

    const VppConfig& getConfig() const {
        return parameters;
    }

    VppImpl(const VppConfig& parameters) : parameters(parameters) {}

    void process(cv::Mat& imgLeft, cv::Mat& imgRight, const cv::Mat& disparity, const cv::Mat& confidence);

   private:
    struct UniformPatch {
        uint8_t color;
        uint8_t width;
        uint8_t height;
        int startX;
        int startY;
    };

    struct NonuniformPatch {
        cv::Mat colors;
        int startX;
        int startY;
    };

    static uint8_t findFarthestColor(const cv::Mat& img, cv::Mat& hist, float sigma = 1.0);

    uint8_t getRandomGrayValue();

    uint8_t getPatchColor(const cv::Mat& img, const UniformPatch& patch, VppConfig::PatchColoringType coloringType, cv::Mat& hist);

    void randomColorPatch(NonuniformPatch& patch);

    static bool markOccupied(cv::Mat& occupancyMask, int startX, int startY, int width, int height, float dispDiff);

    // Draws a patch into the image at specified location with blending
    void drawPatch(cv::Mat& img, const UniformPatch& patch);

    void drawPatch(cv::Mat& img, const UniformPatch& patch, float subpixelShift);

    void drawPatch(cv::Mat& img, const NonuniformPatch& patch);

    void drawPatch(cv::Mat& img, const NonuniformPatch& patch, float subpixelShift);

    // Dense, nonuniform 1x1 specialization. Other configurations use the generic path.
    static bool markSinglePixelOccupied(uint8_t* occupied, int width, int startX, float subpixelShift);
    void drawSinglePixel(uint8_t& left, uint8_t* right, int width, int startX, float subpixelShift, uint8_t color);
    void processSinglePixel(cv::Mat& imgLeft, cv::Mat& imgRight, const cv::Mat& disparity, const cv::Mat& mask);

    cv::Mat getDisparityMask(const cv::Mat& disparity, const cv::Mat& img, const cv::Mat& neuralConfidence);

    virtual cv::Mat getEnergyMap(const cv::Mat& img);

    cv::Mat allValidMask;
    VppConfig parameters;

    friend class VppImplTestable;
};
}  // namespace gate
}  // namespace dai

#endif  // MAIN_H_
