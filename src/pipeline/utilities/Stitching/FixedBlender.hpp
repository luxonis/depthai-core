#pragma once

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <vector>

namespace dai {
namespace utilities {

std::vector<cv::Scalar> estimateChannelGains(const std::vector<cv::Point>& corners, const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& masks);

std::vector<cv::Mat> findSeamMasks(const cv::Size& canvas,
                                   const std::vector<cv::Point>& corners,
                                   const std::vector<cv::Mat>& images,
                                   const std::vector<cv::Mat>& masks,
                                   cv::detail::SeamFinder& finder,
                                   double megapixels);

class FixedBlender {
   public:
    struct Source {
        cv::Point tl;
        cv::Mat mask;
        cv::Mat seamMask;
        cv::Scalar gain{1.0, 1.0, 1.0, 1.0};
        cv::Mat map1;
        cv::Mat map2;
    };

    void prepare(const cv::Rect& canvas, const std::vector<Source>& sources, int bandWidth);

    cv::Mat blend(const std::vector<cv::Mat>& images, int borderMode) const;

   private:
    struct Part {
        size_t piece;
        cv::Rect rect;
        cv::Mat weight;
    };
    struct Strip {
        cv::Rect rect;
        cv::Mat mask;
        std::vector<Part> parts;
    };
    struct Piece {
        size_t source;
        cv::Rect used;
        cv::Mat own;
        cv::Scalar gain;
        cv::Mat map1;
        cv::Mat map2;
    };

    cv::Rect canvas;
    std::vector<Piece> pieces;
    std::vector<Strip> strips;
};

}  // namespace utilities
}  // namespace dai
