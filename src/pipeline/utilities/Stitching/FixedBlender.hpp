#pragma once

#include <opencv2/core.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <vector>

namespace dai {
namespace utilities {

/**
 * Per-channel gain of each source that makes the overlaps agree, solved by least squares on the mean overlap intensities.
 * The mean gain is held at one and every gain is only weakly pulled towards one, so that large exposure differences are
 * matched while a source without overlaps keeps a gain of one.
 *
 * @param corners Top-left corner of each image in a shared canvas
 * @param images CV_8UC3 images
 * @param masks CV_8U pixels each image covers
 */
std::vector<cv::Scalar> estimateChannelGains(const std::vector<cv::Point>& corners, const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& masks);

/**
 * Seams between warped images. The seam finder runs on copies downscaled to `megapixels` of the canvas, as OpenCV's stitcher
 * does, and its masks are scaled back to the images.
 *
 * @param canvas Size of the canvas the corners are placed in
 * @param corners Top-left corner of each image in the canvas
 * @param images CV_8UC3 images
 * @param masks CV_8U pixels each image covers
 * @param megapixels Resolution of the seam estimation, 0 or less for full resolution
 * @return CV_8U pixels of each image's mask the seam finder assigned to it, dilated by one pixel
 */
std::vector<cv::Mat> findSeamMasks(const cv::Size& canvas,
                                   const std::vector<cv::Point>& corners,
                                   const std::vector<cv::Mat>& images,
                                   const std::vector<cv::Mat>& masks,
                                   cv::detail::SeamFinder& finder,
                                   double megapixels);

/**
 * Blends warped sources whose placement, seams and exposure do not change between frames.
 *
 * Everything that depends only on the geometry is computed once in prepare(): feather weights that ramp across each seam
 * over a fixed band, the pixels each source owns alone, and the rectangles each source contributes to. A source whose
 * contribution is split, e.g. across the wrap of a 360° panorama, gets one rectangle per part, with the warp maps cut to
 * it. Per frame, each part is warped from its source image and multiplied by the source's per-channel gain, its own pixels
 * are copied and only the bands around the seams are blended.
 */
class FixedBlender {
   public:
    struct Source {
        cv::Point tl;                         ///< Top-left corner of the warped source in canvas coordinates
        cv::Mat mask;                         ///< CV_8U pixels the warped source covers
        cv::Mat seamMask;                     ///< CV_8U pixels assigned to the source, by a seam finder or by a direct copy
        cv::Scalar gain{1.0, 1.0, 1.0, 1.0};  ///< Per-channel exposure gain
        cv::Mat map1;                         ///< cv::remap maps from the source image to the warped source (mask-sized)
        cv::Mat map2;                         ///< Second map of a fixed-point pair, empty for a two-channel float map1
    };

    /**
     * @param canvas Canvas in the coordinates of the source corners
     * @param sources Sources, their masks sized like their warped images
     * @param bandWidth Width in pixels of the ramp across a seam; 0 gives hard seams, each pixel from one source
     */
    void prepare(const cv::Rect& canvas, const std::vector<Source>& sources, int bandWidth);

    /**
     * @param images CV_8UC3 source images, one per source, as the maps of prepare() sample them
     * @param borderMode cv::remap border mode for samples outside an image
     * @return CV_8UC3 canvas
     */
    cv::Mat blend(const std::vector<cv::Mat>& images, int borderMode) const;

   private:
    struct Part {
        size_t piece;
        cv::Rect rect;   ///< In strip coordinates
        cv::Mat weight;  ///< CV_8UC3 weights (255 = 1) of the piece inside rect
    };
    struct Strip {
        cv::Rect rect;  ///< In canvas pixel coordinates
        cv::Mat mask;   ///< CV_8U pixels of the strip that are blended
        std::vector<Part> parts;
    };
    struct Piece {
        size_t source;
        cv::Rect used;  ///< In canvas pixel coordinates
        cv::Mat own;    ///< CV_8U pixels of `used` the source fills alone
        cv::Scalar gain;
        cv::Mat map1;  ///< Warp maps of the source cut to `used`
        cv::Mat map2;
    };

    cv::Rect canvas;
    std::vector<Piece> pieces;
    std::vector<Strip> strips;
};

}  // namespace utilities
}  // namespace dai
