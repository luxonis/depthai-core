#pragma once

#include <nop/structure.h>

#include <array>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <variant>
#include <vector>

#include "depthai/common/Colormap.hpp"
#include "depthai/common/Interpolation.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {

struct Translate {
    float offsetX;
    float offsetY;
    bool normalized;

    Translate() = default;
    Translate(float offsetX, float offsetY, bool normalized = false) : offsetX(offsetX), offsetY(offsetY), normalized(normalized) {}

    DEPTHAI_SERIALIZE(Translate, offsetX, offsetY, normalized);
};

struct Rotate {
    float angle;  // in radians
    bool center;  // if true, rotation is around center of image, otherwise around top-left corner

    Rotate() = default;
    explicit Rotate(float angle, bool center = true) : angle(angle), center(center) {}

    DEPTHAI_SERIALIZE(Rotate, angle, center);
};

struct Resize {
    enum Mode { VALUE, FIT, FILL };
    float width;
    float height;
    bool normalized;
    Mode mode = FIT;

    Resize() = default;
    Resize(float width, float height, bool normalized = false) : width(width), height(height), normalized(normalized), mode(VALUE) {}

    static Resize fit() {
        Resize r(-1, -1);
        r.mode = FIT;
        return r;
    }

    static Resize fill() {
        Resize r(-1, -1);
        r.mode = FILL;
        return r;
    }

    DEPTHAI_SERIALIZE(Resize, width, height, normalized, mode);
};

struct Flip {
    enum Direction { HORIZONTAL, VERTICAL };
    Direction direction = HORIZONTAL;
    bool center = false;  // if true, flip is around center of image, otherwise around top-left corner

    Flip() = default;
    explicit Flip(Direction direction, bool center = true) : direction(direction), center(center) {}

    DEPTHAI_SERIALIZE(Flip, direction, center);
};

struct Affine {
    std::array<float, 4> matrix{1, 0, 0, 1};

    Affine() = default;
    explicit Affine(std::array<float, 4> matrix) : matrix(matrix) {}

    DEPTHAI_SERIALIZE(Affine, matrix);
};

struct Perspective {
    std::array<float, 9> matrix{1, 0, 0, 0, 1, 0, 0, 0, 1};

    Perspective() = default;
    explicit Perspective(std::array<float, 9> matrix) : matrix(matrix) {}

    DEPTHAI_SERIALIZE(Perspective, matrix);
};

struct FourPoints {
    std::array<dai::Point2f, 4> src{dai::Point2f(0.0, 0.0), dai::Point2f(1.0, 0.0), dai::Point2f(1.0, 1.0), dai::Point2f(0.0, 1.0)};
    std::array<dai::Point2f, 4> dst{dai::Point2f(0.0, 0.0), dai::Point2f(1.0, 0.0), dai::Point2f(1.0, 1.0), dai::Point2f(0.0, 1.0)};
    bool normalized;

    FourPoints() = default;
    FourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalized = false)
        : src(src), dst(dst), normalizedCoords(normalized) {}

    DEPTHAI_SERIALIZE(FourPoints, src, dst, normalized);
};

struct Canvas {
    enum class Background { COLOR, REPLICATE, MIRROR };
    enum class ResizeMode { STRETCH, LETTERBOX, CENTER_CROP };

    float width;
    float height;
    Background background;
    bool center;
    bool resizeOutput = false;
    ResizeMode resizeMode = ResizeMode::STRETCH;
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    Canvas() : width(-1), height(-1), background(Background::COLOR), center(true), resizeOutput(false), resizeMode(ResizeMode::STRETCH), red(0), green(0), blue(0) {}
    Canvas(float width,
           float height,
           Background background,
           bool center = false,
           bool resizeOutput = false,
           ResizeMode resizeMode = ResizeMode::STRETCH,
           uint8_t red = 0,
           uint8_t green = 0,
           uint8_t blue = 0)
        : width(width),
          height(height),
          background(background),
          center(center),
          resizeOutput(resizeOutput),
          resizeMode(resizeMode),
          red(red),
          green(green),
          blue(blue) {}

    Canvas clone() const {
        return *this;
    }

    DEPTHAI_SERIALIZE(Canvas, width, height, background, center, resizeOutput, resizeMode, red, green, blue);
};

struct ApplyColormap {
    Colormap colormap;

    ApplyColormap() = default;
    explicit ApplyColormap(Colormap colormap) : colormap(colormap) {}

    DEPTHAI_SERIALIZE(ApplyColormap, colormap);
};

struct ManipOp {
    std::variant<Translate, Rotate, Resize, Flip, Affine, Perspective, FourPoints, Canvas, ApplyColormap> op;

    ManipOp() = default;
    ManipOp(Translate op) : op(op) {}      // NOLINT
    ManipOp(Rotate op) : op(op) {}         // NOLINT
    ManipOp(Resize op) : op(op) {}         // NOLINT
    ManipOp(Flip op) : op(op) {}           // NOLINT
    ManipOp(Affine op) : op(op) {}         // NOLINT
    ManipOp(Perspective op) : op(op) {}    // NOLINT
    ManipOp(FourPoints op) : op(op) {}     // NOLINT
    ManipOp(Canvas op) : op(op) {}         // NOLINT
    ManipOp(ApplyColormap op) : op(op) {}  // NOLINT

    DEPTHAI_SERIALIZE(ManipOp, op);
};

class ImageManipBase {
   protected:
    std::vector<ManipOp> operations{};

   public:
    ImageManipBase() = default;
    virtual ~ImageManipBase() = default;

    ImageManipBase& addOp(ManipOp op) {
        operations.push_back(op);
        return *this;
    }

    ImageManipBase& transformPerspective(std::array<float, 9> matrix) {
        operations.emplace_back(Perspective(matrix));
        return *this;
    }

    ImageManipBase& transformAffine(std::array<float, 4> matrix) {
        operations.emplace_back(Affine(matrix));
        return *this;
    }

    ImageManipBase& transformFourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalizedCoords = false) {
        operations.emplace_back(FourPoints(src, dst, normalizedCoords));
        return *this;
    }

    ImageManipBase& flipHorizontal(bool center = true) {
        operations.emplace_back(Flip(Flip::Direction::HORIZONTAL, center));
        return *this;
    }

    ImageManipBase& flipVertical(bool center = true) {
        operations.emplace_back(Flip(Flip::Direction::VERTICAL, center));
        return *this;
    }

    ImageManipBase& resize(float width, float height) {
        operations.emplace_back(Resize(width, height));
        return *this;
    }

    ImageManipBase& resizeFit() {
        operations.emplace_back(Resize::fit());
        return *this;
    }

    ImageManipBase& resizeFill() {
        operations.emplace_back(Resize::fill());
        return *this;
    }

    ImageManipBase& resizeWidthKeepAspectRatio(float width) {
        operations.emplace_back(Resize(width, -1));
        return *this;
    }

    ImageManipBase& resizeHeightKeepAspectRatio(float height) {
        operations.emplace_back(Resize(-1, height));
        return *this;
    }

    ImageManipBase& rotateRadians(float angle, bool center = true) {
        operations.emplace_back(Rotate(angle, center));
        return *this;
    }

    ImageManipBase& rotateDegrees(float angle, bool center = true) {
        return rotateRadians(angle * 3.14159265358979323846f / 180.0f, center);
    }

    ImageManipBase& translate(float offsetX, float offsetY, bool normalizedCoords = false) {
        operations.emplace_back(Translate(offsetX, offsetY, normalizedCoords));
        return *this;
    }

    ImageManipBase& setColormap(Colormap colormap) {
        operations.emplace_back(ApplyColormap(colormap));
        return *this;
    }

    ImageManipBase& setOutputSize(float width, float height) {
        std::optional<Canvas> canvasOpt = std::nullopt;
        for(auto& op : operations) {
            if(std::holds_alternative<Canvas>(op.op)) {
                canvasOpt = std::get<Canvas>(op.op).clone();
                break;
            }
        }
        if(canvasOpt) {
            canvasOpt->width = width;
            canvasOpt->height = height;
        } else {
            canvasOpt = Canvas(width, height, Canvas::Background::COLOR);
        }
        operations.emplace_back(*canvasOpt);
        return *this;
    }

    ImageManipBase& setOutputResize(float width, float height, Canvas::ResizeMode resizeMode) {
        std::optional<Canvas> canvasOpt = std::nullopt;
        for(auto& op : operations) {
            if(std::holds_alternative<Canvas>(op.op)) {
                canvasOpt = std::get<Canvas>(op.op).clone();
                break;
            }
        }
        if(canvasOpt) {
            canvasOpt->width = width;
            canvasOpt->height = height;
            canvasOpt->resizeOutput = true;
            canvasOpt->resizeMode = resizeMode;
        } else {
            canvasOpt = Canvas(width, height, Canvas::Background::COLOR, true, true, resizeMode);
        }
        operations.emplace_back(*canvasOpt);
        return *this;
    }

    ImageManipBase& setOutputCenter(bool center) {
        std::optional<Canvas> canvasOpt = std::nullopt;
        for(auto& op : operations) {
            if(std::holds_alternative<Canvas>(op.op)) {
                canvasOpt = std::get<Canvas>(op.op).clone();
                break;
            }
        }
        if(canvasOpt) {
            canvasOpt->center = center;
        } else {
            canvasOpt = Canvas(-1, -1, Canvas::Background::COLOR, center, false, Canvas::ResizeMode::STRETCH, 0, 0, 0);
        }
        operations.emplace_back(*canvasOpt);
        return *this;
    }

    ImageManipBase& setBackgroundColor(uint8_t red, uint8_t green, uint8_t blue) {
        std::optional<Canvas> canvasOpt = std::nullopt;
        for(auto& op : operations) {
            if(std::holds_alternative<Canvas>(op.op)) {
                canvasOpt = std::get<Canvas>(op.op).clone();
                break;
            }
        }
        if(canvasOpt) {
            canvasOpt->background = Canvas::Background::COLOR;
            canvasOpt->red = red;
            canvasOpt->green = green;
            canvasOpt->blue = blue;
        } else {
            canvasOpt = Canvas(-1, -1, Canvas::Background::COLOR, false, false, Canvas::ResizeMode::STRETCH, red, green, blue);
        }
        operations.emplace_back(*canvasOpt);
        return *this;
    }

    ImageManipBase& setBackgroundReplicate() {
        std::optional<Canvas> canvasOpt = std::nullopt;
        for(auto& op : operations) {
            if(std::holds_alternative<Canvas>(op.op)) {
                canvasOpt = std::get<Canvas>(op.op).clone();
                break;
            }
        }
        if(canvasOpt) {
            canvasOpt->background = Canvas::Background::REPLICATE;
        } else {
            canvasOpt = Canvas(-1, -1, Canvas::Background::REPLICATE, false, false, Canvas::ResizeMode::STRETCH, 0, 0, 0);
        }
        operations.emplace_back(*canvasOpt);
        return *this;
    }

    virtual ImageManipBase& clear() {
        operations.clear();
        return *this;
    }

    DEPTHAI_SERIALIZE(ImageManipBase, operations);
};

/**
 * ImageManipConfig message. Specifies image manipulation options like:
 *
 *  - Crop
 *
 *  - Resize
 *
 *  - Warp
 *
 *  - ...
 */
class ImageManipConfig : public Buffer {
   public:
    ImageManipConfig() = default;
    virtual ~ImageManipConfig() = default;

    // New config
    ImageManipBase base;

    // Old config

    struct CropRect {
        // Normalized range 0-1
        float xmin = 0.0f, ymin = 0.0f, xmax = 0.0f, ymax = 0.0f;

        DEPTHAI_SERIALIZE(CropRect, xmin, ymin, xmax, ymax);
    };

    struct CropConfig {
        CropRect cropRect;
        RotatedRect cropRotatedRect;

        bool enableCenterCropRectangle = false;
        // if enableCenterCropRectangle -> automatically calculated crop parameters
        float cropRatio = 1.0f, widthHeightAspectRatio = 1.0f;

        bool enableRotatedRect = false;

        // Range 0..1 by default. Set 'false' to specify in pixels
        bool normalizedCoords = true;

        DEPTHAI_SERIALIZE(
            CropConfig, cropRect, cropRotatedRect, enableCenterCropRectangle, cropRatio, widthHeightAspectRatio, enableRotatedRect, normalizedCoords);
    };

    struct ResizeConfig {
        int width = 0, height = 0;
        bool lockAspectRatioFill = false;
        char bgRed = 0, bgGreen = 0, bgBlue = 0;

        //  clockwise order, pt[0] is mapped to the top-left output corner
        std::vector<Point2f> warpFourPoints;
        bool normalizedCoords = true;
        bool enableWarp4pt = false;

        std::vector<float> warpMatrix3x3;
        bool enableWarpMatrix = false;

        // Warp background / border mode: replicates pixels if true,
        // otherwise fills with a constant color defined by: bgRed, bgGreen, bgBlue
        bool warpBorderReplicate = false;

        // clockwise
        float rotationAngleDeg;
        bool enableRotation = false;

        /**
         * Whether to keep aspect ratio of input or not
         */
        bool keepAspectRatio = true;

        DEPTHAI_SERIALIZE(ResizeConfig,
                          width,
                          height,
                          lockAspectRatioFill,
                          bgRed,
                          bgGreen,
                          bgBlue,
                          warpFourPoints,
                          normalizedCoords,
                          enableWarp4pt,
                          warpMatrix3x3,
                          enableWarpMatrix,
                          warpBorderReplicate,
                          rotationAngleDeg,
                          enableRotation,
                          keepAspectRatio);
    };

    struct FormatConfig {
        ImgFrame::Type type = ImgFrame::Type::NONE;
        bool flipHorizontal = false;
        bool flipVertical = false;
        Colormap colormap = Colormap::NONE;
        int colormapMin = 0;
        int colormapMax = 255;

        DEPTHAI_SERIALIZE(FormatConfig, type, flipHorizontal, flipVertical, colormap, colormapMin, colormapMax);
    };

    // Functions to set properties
    /**
     * Specifies crop with rectangle with normalized values (0..1)
     * @param xmin Top left X coordinate of rectangle
     * @param ymin Top left Y coordinate of rectangle
     * @param xmax Bottom right X coordinate of rectangle
     * @param ymax Bottom right Y coordinate of rectangle
     */
    ImageManipConfig& setCropRect(float xmin, float ymin, float xmax, float ymax);

    /**
     * Specifies crop with rectangle with normalized values (0..1)
     * @param coordinates Coordinate of rectangle
     */
    ImageManipConfig& setCropRect(std::tuple<float, float, float, float> coordinates);

    /**
     * Specifies crop with rotated rectangle. Optionally as non normalized coordinates
     * @param rr Rotated rectangle which specifies crop
     * @param normalizedCoords If true coordinates are in normalized range (0..1) otherwise absolute
     */
    ImageManipConfig& setCropRotatedRect(RotatedRect rr, bool normalizedCoords = true);

    /**
     * Specifies a centered crop.
     * @param ratio Ratio between input image and crop region (0..1)
     * @param whRatio Crop region aspect ratio - 1 equals to square, 1.7 equals to 16:9, ...
     */
    ImageManipConfig& setCenterCrop(float ratio, float whRatio = 1.0f);

    /**
     * Specifies warp by supplying 4 points in either absolute or normalized coordinates
     * @param pt 4 points specifying warp
     * @param normalizedCoords If true pt is interpreted as normalized, absolute otherwise
     */
    ImageManipConfig& setWarpTransformFourPoints(std::vector<Point2f> pt, bool normalizedCoords);

    /**
     * Specifies warp with a 3x3 matrix
     * @param mat 3x3 matrix
     */
    ImageManipConfig& setWarpTransformMatrix3x3(std::vector<float> mat);

    /**
     * Specifies that warp replicates border pixels
     */
    ImageManipConfig& setWarpBorderReplicatePixels();

    /**
     * Specifies fill color for border pixels. Example:
     *
     *  - setWarpBorderFillColor(255,255,255) -> white
     *
     *  - setWarpBorderFillColor(0,0,255) -> blue
     *
     * @param red Red component
     * @param green Green component
     * @param blue Blue component
     */
    ImageManipConfig& setWarpBorderFillColor(int red, int green, int blue);

    /**
     * Specifies clockwise rotation in degrees
     * @param deg Rotation in degrees
     */
    ImageManipConfig& setRotationDegrees(float deg);

    /**
     * Specifies clockwise rotation in radians
     * @param rad Rotation in radians
     */
    ImageManipConfig& setRotationRadians(float rad);

    /**
     * Specifies output image size. After crop stage the image will be stretched to fit.
     * @param w Width in pixels
     * @param h Height in pixels
     */
    ImageManipConfig& setResize(int w, int h);

    /**
     * Specifies output image size. After crop stage the image will be stretched to fit.
     * @param size Size in pixels
     */
    ImageManipConfig& setResize(std::tuple<int, int> size);

    /**
     * Specifies output image size. After crop stage the image will be resized by preserving aspect ration.
     * Optionally background can be specified.
     *
     * @param w Width in pixels
     * @param h Height in pixels
     * @param bgRed Red component
     * @param bgGreen Green component
     * @param bgBlue Blue component
     */
    ImageManipConfig& setResizeThumbnail(int w, int h, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);

    /**
     * Specifies output image size. After crop stage the image will be resized by preserving aspect ration.
     * Optionally background can be specified.
     *
     * @param size Size in pixels
     * @param bgRed Red component
     * @param bgGreen Green component
     * @param bgBlue Blue component
     */
    ImageManipConfig& setResizeThumbnail(std::tuple<int, int> size, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);

    /**
     * Specify output frame type.
     * @param name Frame type
     */
    ImageManipConfig& setFrameType(ImgFrame::Type name);

    /**
     * Specify gray to color conversion map
     * @param colormap map from Colormap enum or Colormap::NONE to disable
     */
    ImageManipConfig& setColormap(Colormap colormap, int min, int max);
    ImageManipConfig& setColormap(Colormap colormap, float maxf);
    ImageManipConfig& setColormap(Colormap colormap, int max = 255);

    /**
     * Specify horizontal flip
     * @param flip True to enable flip, false otherwise
     */
    ImageManipConfig& setHorizontalFlip(bool flip);

    /**
     * Specify vertical flip
     * @param flip True to enable vertical flip, false otherwise
     */
    void setVerticalFlip(bool flip);

    /**
     * Instruct ImageManip to not remove current image from its queue and use the same for next message.
     * @param reuse True to enable reuse, false otherwise
     */
    ImageManipConfig& setReusePreviousImage(bool reuse);

    /**
     * Instructs ImageManip to skip current image and wait for next in queue.
     * @param skip True to skip current image, false otherwise
     */
    ImageManipConfig& setSkipCurrentImage(bool skip);

    /**
     * Specifies to whether to keep aspect ratio or not
     */
    ImageManipConfig& setKeepAspectRatio(bool keep);

    /**
     * Specify which interpolation method to use
     * @param interpolation type of interpolation
     */
    ImageManipConfig& setInterpolation(dai::Interpolation interpolation);

    // Functions to retrieve properties
    /**
     * @returns Top left X coordinate of crop region
     */
    float getCropXMin() const;

    /**
     * @returns Top left Y coordinate of crop region
     */
    float getCropYMin() const;

    /**
     * @returns Bottom right X coordinate of crop region
     */
    float getCropXMax() const;

    /**
     * @returns Bottom right Y coordinate of crop region
     */
    float getCropYMax() const;

    /**
     * @returns Output image width
     */
    int getResizeWidth() const;

    /**
     * @returns Output image height
     */
    int getResizeHeight() const;

    /**
     * @returns Crop configuration
     */
    CropConfig getCropConfig() const;

    /**
     * @returns Resize configuration
     */
    ResizeConfig getResizeConfig() const;

    /**
     * @returns Format configuration
     */
    FormatConfig getFormatConfig() const;

    /**
     * @returns True if resize thumbnail mode is set, false otherwise
     */
    bool isResizeThumbnail() const;

    /**
     * Instruct ImageManip to not remove current image from its queue and use the same for next message.
     * @returns True to enable reuse, false otherwise
     */
    bool getReusePreviousImage() const;

    /**
     * Instructs ImageManip to skip current image and wait for next in queue.
     * @returns True to skip current image, false otherwise
     */
    bool getSkipCurrentImage() const;

    /**
     * @returns specified colormap
     */
    Colormap getColormap() const;

    CropConfig cropConfig;
    ResizeConfig resizeConfig;
    FormatConfig formatConfig;

    bool enableCrop = false;
    bool enableResize = false;
    bool enableFormat = false;

    // Usable with runtime config only,
    // when ImageManipProperties.inputConfig.setWaitForMessage(true) is set
    bool reusePreviousImage = false;
    bool skipCurrentImage = false;
    Interpolation interpolation = Interpolation::AUTO;
    DEPTHAI_SERIALIZE(ImageManipConfig,
                      cropConfig,
                      resizeConfig,
                      formatConfig,
                      enableCrop,
                      enableResize,
                      enableFormat,
                      reusePreviousImage,
                      skipCurrentImage,
                      interpolation,
                      base);
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImageManipConfig;
    };

    /// Retrieve which interpolation method to use
    dai::Interpolation getInterpolation() const;
};

}  // namespace dai
