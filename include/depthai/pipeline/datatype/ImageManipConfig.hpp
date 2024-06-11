#pragma once

#include <nop/structure.h>
#include <spdlog/async_logger.h>

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

struct OpBase {
    virtual ~OpBase() = default;
    virtual std::string toStr() const = 0;
};

struct Translate : OpBase {
    float offsetX;
    float offsetY;
    bool normalized;

    Translate() = default;
    Translate(float offsetX, float offsetY, bool normalized = false) : offsetX(offsetX), offsetY(offsetY), normalized(normalized) {}

    std::string toStr() const override {
        return "T:x=" + std::to_string(offsetX) + ",y=" + std::to_string(offsetY) + ",n=" + std::to_string(normalized);
    }

    DEPTHAI_SERIALIZE(Translate, offsetX, offsetY, normalized);
};

struct Rotate : OpBase {
    float angle;  // in radians
    bool center;  // if true, rotation is around center of image, otherwise around top-left corner

    Rotate() = default;
    explicit Rotate(float angle, bool center = true) : angle(angle), center(center) {}

    std::string toStr() const override {
        return "Rot:a=" + std::to_string(angle) + ",c=" + std::to_string(center);
    }

    DEPTHAI_SERIALIZE(Rotate, angle, center);
};

struct Resize : OpBase {
    enum Mode { VALUE, FIT, FILL };
    float width;
    float height;
    bool normalized;
    Mode mode = FIT;

    Resize() = default;
    Resize(float width, float height, bool normalized = false) : width(width), height(height), normalized(normalized), mode(VALUE) {}

    static Resize fit() {
        Resize r(0, 0);
        r.mode = FIT;
        return r;
    }

    static Resize fill() {
        Resize r(0, 0);
        r.mode = FILL;
        return r;
    }

    std::string toStr() const override {
        return "Res:w=" + std::to_string(width) + ",h=" + std::to_string(height) + ",n=" + std::to_string(normalized) + ",m=" + std::to_string(mode);
    }

    DEPTHAI_SERIALIZE(Resize, width, height, normalized, mode);
};

struct Flip : OpBase {
    enum Direction { HORIZONTAL, VERTICAL };
    Direction direction = HORIZONTAL;
    bool center = false;  // if true, flip is around center of image, otherwise around top-left corner

    Flip() = default;
    explicit Flip(Direction direction, bool center = true) : direction(direction), center(center) {}

    std::string toStr() const override {
        return "F:d=" + std::to_string(direction) + ",c=" + std::to_string(center);
    }

    DEPTHAI_SERIALIZE(Flip, direction, center);
};

struct Affine : OpBase {
    std::array<float, 4> matrix{1, 0, 0, 1};

    Affine() = default;
    explicit Affine(std::array<float, 4> matrix) : matrix(matrix) {}

    std::string toStr() const override {
        return "A:m=" + std::to_string(matrix[0]) + "," + std::to_string(matrix[1]) + "," + std::to_string(matrix[2]) + "," + std::to_string(matrix[3]);
    }

    DEPTHAI_SERIALIZE(Affine, matrix);
};

struct Perspective : OpBase {
    std::array<float, 9> matrix{1, 0, 0, 0, 1, 0, 0, 0, 1};

    Perspective() = default;
    explicit Perspective(std::array<float, 9> matrix) : matrix(matrix) {}

    std::string toStr() const override {
        return "P:m=" + std::to_string(matrix[0]) + "," + std::to_string(matrix[1]) + "," + std::to_string(matrix[2]) + "," + std::to_string(matrix[3]) + ","
               + std::to_string(matrix[4]) + "," + std::to_string(matrix[5]) + "," + std::to_string(matrix[6]) + "," + std::to_string(matrix[7]) + ","
               + std::to_string(matrix[8]);
    }

    DEPTHAI_SERIALIZE(Perspective, matrix);
};

struct FourPoints : OpBase {
    std::array<dai::Point2f, 4> src{dai::Point2f(0.0, 0.0), dai::Point2f(1.0, 0.0), dai::Point2f(1.0, 1.0), dai::Point2f(0.0, 1.0)};
    std::array<dai::Point2f, 4> dst{dai::Point2f(0.0, 0.0), dai::Point2f(1.0, 0.0), dai::Point2f(1.0, 1.0), dai::Point2f(0.0, 1.0)};
    bool normalized = false;

    FourPoints() = default;
    FourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalized = false) : src(src), dst(dst), normalized(normalized) {}

    std::string toStr() const override {
        return "4P:s1=" + std::to_string(src[0].x) + "," + std::to_string(src[0].y) + ",s2=" + std::to_string(src[1].x) + "," + std::to_string(src[1].y)
               + ",s3=" + std::to_string(src[2].x) + "," + std::to_string(src[2].y) + ",s4=" + std::to_string(src[3].x) + "," + std::to_string(src[3].y)
               + "d1=" + std::to_string(dst[0].x) + "," + std::to_string(dst[0].y) + ",d2=" + std::to_string(dst[1].x) + "," + std::to_string(dst[1].y)
               + ",d3=" + std::to_string(dst[2].x) + "," + std::to_string(dst[2].y) + ",d4=" + std::to_string(dst[3].x) + "," + std::to_string(dst[3].y);
    }

    DEPTHAI_SERIALIZE(FourPoints, src, dst, normalized);
};

struct Crop : OpBase {
    uint32_t width;
    uint32_t height;
    bool center;

    Crop() : width(0), height(0), center(true) {}
    Crop(uint32_t width, uint32_t height, bool center = false) : width(width), height(height), center(center) {}

    Crop clone() const {
        return *this;
    }

    std::string toStr() const override {
        return "C:w=" + std::to_string(width) + ",h=" + std::to_string(height) + ",c=" + std::to_string(center);
    }

    DEPTHAI_SERIALIZE(Crop, width, height, center);
};

struct ManipOp {
    std::variant<Translate, Rotate, Resize, Flip, Affine, Perspective, FourPoints, Crop> op;

    ManipOp() = default;
    ManipOp(Translate op) : op(op) {}      // NOLINT
    ManipOp(Rotate op) : op(op) {}         // NOLINT
    ManipOp(Resize op) : op(op) {}         // NOLINT
    ManipOp(Flip op) : op(op) {}           // NOLINT
    ManipOp(Affine op) : op(op) {}         // NOLINT
    ManipOp(Perspective op) : op(op) {}    // NOLINT
    ManipOp(FourPoints op) : op(op) {}     // NOLINT
    ManipOp(Crop op) : op(op) {}           // NOLINT

    DEPTHAI_SERIALIZE(ManipOp, op);
};

class ImageManipBase {
   public:
    enum class Background : uint8_t { COLOR, REPLICATE, MIRROR };
    enum class ResizeMode : uint8_t { NONE, STRETCH, LETTERBOX, CENTER_CROP };

   protected:
    std::vector<ManipOp> operations{};

   public:
    uint32_t outputWidth = 0;
    uint32_t outputHeight = 0;
    bool center = false;
    ResizeMode resizeMode = ResizeMode::NONE;
    Background background = Background::COLOR;
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    Colormap colormap = Colormap::NONE;

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

    ImageManipBase& crop(float x, float y, float w, float h, bool center = false) {
        operations.emplace_back(Translate(-x, -y, x < 1 && y < 1 && w <= 2 && h <= 2));
        operations.emplace_back(Crop(w, h, center));
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

    ImageManipBase& resizeWidthKeepAspectRatio(float width, bool normalized = false) {
        operations.emplace_back(Resize(width, 0, normalized));
        return *this;
    }

    ImageManipBase& resizeHeightKeepAspectRatio(float height, bool normalized = false) {
        operations.emplace_back(Resize(0, height, normalized));
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

    ImageManipBase& setOutputSize(float width, float height) {
        outputWidth = width;
        outputHeight = height;
        resizeMode = ResizeMode::NONE;
        return *this;
    }

    ImageManipBase& setOutputResize(uint32_t width, uint32_t height, ResizeMode mode) {
        outputWidth = width;
        outputHeight = height;
        resizeMode = mode;
        center = true;
        return *this;
    }

    ImageManipBase& setOutputCenter(bool c=true) {
        center = c;
        return *this;
    }

    ImageManipBase& setBackgroundColor(uint8_t red, uint8_t green, uint8_t blue) {
        background = Background::COLOR;
        this->red = red;
        this->green = green;
        this->blue = blue;
        return *this;
    }

    ImageManipBase& setBackgroundReplicate() {
        background = Background::REPLICATE;
        return *this;
    }

    ImageManipBase& setColormap(Colormap clr) {
        colormap = clr;
        return *this;
    }

    const std::vector<ManipOp>& getOperations() const {
        return this->operations;
    }

    ImageManipBase& clear() {
        operations.clear();
        return *this;
    }

    DEPTHAI_SERIALIZE(ImageManipBase, operations, outputWidth, outputHeight, center, resizeMode, background, red, green, blue, colormap);
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

    // New API
    ImageManipConfig& crop(uint32_t x, uint32_t y, uint32_t w, uint32_t h);
    ImageManipConfig& resize(uint32_t w, uint32_t h);
    ImageManipConfig& scale(float scale);
    ImageManipConfig& rotateDeg(float angle);
    ImageManipConfig& setOutputSize(uint32_t w, uint32_t h, ImageManipBase::ResizeMode mode = ImageManipBase::ResizeMode::NONE);

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
