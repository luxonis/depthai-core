#pragma once

#include <nop/structure.h>
#include <spdlog/async_logger.h>

#include <array>
#include <nlohmann/json.hpp>
#include <vector>

#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/Colormap.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/Serialization.hpp"

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
    float width;
    float height;
    bool normalized;
    bool center;

    Crop() : width(0), height(0), normalized(true), center(true) {}
    Crop(float width, float height, bool normalized = false, bool center = false) : width(width), height(height), normalized(normalized), center(center) {}

    Crop clone() const {
        return *this;
    }

    std::string toStr() const override {
        return "C:w=" + std::to_string(width) + ",h=" + std::to_string(height) + ",n=" + std::to_string(normalized) + ",c=" + std::to_string(center);
    }

    DEPTHAI_SERIALIZE(Crop, width, height, normalized, center);
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

class ImageManipOpsBase {
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

    ImageManipOpsBase() = default;
    virtual ~ImageManipOpsBase() = default;

    ImageManipOpsBase& addOp(ManipOp op) {
        operations.push_back(op);
        return *this;
    }

    ImageManipOpsBase& transformPerspective(std::array<float, 9> matrix) {
        operations.emplace_back(Perspective(matrix));
        return *this;
    }

    ImageManipOpsBase& transformAffine(std::array<float, 4> matrix) {
        operations.emplace_back(Affine(matrix));
        return *this;
    }

    ImageManipOpsBase& transformFourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalizedCoords = false) {
        operations.emplace_back(FourPoints(src, dst, normalizedCoords));
        return *this;
    }

    ImageManipOpsBase& flipHorizontal(bool center = true) {
        operations.emplace_back(Flip(Flip::Direction::HORIZONTAL, center));
        return *this;
    }

    ImageManipOpsBase& flipVertical(bool center = true) {
        operations.emplace_back(Flip(Flip::Direction::VERTICAL, center));
        return *this;
    }

    ImageManipOpsBase& resize(float width, float height, bool normalized = false) {
        operations.emplace_back(Resize(width, height, normalized));
        return *this;
    }

    ImageManipOpsBase& crop(float x, float y, float w, float h, bool normalized = false, bool center = false) {
        operations.emplace_back(Translate(-x, -y, normalized));
        operations.emplace_back(Crop(w, h, normalized, center));
        return *this;
    }

    ImageManipOpsBase& resizeFit() {
        operations.emplace_back(Resize::fit());
        return *this;
    }

    ImageManipOpsBase& resizeFill() {
        operations.emplace_back(Resize::fill());
        return *this;
    }

    ImageManipOpsBase& resizeWidthKeepAspectRatio(float width, bool normalized = false) {
        operations.emplace_back(Resize(width, 0, normalized));
        return *this;
    }

    ImageManipOpsBase& resizeHeightKeepAspectRatio(float height, bool normalized = false) {
        operations.emplace_back(Resize(0, height, normalized));
        return *this;
    }

    ImageManipOpsBase& rotateRadians(float angle, bool center = true) {
        operations.emplace_back(Rotate(angle, center));
        return *this;
    }

    ImageManipOpsBase& rotateDegrees(float angle, bool center = true) {
        return rotateRadians(angle * 3.14159265358979323846f / 180.0f, center);
    }

    ImageManipOpsBase& translate(float offsetX, float offsetY, bool normalizedCoords = false) {
        operations.emplace_back(Translate(offsetX, offsetY, normalizedCoords));
        return *this;
    }

    ImageManipOpsBase& setOutputSize(float width, float height) {
        outputWidth = width;
        outputHeight = height;
        resizeMode = ResizeMode::NONE;
        return *this;
    }

    ImageManipOpsBase& setOutputResize(uint32_t width, uint32_t height, ResizeMode mode) {
        outputWidth = width;
        outputHeight = height;
        resizeMode = mode;
        center = true;
        return *this;
    }

    ImageManipOpsBase& setOutputCenter(bool c=true) {
        center = c;
        return *this;
    }

    ImageManipOpsBase& setBackgroundColor(uint8_t red, uint8_t green, uint8_t blue) {
        background = Background::COLOR;
        this->red = red;
        this->green = green;
        this->blue = blue;
        return *this;
    }

    ImageManipOpsBase& setBackgroundReplicate() {
        background = Background::REPLICATE;
        return *this;
    }

    ImageManipOpsBase& setColormap(Colormap clr) {
        colormap = clr;
        return *this;
    }

    const std::vector<ManipOp>& getOperations() const {
        return this->operations;
    }

    ImageManipOpsBase& clear() {
        operations.clear();
        return *this;
    }

    DEPTHAI_SERIALIZE(ImageManipOpsBase, operations, outputWidth, outputHeight, center, resizeMode, background, red, green, blue, colormap);
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
class ImageManipConfigV2 : public Buffer {
   public:
    ImageManipConfigV2() = default;
    virtual ~ImageManipConfigV2() = default;

    using ResizeMode = ImageManipOpsBase::ResizeMode;

    // New config
    ImageManipOpsBase base;
    ImgFrame::Type outputFrameType = ImgFrame::Type::NONE;

    // Usable with runtime config only,
    // when ImageManipProperties.inputConfig.setWaitForMessage(true) is set
    bool reusePreviousImage = false;
    bool skipCurrentImage = false;

    // New API
    /**
     * Crops the image to the specified rectangle
     * @param x X coordinate of the top-left corner
     * @param y Y coordinate of the top-left corner
     * @param w Width of the rectangle
     * @param h Height of the rectangle
     */
    ImageManipConfigV2& crop(uint32_t x, uint32_t y, uint32_t w, uint32_t h);
    /**
     * Crops the image to the specified (rotated) rectangle
     * @param rect RotatedRect to crop
     * @param normalizedCoords If true, the coordinates are normalized to range [0, 1] where 1 maps to the width/height of the image
     */
    ImageManipConfigV2& cropRotatedRect(dai::RotatedRect rotatedRect, bool normalizedCoords = false);
    /**
     * Resizes the image to the specified width and height
     * @param w Width of the output image
     * @param h Height of the output image
     */
    ImageManipConfigV2& resize(uint32_t w, uint32_t h);
    /**
     * Rescales the image using the specified factors
     * @param scaleX Scale factor for the X axis
     * @param scaleY Scale factor for the Y axis. If not specified, scaleY is set to the same value as scaleX
     */
    ImageManipConfigV2& scale(float scaleX, float scaleY = 0);
    /**
     * Rotates the image around its center by the specified angle in degrees
     * @param angle Angle in radians
     */
    ImageManipConfigV2& rotateDeg(float angle);
    /**
     * Rotates the image around the specified point by the specified angle in degrees
     * @param angle Angle in radians
     * @param center Center of the rotation using normalized coordinates
     */
    ImageManipConfigV2& rotateDeg(float angle, Point2f center);
    /**
     * Flips the image horizontally
     */
    ImageManipConfigV2& flipHorizontal();
    /**
     * Flips the image vertically
     */
    ImageManipConfigV2& flipVertical();
    /**
     * Applies an affine transformation to the image
     * @param matrix an array containing a 2x2 matrix representing the affine transformation
     */
    ImageManipConfigV2& transformAffine(std::array<float, 4> matrix);
    /**
     * Applies a perspective transformation to the image
     * @param matrix an array containing a 3x3 matrix representing the perspective transformation
     */
    ImageManipConfigV2& transformPerspective(std::array<float, 9> matrix);
    /**
     * Applies a perspective transformation to the image
     * @param src Source points
     * @param dst Destination points
     * @param normalizedCoords If true, the coordinates are normalized to range [0, 1] where 1 maps to the width/height of the image
     */
    ImageManipConfigV2& transformFourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalizedCoords = false);
    /**
     * Sets the output size of the image
     * @param w Width of the output image
     * @param h Height of the output image
     * @param mode Resize mode. NONE - no resize, STRETCH - stretch to fit, LETTERBOX - keep aspect ratio and pad with background color, CENTER_CROP - keep aspect ratio and crop
     */
    ImageManipConfigV2& setOutputSize(uint32_t w, uint32_t h, ResizeMode mode = ResizeMode::NONE);
    /**
     * Sets the colormap to be applied to a grayscale image
     * @param colormap Colormap type to be applied
     */
    ImageManipConfigV2& setColormap(Colormap colormap);
    /**
     * Sets the frame type of the output image
     * @param frameType Frame type of the output image
     */
    ImageManipConfigV2& setFrameType(ImgFrame::Type frameType);

    /**
     * Instruct ImageManip to not remove current image from its queue and use the same for next message.
     * @param reuse True to enable reuse, false otherwise
     */
    ImageManipConfigV2& setReusePreviousImage(bool reuse);

    /**
     * Instructs ImageManip to skip current image and wait for next in queue.
     * @param skip True to skip current image, false otherwise
     */
    ImageManipConfigV2& setSkipCurrentImage(bool skip);

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

    DEPTHAI_SERIALIZE(ImageManipConfigV2, base, outputFrameType, reusePreviousImage, skipCurrentImage);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImageManipConfigV2;
    };
};
}  // namespace dai
