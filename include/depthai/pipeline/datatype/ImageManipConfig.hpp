#pragma once

#include <nop/structure.h>

#include <array>
#include <locale>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/common/Colormap.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

struct OpBase {
    virtual ~OpBase();
    virtual std::string toStr() const = 0;
};

struct Translate : OpBase {
    float offsetX;
    float offsetY;
    bool normalized;

    ~Translate() override;

    Translate() = default;
    /**
     * Construct a translation operation.
     */
    Translate(float offsetX, float offsetY, bool normalized = false) : offsetX(offsetX), offsetY(offsetY), normalized(normalized) {}

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "T:x=" << offsetX << ",y=" << offsetY << ",n=" << normalized;
        return ss.str();
    }

    DEPTHAI_SERIALIZE(Translate, offsetX, offsetY, normalized);
};

struct Rotate : OpBase {
    float angle;  // in radians
    bool center;  // if true, rotation is around center of image, otherwise around top-left corner
    float offsetX;
    float offsetY;
    bool normalized;

    ~Rotate() override;

    Rotate() = default;
    /**
     * Construct a rotation operation.
     */
    explicit Rotate(float angle, bool center = true, float offsetX = 0, float offsetY = 0, bool normalized = false)
        : angle(angle), center(center), offsetX(offsetX), offsetY(offsetY), normalized(normalized) {}

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "Rot:a=" << angle << ",c=" << center << ",x=" << offsetX << ",y" << offsetY << ",n=" << normalized;
        return ss.str();
    }

    DEPTHAI_SERIALIZE(Rotate, angle, center, offsetX, offsetY, normalized);
};

struct Resize : OpBase {
    enum Mode { VALUE, FIT, FILL };
    float width;
    float height;
    bool normalized;
    Mode mode = FIT;

    ~Resize() override;

    Resize() = default;
    /**
     * Construct a resize operation.
     */
    Resize(float width, float height, bool normalized = false) : width(width), height(height), normalized(normalized), mode(VALUE) {}

    /**
     * Create a resize operation with FIT mode.
     */
    static Resize fit() {
        Resize r(0, 0);
        r.mode = FIT;
        return r;
    }

    /**
     * Create a resize operation with FILL mode.
     */
    static Resize fill() {
        Resize r(0, 0);
        r.mode = FILL;
        return r;
    }

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "Res:w=" << width << ",h=" << height << ",n=" << normalized << ",m=" << mode;
        return ss.str();
    }

    DEPTHAI_SERIALIZE(Resize, width, height, normalized, mode);
};

struct Flip : OpBase {
    enum Direction { HORIZONTAL, VERTICAL };
    Direction direction = HORIZONTAL;
    bool center = false;  // if true, flip is around center of image, otherwise around top-left corner

    ~Flip() override;

    Flip() = default;
    /**
     * Construct a flip operation.
     */
    explicit Flip(Direction direction, bool center = true) : direction(direction), center(center) {}

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "F:d=" << direction << ",c=" << center;
        return ss.str();
    }

    DEPTHAI_SERIALIZE(Flip, direction, center);
};

struct Affine : OpBase {
    std::array<float, 4> matrix{1, 0, 0, 1};

    ~Affine() override;

    Affine() = default;
    /**
     * Construct an affine transformation.
     */
    explicit Affine(std::array<float, 4> matrix) : matrix(matrix) {}

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "A:m=" << matrix[0] << "," << matrix[1] << "," << matrix[2] << "," << matrix[3];
        return ss.str();
    }

    DEPTHAI_SERIALIZE(Affine, matrix);
};

struct Perspective : OpBase {
    std::array<float, 9> matrix{1, 0, 0, 0, 1, 0, 0, 0, 1};

    ~Perspective() override;

    Perspective() = default;
    /**
     * Construct a perspective transformation.
     */
    explicit Perspective(std::array<float, 9> matrix) : matrix(matrix) {}

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "P:m=" << matrix[0] << "," << matrix[1] << "," << matrix[2] << "," << matrix[3] << "," << matrix[4] << "," << matrix[5] << "," << matrix[6] << ","
           << matrix[7] << "," << matrix[8];
        return ss.str();
    }

    DEPTHAI_SERIALIZE(Perspective, matrix);
};

struct FourPoints : OpBase {
    std::array<dai::Point2f, 4> src{dai::Point2f(0.0, 0.0), dai::Point2f(1.0, 0.0), dai::Point2f(1.0, 1.0), dai::Point2f(0.0, 1.0)};
    std::array<dai::Point2f, 4> dst{dai::Point2f(0.0, 0.0), dai::Point2f(1.0, 0.0), dai::Point2f(1.0, 1.0), dai::Point2f(0.0, 1.0)};
    bool normalized = false;

    ~FourPoints() override;

    FourPoints() = default;
    /**
     * Construct a four-point warp operation.
     */
    FourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalized = false) : src(src), dst(dst), normalized(normalized) {}

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "4P:s1=" << src[0].x << "," << src[0].y << ",s2=" << src[1].x << "," << src[1].y << ",s3=" << src[2].x << "," << src[2].y << ",s4=" << src[3].x
           << "," << src[3].y << "d1=" << dst[0].x << "," << dst[0].y << ",d2=" << dst[1].x << "," << dst[1].y << ",d3=" << dst[2].x << "," << dst[2].y
           << ",d4=" << dst[3].x << "," << dst[3].y << ",d=" << normalized;
        return ss.str();
    }

    DEPTHAI_SERIALIZE(FourPoints, src, dst, normalized);
};

struct Crop : OpBase {
    float width;
    float height;
    bool normalized;
    bool center;

    ~Crop() override;

    /**
     * Construct a default crop operation.
     */
    Crop() : width(0), height(0), normalized(true), center(true) {}
    /**
     * Construct a crop operation.
     */
    Crop(float width, float height, bool normalized = false, bool center = false) : width(width), height(height), normalized(normalized), center(center) {}

    /**
     * Clone this crop operation.
     */
    Crop clone() const {
        return *this;
    }

    std::string toStr() const override {
        std::stringstream ss;
        ss.imbue(std::locale(""));
        ss << "C:w=" << width << ",h=" << height << ",n=" << normalized << ",c=" << center;
        return ss.str();
    }

    DEPTHAI_SERIALIZE(Crop, width, height, normalized, center);
};

struct ManipOp {
    std::variant<Translate, Rotate, Resize, Flip, Affine, Perspective, FourPoints, Crop> op;

    ManipOp() = default;
    /**
     * Construct from a Translate operation.
     */
    ManipOp(Translate op) : op(op) {}  // NOLINT
    /**
     * Construct from a Rotate operation.
     */
    ManipOp(Rotate op) : op(op) {}  // NOLINT
    /**
     * Construct from a Resize operation.
     */
    ManipOp(Resize op) : op(op) {}  // NOLINT
    /**
     * Construct from a Flip operation.
     */
    ManipOp(Flip op) : op(op) {}  // NOLINT
    /**
     * Construct from an Affine operation.
     */
    ManipOp(Affine op) : op(op) {}  // NOLINT
    /**
     * Construct from a Perspective operation.
     */
    ManipOp(Perspective op) : op(op) {}  // NOLINT
    /**
     * Construct from a FourPoints operation.
     */
    ManipOp(FourPoints op) : op(op) {}  // NOLINT
    /**
     * Construct from a Crop operation.
     */
    ManipOp(Crop op) : op(op) {}  // NOLINT

    DEPTHAI_SERIALIZE(ManipOp, op);
};

class ImageManipOpsEnums {
   public:
    enum class Background : uint8_t { COLOR /* , REPLICATE, MIRROR */ };  // TODO(asahtik): replicate impl
    enum class ResizeMode : uint8_t { NONE, STRETCH, LETTERBOX, CENTER_CROP };
};

template <typename C>
class ImageManipOpsBase : public ImageManipOpsEnums {
   public:
    uint32_t outputWidth = 0;
    uint32_t outputHeight = 0;
    bool center = true;
    ResizeMode resizeMode = ResizeMode::NONE;
    Background background = Background::COLOR;
    uint32_t backgroundR = 0;
    uint32_t backgroundG = 0;
    uint32_t backgroundB = 0;
    Colormap colormap = Colormap::NONE;
    bool undistort = false;

    C operations;

    template <typename C2>
    /**
     * Copy configuration into another ImageManipOpsBase container.
     */
    void cloneTo(ImageManipOpsBase<C2>& to) const {
        to.outputWidth = outputWidth;
        to.outputHeight = outputHeight;
        to.center = center;
        to.resizeMode = resizeMode;
        to.background = background;
        to.backgroundR = backgroundR;
        to.backgroundG = backgroundG;
        to.backgroundB = backgroundB;
        to.colormap = colormap;
        to.undistort = undistort;

        to.operations.clear();
        to.operations.insert(to.operations.end(), operations.begin(), operations.end());
    }

    /**
     * Return true if any warp-like operation is configured.
     */
    bool hasWarp(const size_t inputWidth, const size_t inputHeight) const {
        return operations.size() > 0 || (outputWidth != 0 && outputWidth != inputWidth) || (outputHeight != 0 && outputHeight != inputHeight) || undistort;
    }

    /**
     * Append a manipulation operation.
     */
    ImageManipOpsBase& addOp(ManipOp op) {
        operations.push_back(op);
        return *this;
    }

    /**
     * Append a perspective transform operation.
     */
    ImageManipOpsBase& transformPerspective(std::array<float, 9> matrix) {
        operations.emplace_back(Perspective(matrix));
        return *this;
    }

    /**
     * Append an affine transform operation.
     */
    ImageManipOpsBase& transformAffine(std::array<float, 4> matrix) {
        operations.emplace_back(Affine(matrix));
        return *this;
    }

    /**
     * Append a four-point transform operation.
     */
    ImageManipOpsBase& transformFourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalizedCoords = false) {
        operations.emplace_back(FourPoints(src, dst, normalizedCoords));
        return *this;
    }

    /**
     * Append a horizontal flip operation.
     */
    ImageManipOpsBase& flipHorizontal(bool center = true) {
        operations.emplace_back(Flip(Flip::Direction::HORIZONTAL, center));
        return *this;
    }

    /**
     * Append a vertical flip operation.
     */
    ImageManipOpsBase& flipVertical(bool center = true) {
        operations.emplace_back(Flip(Flip::Direction::VERTICAL, center));
        return *this;
    }

    /**
     * Append a resize operation.
     */
    ImageManipOpsBase& resize(float width, float height, bool normalized = false) {
        operations.emplace_back(Resize(width, height, normalized));
        return *this;
    }

    /**
     * Append translate+crop operations.
     */
    ImageManipOpsBase& crop(float x, float y, float w, float h, bool normalized = false, bool center = false) {
        operations.emplace_back(Translate(-x, -y, normalized));
        operations.emplace_back(Crop(w, h, normalized, center));
        return *this;
    }

    /**
     * Append a fit-resize operation.
     */
    ImageManipOpsBase& resizeFit() {
        operations.emplace_back(Resize::fit());
        return *this;
    }

    /**
     * Append a fill-resize operation.
     */
    ImageManipOpsBase& resizeFill() {
        operations.emplace_back(Resize::fill());
        return *this;
    }

    /**
     * Append a resize-width operation keeping aspect ratio.
     */
    ImageManipOpsBase& resizeWidthKeepAspectRatio(float width, bool normalized = false) {
        operations.emplace_back(Resize(width, 0, normalized));
        return *this;
    }

    /**
     * Append a resize-height operation keeping aspect ratio.
     */
    ImageManipOpsBase& resizeHeightKeepAspectRatio(float height, bool normalized = false) {
        operations.emplace_back(Resize(0, height, normalized));
        return *this;
    }

    /**
     * Append a rotation in radians.
     */
    ImageManipOpsBase& rotateRadians(float angle, bool center = true, float offsetX = 0, float offsetY = 0, bool normalized = false) {
        operations.emplace_back(Rotate(angle, center, offsetX, offsetY, normalized));
        return *this;
    }

    /**
     * Append a rotation in degrees.
     */
    ImageManipOpsBase& rotateDegrees(float angle, bool center = true, float offsetX = 0, float offsetY = 0, bool normalized = false) {
        return rotateRadians(angle * 3.14159265358979323846f / 180.0f, center, offsetX, offsetY, normalized);
    }

    /**
     * Append a translation operation.
     */
    ImageManipOpsBase& translate(float offsetX, float offsetY, bool normalizedCoords = false) {
        operations.emplace_back(Translate(offsetX, offsetY, normalizedCoords));
        return *this;
    }

    /**
     * Set output size and reset resize mode.
     */
    ImageManipOpsBase& setOutputSize(float width, float height) {
        outputWidth = width;
        outputHeight = height;
        resizeMode = ResizeMode::NONE;
        return *this;
    }

    /**
     * Set output size and resize mode.
     */
    ImageManipOpsBase& setOutputResize(uint32_t width, uint32_t height, ResizeMode mode) {
        outputWidth = width;
        outputHeight = height;
        resizeMode = mode;
        center = true;
        return *this;
    }

    /**
     * Set whether resize operations are centered.
     */
    ImageManipOpsBase& setOutputCenter(bool c = true) {
        center = c;
        return *this;
    }

    /**
     * Set background color from RGB components.
     */
    ImageManipOpsBase& setBackgroundColor(uint32_t red, uint32_t green, uint32_t blue) {
        background = Background::COLOR;
        backgroundR = red;
        backgroundG = green;
        backgroundB = blue;
        return *this;
    }

    /**
     * Set background color from a grayscale value.
     */
    ImageManipOpsBase& setBackgroundColor(uint32_t val) {
        background = Background::COLOR;
        backgroundR = val;
        backgroundG = val;
        backgroundB = val;
        return *this;
    }

    // ImageManipOpsBase& setBackgroundReplicate() {
    //     throw std::runtime_error("Not implemented");
    //     background = Background::REPLICATE;
    //     return *this;
    // }

    /**
     * Set output colormap.
     */
    ImageManipOpsBase& setColormap(Colormap clr) {
        colormap = clr;
        return *this;
    }

    /**
     * Enable or disable undistortion.
     */
    ImageManipOpsBase& setUndistort(bool undistort) {
        this->undistort = undistort;
        return *this;
    }

    /**
     * Return whether undistortion is enabled.
     */
    bool getUndistort() const {
        return undistort;
    }

    /**
     * Return the list of operations.
     */
    const C& getOperations() const {
        return this->operations;
    }

    /**
     * Clear all operations.
     */
    ImageManipOpsBase& clear() {
        operations.clear();
        return *this;
    }

    DEPTHAI_SERIALIZE(
        ImageManipOpsBase, operations, outputWidth, outputHeight, center, resizeMode, background, backgroundR, backgroundG, backgroundB, colormap, undistort);
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
    using Container = std::vector<ManipOp>;

   public:
    ImageManipConfig() = default;
    virtual ~ImageManipConfig();

    using ResizeMode = ImageManipOpsBase<Container>::ResizeMode;

    // New config
    ImageManipOpsBase<Container> base;
    ImgFrame::Type outputFrameType = ImgFrame::Type::NONE;

    // Usable with runtime config only,
    // when ImageManipProperties.inputConfig.setWaitForMessage(true) is set
    bool reusePreviousImage = false;
    bool skipCurrentImage = false;

    // New API
    /**
     * Removes all operations from the list (does not affect output configuration)
     */
    ImageManipConfig& clearOps();
    /**
     * Crops the image to the specified rectangle
     * @param x X coordinate of the top-left corner
     * @param y Y coordinate of the top-left corner
     * @param w Width of the rectangle
     * @param h Height of the rectangle
     */
    ImageManipConfig& addCrop(uint32_t x, uint32_t y, uint32_t w, uint32_t h);
    /**
     * Crops the image to the specified rectangle
     * @param rect Rect to crop
     * @param normalizedCoords If true, the coordinates are normalized to range [0, 1] where 1 maps to the width/height of the image
     */
    ImageManipConfig& addCrop(dai::Rect rect, bool normalizedCoords = false);
    /**
     * Crops the image to the specified (rotated) rectangle
     * @param rect RotatedRect to crop
     * @param normalizedCoords If true, the coordinates are normalized to range [0, 1] where 1 maps to the width/height of the image
     */
    ImageManipConfig& addCropRotatedRect(dai::RotatedRect rotatedRect, bool normalizedCoords = false);
    /**
     * Rescales the image using the specified factors
     * @param scaleX Scale factor for the X axis
     * @param scaleY Scale factor for the Y axis. If not specified, scaleY is set to the same value as scaleX
     */
    ImageManipConfig& addScale(float scaleX, float scaleY = 0);
    /**
     * Rotates the image around its center by the specified angle in degrees
     * @param angle Angle in radians
     */
    ImageManipConfig& addRotateDeg(float angle);
    /**
     * Rotates the image around the specified point by the specified angle in degrees
     * @param angle Angle in radians
     * @param center Center of the rotation using normalized coordinates
     */
    ImageManipConfig& addRotateDeg(float angle, Point2f center);
    /**
     * Flips the image horizontally
     */
    ImageManipConfig& addFlipHorizontal();
    /**
     * Flips the image vertically
     */
    ImageManipConfig& addFlipVertical();
    /**
     * Applies an affine transformation to the image
     * @param matrix an array containing a 2x2 matrix representing the affine transformation
     */
    ImageManipConfig& addTransformAffine(std::array<float, 4> matrix);
    /**
     * Applies a perspective transformation to the image
     * @param matrix an array containing a 3x3 matrix representing the perspective transformation
     */
    ImageManipConfig& addTransformPerspective(std::array<float, 9> matrix);
    /**
     * Applies a perspective transformation to the image
     * @param src Source points
     * @param dst Destination points
     * @param normalizedCoords If true, the coordinates are normalized to range [0, 1] where 1 maps to the width/height of the image
     */
    ImageManipConfig& addTransformFourPoints(std::array<dai::Point2f, 4> src, std::array<dai::Point2f, 4> dst, bool normalizedCoords = false);
    /**
     * Sets the output size of the image
     * @param w Width of the output image
     * @param h Height of the output image
     * @param mode Resize mode. NONE - no resize, STRETCH - stretch to fit, LETTERBOX - keep aspect ratio and pad with background color, CENTER_CROP - keep
     * aspect ratio and crop
     */
    ImageManipConfig& setOutputSize(uint32_t w, uint32_t h, ResizeMode mode = ResizeMode::NONE);
    /**
     * Centers the content in the output image without resizing
     * @param c True to center the content, false otherwise
     */
    ImageManipConfig& setOutputCenter(bool c = true);
    /**
     * Sets the colormap to be applied to a grayscale image
     * @param colormap Colormap type to be applied
     */
    ImageManipConfig& setColormap(Colormap colormap);
    /**
     * Sets the rgb background color of the output image
     * @param red Red component of the background color
     * @param green Green component of the background color
     * @param blue Blue component of the background color
     */
    ImageManipConfig& setBackgroundColor(uint32_t red, uint32_t green, uint32_t blue);
    /**
     * Sets the grayscale background color of the output image
     * @param val Grayscale value of the background color
     */
    ImageManipConfig& setBackgroundColor(uint32_t val);
    /**
     * Sets the frame type of the output image
     * @param frameType Frame type of the output image
     */
    ImageManipConfig& setFrameType(ImgFrame::Type frameType);

    /**
     * Sets the undistort flag
     */
    ImageManipConfig& setUndistort(bool undistort);

    /**
     * Gets the undistort flag
     * @returns True if undistort is enabled, false otherwise
     */
    bool getUndistort() const;

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
     * Instruct ImageManip to not remove current image from its queue and use the same for next message.
     * @returns True to enable reuse, false otherwise
     */
    bool getReusePreviousImage() const;

    /**
     * Instructs ImageManip to skip current image and wait for next in queue.
     * @returns True to skip current image, false otherwise
     */
    bool getSkipCurrentImage() const;

    DEPTHAI_SERIALIZE(ImageManipConfig, base, outputFrameType, reusePreviousImage, skipCurrentImage);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ImageManipConfig;
    }
};
}  // namespace dai
