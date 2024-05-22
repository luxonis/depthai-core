#pragma once

#include <nlohmann/json.hpp>
#include <unordered_map>
#include <variant>
#include <vector>

#include "depthai/common/Colormap.hpp"
#include "depthai/common/Interpolation.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "utility/Serialization.hpp"
namespace dai {

namespace ImageManipOperations {

// Helper structs
struct SizeVal {
    enum class Type { ABSOLUTE = 0, RELATIVE, AUTO_FIT, AUTO_FILL };

   private:
    int valueAbs = 0;
    float valueRel = 0.0;
    Type type = Type::ABSOLUTE;

   public:
    static SizeVal absolute(int value) {
        SizeVal sv;
        sv.type = Type::ABSOLUTE;
        sv.valueAbs = value;
        return sv;
    }
    static SizeVal relative(float value) {
        SizeVal sv;
        sv.type = Type::RELATIVE;
        sv.valueRel = value;
        return sv;
    }
    static SizeVal fit() {
        SizeVal sv;
        sv.type = Type::AUTO_FIT;
        return sv;
    }
    static SizeVal fill() {
        SizeVal sv;
        sv.type = Type::AUTO_FILL;
        return sv;
    }
    int getAbsolute() const {
        if(type == Type::ABSOLUTE) return valueAbs;
        throw std::runtime_error("SizeVal is not absolute");
    }
    float getRelative() const {
        if(type == Type::RELATIVE) return valueRel;
        throw std::runtime_error("SizeVal is not relative");
    }
    Type getType() const {
        return type;
    }

    DEPTHAI_SERIALIZE(SizeVal, type, valueAbs, valueRel);
};
struct OffsetVal {
    enum class Type { ABSOLUTE = 0, RELATIVE };

   private:
    int valueAbs;
    float valueRel;
    Type type = Type::ABSOLUTE;

   public:
    static OffsetVal absolute(int value) {
        OffsetVal sv;
        sv.type = Type::ABSOLUTE;
        sv.valueAbs = value;
        return sv;
    }
    static OffsetVal relative(float value) {
        OffsetVal sv;
        sv.type = Type::RELATIVE;
        sv.valueRel = value;
        return sv;
    }
    int getAbsolute() const {
        if(type == Type::ABSOLUTE) return valueAbs;
        throw std::runtime_error("OffsetVal is not absolute");
    }
    float getRelative() const {
        if(type == Type::RELATIVE) return valueRel;
        throw std::runtime_error("OffsetVal is not relative");
    }
    Type getType() const {
        return type;
    }

    DEPTHAI_SERIALIZE(OffsetVal, type, valueAbs, valueRel);
};
// Structs that define actions

// Resize canvas
struct CanvasResize {
    enum class CanvasBackground { COLOR, REPLICATE, MIRROR };
    enum class CanvasAnchor { TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT, CENTER };

   private:
    SizeVal width;
    SizeVal height;

    CanvasAnchor anchorType = CanvasAnchor::CENTER;

    CanvasBackground background = CanvasBackground::COLOR;
    uint8_t red = 0, green = 0, blue = 0;

   public:
    CanvasResize() = default;
    CanvasResize(SizeVal width, SizeVal height, CanvasAnchor anchorType, CanvasBackground background, uint8_t red, uint8_t green, uint8_t blue)
        : width(width), height(height), anchorType(anchorType), background(background), red(red), green(green), blue(blue) {}

    DEPTHAI_SERIALIZE(CanvasResize, width, height, anchorType, background, red, green, blue);
};

struct ImageResize {
   private:
    SizeVal width;
    SizeVal height;

   public:
    ImageResize() = default;
    ImageResize(SizeVal width, SizeVal height) : width(width), height(height) {}

    DEPTHAI_SERIALIZE(ImageResize, width, height);
};

struct ImageTransform2D {
   private:
    int rotation = 0;
    OffsetVal offsetX;
    OffsetVal offsetY;

   public:
    ImageTransform2D() = default;
    ImageTransform2D(int rotation, OffsetVal offsetX, OffsetVal offsetY) : rotation(rotation), offsetX(offsetX), offsetY(offsetY) {}

    DEPTHAI_SERIALIZE(ImageTransform2D, rotation, offsetX, offsetY);
};

struct ImageWarp {
    enum class Type { FOUR_POINTS, MATRIX };

   private:
    std::array<dai::Point2f, 4> points;
    std::array<float, 9> mat{1, 0, 0, 0, 1, 0, 0, 0, 1};

    Type type = Type::MATRIX;

   public:
    static ImageWarp fourPoints(const std::array<dai::Point2f, 4>& points) {
        ImageWarp iw;
        iw.type = Type::FOUR_POINTS;
        iw.points = points;
        return iw;
    }
    static ImageWarp matrix(const std::array<float, 9>& mat) {
        ImageWarp iw;
        iw.type = Type::MATRIX;
        iw.mat = mat;
        return iw;
    }
    std::array<dai::Point2f, 4> getPoints() const {
        if(type == Type::FOUR_POINTS) return points;
        throw std::runtime_error("ImageWarp is not four points");
    }
    std::array<float, 9> getMatrix() const {
        if(type == Type::MATRIX) return mat;
        throw std::runtime_error("ImageWarp is not matrix");
    }
    Type getType() const {
        return type;
    }

    DEPTHAI_SERIALIZE(ImageWarp, type, points, mat);
};

struct ImageFlip {
    enum class Type { HORIZONTAL, VERTICAL };

   private:
    Type type = Type::HORIZONTAL;

   public:
    static ImageFlip horizontal() {
        ImageFlip flip;
        flip.type = Type::HORIZONTAL;
        return flip;
    }
    static ImageFlip vertical() {
        ImageFlip flip;
        flip.type = Type::VERTICAL;
        return flip;
    }
    Type getType() const {
        return type;
    }

    DEPTHAI_SERIALIZE(ImageFlip, type);
};

struct ImageColormap {
    Colormap type = Colormap::NONE;

    ImageColormap() = default;
    ImageColormap(Colormap type) : type(type) {}

    DEPTHAI_SERIALIZE(ImageColormap, type);
};

struct ManipOp {
    enum class Type { NONE, CANVAS_RESIZE, IMAGE_TRANSFORM_2D, IMAGE_WARP, IMAGE_RESIZE, IMAGE_FLIP, IMAGE_COLORMAP };

   private:
    Type type = Type::NONE;
    union {
        CanvasResize canvasResize;
        ImageTransform2D imageTransform2D;
        ImageWarp imageWarp;
        ImageResize imageResize;
        ImageFlip imageFlip;
        ImageColormap imageColormap;
    } op;

public:
    ManipOp() = default;
    ManipOp(CanvasResize canvasResize) : type(Type::CANVAS_RESIZE), op({.canvasResize = canvasResize}) {}
    ManipOp(ImageTransform2D imageTransform2D) : type(Type::IMAGE_TRANSFORM_2D), op({.imageTransform2D = imageTransform2D}) {}
    ManipOp(ImageWarp imageWarp) : type(Type::IMAGE_WARP), op({.imageWarp = imageWarp}) {}
    ManipOp(ImageResize imageResize) : type(Type::IMAGE_RESIZE), op({.imageResize = imageResize}) {}
    ManipOp(ImageFlip imageFlip) : type(Type::IMAGE_FLIP), op({.imageFlip = imageFlip}) {}
    ManipOp(ImageColormap imageColormap) : type(Type::IMAGE_COLORMAP), op({.imageColormap = imageColormap}) {}

    Type getType() const {
        return type;
    }
    CanvasResize getCanvasResize() const {
        if(type == Type::CANVAS_RESIZE) return op.canvasResize;
        throw std::runtime_error("ManipOp is not CanvasResize");
    }
    ImageTransform2D getImageTransform2D() const {
        if(type == Type::IMAGE_TRANSFORM_2D) return op.imageTransform2D;
        throw std::runtime_error("ManipOp is not ImageTransform2D");
    }
    ImageWarp getImageWarp() const {
        if(type == Type::IMAGE_WARP) return op.imageWarp;
        throw std::runtime_error("ManipOp is not ImageWarp");
    }
    ImageResize getImageResize() const {
        if(type == Type::IMAGE_RESIZE) return op.imageResize;
        throw std::runtime_error("ManipOp is not ImageResize");
    }
    ImageFlip getImageFlip() const {
        if(type == Type::IMAGE_FLIP) return op.imageFlip;
        throw std::runtime_error("ManipOp is not ImageFlip");
    }
    ImageColormap getImageColormap() const {
        if(type == Type::IMAGE_COLORMAP) return op.imageColormap;
        throw std::runtime_error("ManipOp is not ImageColormap");
    }

    NOP_STRUCTURE(ManipOp, type, op);
};

}  // namespace ImageManipOperations

namespace ns {
void to_json(nlohmann::json& j, const dai::ImageManipOperations::ManipOp& p) { // NOLINT
    nlohmann::json opJson = nlohmann::json{};
    switch(p.getType()) {
        case dai::ImageManipOperations::ManipOp::Type::NONE:
            break;
        case dai::ImageManipOperations::ManipOp::Type::CANVAS_RESIZE:
            opJson = p.getCanvasResize();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_TRANSFORM_2D:
            opJson = p.getImageTransform2D();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_WARP:
            opJson = p.getImageWarp();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_RESIZE:
            opJson = p.getImageResize();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_FLIP:
            opJson = p.getImageFlip();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_COLORMAP:
            opJson = p.getImageColormap();
            break;
    }
    j = nlohmann::json{{"type", p.getType()}, {"op", opJson}};
}

void from_json(const nlohmann::json& j, dai::ImageManipOperations::ManipOp& p) { // NOLINT
    switch(j.at("type").get<dai::ImageManipOperations::ManipOp::Type>()) {
        case dai::ImageManipOperations::ManipOp::Type::NONE:
            break;
        case dai::ImageManipOperations::ManipOp::Type::CANVAS_RESIZE:
            p = j.at("op").get<dai::ImageManipOperations::CanvasResize>();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_TRANSFORM_2D:
            p = j.at("op").get<dai::ImageManipOperations::ImageTransform2D>();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_WARP:
            p = j.at("op").get<dai::ImageManipOperations::ImageWarp>();
            break;
        case ImageManipOperations::ManipOp::Type::IMAGE_RESIZE:
            p = j.at("op").get<dai::ImageManipOperations::ImageResize>();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_FLIP:
            p = j.at("op").get<dai::ImageManipOperations::ImageFlip>();
            break;
        case dai::ImageManipOperations::ManipOp::Type::IMAGE_COLORMAP:
            p = j.at("op").get<dai::ImageManipOperations::ImageColormap>();
            break;
    }
}
}  // namespace ns

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

    std::vector<ImageManipOperations::ManipOp> operations;

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
                      interpolation
                      );
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ImageManipConfig;
    };

    /// Retrieve which interpolation method to use
    dai::Interpolation getInterpolation() const;
};

}  // namespace dai
