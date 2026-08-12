#include "depthai/beta/datatype/Map2D.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace beta {

namespace {

/// A plasma colormap color as 8-bit (blue, green, red), matching the OpenCV BGR lookup table.
struct PlasmaBgr {
    std::uint8_t b, g, r;
};

// Plasma colormap lookup table, taken from OpenCV's COLORMAP_PLASMA
// (opencv/modules/imgproc/src/colormap.cpp, Apache License 2.0), which itself embeds the
// "plasma" colormap created by Stéfan van der Walt and Nathaniel Smith for matplotlib
// (released under the CC0 license). OpenCV stores 256 float samples per channel and converts
// them to the 8-bit BGR lookup table with convertTo(CV_8U, 255.0); the resulting 256 8-bit
// BGR entries are reproduced here verbatim and verified bit-equal against
// cv2.applyColorMap(..., cv2.COLORMAP_PLASMA), which the source implementation in
// depthai-nodes uses for the Map2D visualization.
// clang-format off
constexpr std::array<PlasmaBgr, 256> PLASMA_LUT = {{
    {135, 8, 13}, {136, 7, 16}, {137, 7, 19}, {138, 7, 22}, {140, 6, 25}, {141, 6, 27}, {142, 6, 29}, {143, 6, 32},
    {144, 6, 34}, {145, 6, 36}, {145, 5, 38}, {146, 5, 40}, {147, 5, 42}, {148, 5, 44}, {149, 5, 46}, {150, 5, 47},
    {151, 5, 49}, {151, 5, 51}, {152, 4, 53}, {153, 4, 55}, {154, 4, 56}, {154, 4, 58}, {155, 4, 60}, {156, 4, 62},
    {156, 4, 63}, {157, 4, 65}, {158, 3, 67}, {158, 3, 68}, {159, 3, 70}, {159, 3, 72}, {160, 3, 73}, {161, 3, 75},
    {161, 2, 76}, {162, 2, 78}, {162, 2, 80}, {163, 2, 81}, {163, 2, 83}, {164, 2, 85}, {164, 1, 86}, {164, 1, 88},
    {165, 1, 89}, {165, 1, 91}, {166, 1, 92}, {166, 1, 94}, {166, 1, 96}, {167, 0, 97}, {167, 0, 99}, {167, 0, 100},
    {167, 0, 102}, {168, 0, 103}, {168, 0, 105}, {168, 0, 106}, {168, 0, 108}, {168, 0, 110}, {168, 0, 111}, {168, 0, 113},
    {168, 1, 114}, {168, 1, 116}, {168, 1, 117}, {168, 1, 119}, {168, 1, 120}, {168, 2, 122}, {168, 2, 123}, {168, 3, 125},
    {168, 3, 126}, {168, 4, 128}, {167, 4, 129}, {167, 5, 131}, {167, 5, 132}, {166, 6, 134}, {166, 7, 135}, {166, 8, 136},
    {165, 9, 138}, {165, 10, 139}, {165, 11, 141}, {164, 12, 142}, {164, 13, 143}, {163, 14, 145}, {163, 15, 146}, {162, 16, 148},
    {161, 17, 149}, {161, 19, 150}, {160, 20, 152}, {159, 21, 153}, {159, 22, 154}, {158, 23, 156}, {157, 24, 157}, {157, 25, 158},
    {156, 26, 160}, {155, 27, 161}, {154, 29, 162}, {154, 30, 163}, {153, 31, 165}, {152, 32, 166}, {151, 33, 167}, {150, 34, 168},
    {149, 35, 170}, {148, 36, 171}, {148, 38, 172}, {147, 39, 173}, {146, 40, 174}, {145, 41, 176}, {144, 42, 177}, {143, 43, 178},
    {142, 44, 179}, {141, 46, 180}, {140, 47, 181}, {139, 48, 182}, {138, 49, 183}, {137, 50, 184}, {136, 51, 186}, {136, 52, 187},
    {135, 53, 188}, {134, 55, 189}, {133, 56, 190}, {132, 57, 191}, {131, 58, 192}, {130, 59, 193}, {129, 60, 194}, {128, 61, 195},
    {127, 62, 196}, {126, 64, 197}, {125, 65, 198}, {124, 66, 199}, {123, 67, 200}, {122, 68, 201}, {122, 69, 202}, {121, 70, 203},
    {120, 71, 204}, {119, 73, 204}, {118, 74, 205}, {117, 75, 206}, {116, 76, 207}, {115, 77, 208}, {114, 78, 209}, {113, 79, 210},
    {113, 81, 211}, {112, 82, 212}, {111, 83, 213}, {110, 84, 213}, {109, 85, 214}, {108, 86, 215}, {107, 87, 216}, {106, 88, 217},
    {106, 90, 218}, {105, 91, 218}, {104, 92, 219}, {103, 93, 220}, {102, 94, 221}, {101, 95, 222}, {100, 97, 222}, {99, 98, 223},
    {99, 99, 224}, {98, 100, 225}, {97, 101, 226}, {96, 102, 226}, {95, 104, 227}, {94, 105, 228}, {93, 106, 229}, {93, 107, 229},
    {92, 108, 230}, {91, 110, 231}, {90, 111, 231}, {89, 112, 232}, {88, 113, 233}, {87, 114, 233}, {87, 116, 234}, {86, 117, 235},
    {85, 118, 235}, {84, 119, 236}, {83, 121, 237}, {82, 122, 237}, {81, 123, 238}, {81, 124, 239}, {80, 126, 239}, {79, 127, 240},
    {78, 128, 240}, {77, 129, 241}, {76, 131, 241}, {75, 132, 242}, {75, 133, 243}, {74, 135, 243}, {73, 136, 244}, {72, 137, 244},
    {71, 139, 245}, {70, 140, 245}, {69, 141, 246}, {68, 143, 246}, {68, 144, 247}, {67, 145, 247}, {66, 147, 247}, {65, 148, 248},
    {64, 149, 248}, {63, 151, 249}, {62, 152, 249}, {62, 154, 249}, {61, 155, 250}, {60, 156, 250}, {59, 158, 250}, {58, 159, 251},
    {57, 161, 251}, {56, 162, 251}, {56, 163, 252}, {55, 165, 252}, {54, 166, 252}, {53, 168, 252}, {52, 169, 252}, {51, 171, 253},
    {51, 172, 253}, {50, 174, 253}, {49, 175, 253}, {48, 177, 253}, {47, 178, 253}, {47, 180, 253}, {46, 181, 253}, {45, 183, 254},
    {44, 184, 254}, {44, 186, 254}, {43, 187, 254}, {42, 189, 254}, {42, 190, 254}, {41, 192, 254}, {41, 194, 253}, {40, 195, 253},
    {39, 197, 253}, {39, 198, 253}, {39, 200, 253}, {38, 202, 253}, {38, 203, 253}, {37, 205, 252}, {37, 206, 252}, {37, 208, 252},
    {37, 210, 252}, {36, 211, 251}, {36, 213, 251}, {36, 215, 251}, {36, 216, 250}, {36, 218, 250}, {36, 220, 249}, {37, 221, 249},
    {37, 223, 248}, {37, 225, 248}, {37, 226, 247}, {37, 228, 247}, {38, 230, 246}, {38, 232, 246}, {38, 233, 245}, {39, 235, 245},
    {39, 237, 244}, {39, 238, 243}, {39, 240, 243}, {39, 242, 242}, {38, 244, 241}, {37, 245, 241}, {36, 247, 240}, {33, 249, 240},
}};
// clang-format on

/// Truncate a float map value to an 8-bit colormap index exactly like NumPy's astype(np.uint8):
/// a C-style cast that truncates toward zero to a 32-bit signed integer and reduces the result
/// modulo 256. Values that do not fit the intermediate 32-bit integer, including NaN, convert
/// to 0, matching the x86 float-to-integer conversion NumPy compiles to.
std::uint8_t toColormapIndex(float value) {
    // Both bounds are exactly representable as float; 2^31 itself is already out of range.
    constexpr float LOWEST = static_cast<float>(std::numeric_limits<std::int32_t>::min());
    constexpr float HIGHEST = 2147483648.0f;
    if(!(value >= LOWEST && value < HIGHEST)) {
        return 0;
    }
    return static_cast<std::uint8_t>(static_cast<std::uint32_t>(static_cast<std::int32_t>(value)));
}

}  // namespace

Map2D::Map2D() = default;
Map2D::~Map2D() = default;

Map2D::Map2D(const std::vector<float>& map, size_t width, size_t height) : Map2D() {
    setMap(map, width, height);
}

void Map2D::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

void Map2D::setMap(const std::vector<float>& map, size_t width, size_t height) {
    setMap(span<const float>(map.data(), map.size()), width, height);
}

void Map2D::setMap(span<const float> map, size_t width, size_t height) {
    if(map.size() != width * height) {
        throw std::runtime_error("Map2D: map size does not match width*height");
    }
    data->setSize(map.size() * sizeof(float));
    std::memcpy(data->getData().data(), map.data(), map.size() * sizeof(float));
    this->width = width;
    this->height = height;
}

std::vector<float> Map2D::getMap() const {
    const auto& d = data->getData();
    if(d.empty()) {
        return {};
    }
    std::vector<float> map(d.size() / sizeof(float));
    std::memcpy(map.data(), d.data(), map.size() * sizeof(float));
    return map;
}

size_t Map2D::getWidth() const {
    return width;
}

size_t Map2D::getHeight() const {
    return height;
}

void Map2D::transformToInternal(const ImgTransformation& target) {
    if(!getTransformation().has_value()) {
        throw std::runtime_error("Source transformation is not set, cannot transform map.");
    }
    setTransformation(target);
}

Map2D Map2D::transformTo(const ImgTransformation& target) const {
    return TransformableCRTP<Map2D>::transformTo(target);
}

dai::VisualizeType Map2D::getVisualizationMessage() const {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setTimestamp(getTimestamp());
    frame->setTimestampDevice(getTimestampDevice());
    frame->setSequenceNum(getSequenceNum());
    if(transformation.has_value()) {
        frame->setTransformation(*transformation);
    }

    const auto map = getMap();

    // When any map value is below 1 the whole map is scaled by 255, replicating the source
    // implementation's np.any(mask < 1) check. A map with mixed ranges, e.g. values both below 1
    // and far above it, is therefore scaled as a whole and large values wrap around during the
    // 8-bit truncation, exactly like the source astype(np.uint8) conversion.
    bool scale = false;
    for(const float value : map) {
        if(value < 1.0f) {
            scale = true;
            break;
        }
    }

    // Interleaved BGR packing of the colormapped values, matching the source
    // cv2.applyColorMap(mask, cv2.COLORMAP_PLASMA) output packed by setCvFrame(..., BGR888i).
    std::vector<std::uint8_t> pixels(map.size() * 3);
    for(size_t i = 0; i < map.size(); i++) {
        const float value = scale ? map[i] * 255.0f : map[i];
        const auto& bgr = PLASMA_LUT[toColormapIndex(value)];
        pixels[i * 3] = bgr.b;
        pixels[i * 3 + 1] = bgr.g;
        pixels[i * 3 + 2] = bgr.r;
    }

    frame->setType(dai::ImgFrame::Type::BGR888i);
    frame->setSize(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    frame->setStride(static_cast<unsigned int>(width * 3));
    frame->setData(std::move(pixels));
    return frame;
}

}  // namespace beta
}  // namespace dai
