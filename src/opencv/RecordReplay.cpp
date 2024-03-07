#include "depthai/utility/RecordReplay.hpp"

namespace dai {
namespace utility {

VideoRecorderOpenCV::~VideoRecorderOpenCV() {
    close();
}

bool VideoRecorderOpenCV::init(std::string filePath, int width, int height, int fps) {
    this->filePath = filePath;
    this->width = width;
    this->height = height;
    this->fps = fps;

    if (writer.isOpened()) writer.release();

    writer.open(filePath, cv::VideoWriter::fourcc('H', '2', '6', '4'), fps, cv::Size(width, height));
    return true;
}

bool VideoRecorderOpenCV::write(span<const uint8_t>& data) {
    if (!writer.isOpened()) return false;

    // TODO(asahtik): Maybe add support for other formats, currently requires cv::Mat data to be passed
    assert(data.size() == (size_t)(width * height * 3) && "Data size does not match frame size");

    cv::Mat img(height, width, CV_8UC3, (void*)data.data());

    writer.write(img);

    return true;
}

bool VideoRecorderOpenCV::close() {
    if (writer.isOpened()) writer.release();
    return true;
}

}  // namespace utility
}  // namespace dai
