#include "utilities/GpioLine.hpp"

#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace dai {
namespace gpio {
namespace {

void closeFd(int& fd) {
    if(fd >= 0) {
        close(fd);
        fd = -1;
    }
}

std::runtime_error systemError(const std::string& message) {
    return std::runtime_error(message + ": " + std::strerror(errno));
}

}  // namespace

Line::Line(const std::string& chipPath, unsigned int offset, Direction direction, Bias bias, const std::string& consumer) {
    int chipFd = open(chipPath.c_str(), O_RDONLY | O_CLOEXEC);
    if(chipFd < 0) {
        throw systemError("Failed to open GPIO chip " + chipPath);
    }

    gpio_v2_line_request request{};
    request.offsets[0] = offset;
    request.num_lines = 1;
    std::snprintf(request.consumer, sizeof(request.consumer), "%s", consumer.c_str());

    switch(direction) {
        case Direction::Input:
            request.config.flags |= GPIO_V2_LINE_FLAG_INPUT;
            break;
        case Direction::Output:
            request.config.flags |= GPIO_V2_LINE_FLAG_OUTPUT;
            break;
    }

    switch(bias) {
        case Bias::Disabled:
            request.config.flags |= GPIO_V2_LINE_FLAG_BIAS_DISABLED;
            break;
        case Bias::PullDown:
            request.config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN;
            break;
        case Bias::PullUp:
            request.config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
            break;
        case Bias::AsIs:
            break;
    }

    if(ioctl(chipFd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
        int savedErrno = errno;
        close(chipFd);
        errno = savedErrno;
        throw systemError("Failed to request GPIO line " + std::to_string(offset));
    }

    close(chipFd);
    fd = request.fd;
}

Line::~Line() {
    closeFd(fd);
}

Line::Line(Line&& other) noexcept : fd(other.fd) {
    other.fd = -1;
}

Line& Line::operator=(Line&& other) noexcept {
    if(this != &other) {
        closeFd(fd);
        fd = other.fd;
        other.fd = -1;
    }
    return *this;
}

int Line::getValue() const {
    gpio_v2_line_values values{};
    values.mask = 1;

    if(ioctl(fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
        throw systemError("Failed to read GPIO line value");
    }

    return (values.bits & 1) ? 1 : 0;
}

void Line::setValue(int value) const {
    gpio_v2_line_values values{};
    values.mask = 1;
    values.bits = value ? 1 : 0;

    if(ioctl(fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &values) < 0) {
        throw systemError("Failed to write GPIO line value");
    }
}

}  // namespace gpio
}  // namespace dai
