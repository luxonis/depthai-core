#pragma once

#include <string>

namespace dai {
namespace gpio {

enum class Direction {
    Input,
    Output,
};

enum class Bias {
    Disabled,
    PullDown,
    PullUp,
};

class Line {
   public:
    Line(const std::string& chipPath, unsigned int offset, Direction direction, Bias bias, const std::string& consumer);
    ~Line();

    Line(const Line&) = delete;
    Line& operator=(const Line&) = delete;

    Line(Line&& other) noexcept;
    Line& operator=(Line&& other) noexcept;

    int getValue() const;
    void setValue(int value) const;

   private:
    int fd = -1;
};

}  // namespace gpio
}  // namespace dai
