#include "GPIOBindings.hpp"
#include "utilities/GpioLine.hpp"

#include <atomic>
#include <bitset>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>

#include <pybind11/chrono.h>
#include <pybind11/embed.h>  // everything needed for embedding
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

constexpr char GPIO_CHIP_PATH[] = "/dev/gpiochip0";

PYBIND11_EMBEDDED_MODULE(GPIO, m) {
    python::gpio::bind(m);
}

namespace py = pybind11;

namespace python {
namespace gpio {

static std::atomic<bool> initialized{false};

enum class Direction {
    IN = 0,
    OUT,
};

enum class State {
    LOW = 0,
    HIGH = 1,
};

enum class PullDownUp {
    PULL_NONE = 0,
    PULL_DOWN,
    PULL_UP,
};

enum class Edge {
    RISING = 0,
    FALLING = 1,
    LEVEL_HIGH = 2,
    LEVEL_LOW = 3,
};

std::unordered_map<int, dai::gpio::Line> gpioHandleMap;
std::unordered_map<int, py::object> gpioCallbackMap;
std::thread interruptHandlerThread;

std::condition_variable gpioCv;
std::mutex gpioMutex;

std::bitset<256> gpioEventMap;
std::bitset<256> gpioEventCallbackMask;

void bind(py::module_ m) {
    if(initialized.exchange(true)) {
        return;
    }

    py::enum_<Direction>(m, "Direction").value("IN", Direction::IN).value("OUT", Direction::OUT).export_values();

    py::enum_<State>(m, "State").value("LOW", State::LOW).value("HIGH", State::HIGH).export_values();

    py::enum_<PullDownUp>(m, "PullDownUp")
        .value("PULL_NONE", PullDownUp::PULL_NONE)
        .value("PULL_DOWN", PullDownUp::PULL_DOWN)
        .value("PULL_UP", PullDownUp::PULL_UP)
        .export_values();

    py::enum_<Edge>(m, "Edge")
        .value("RISING", Edge::RISING)
        .value("FALLING", Edge::FALLING)
        .value("LEVEL_HIGH", Edge::LEVEL_HIGH)
        .value("LEVEL_LOW", Edge::LEVEL_LOW)
        .export_values();

    m.def(
        "setup",
        [](int gpio, Direction dir, PullDownUp pud, bool exclusive) -> int {
            (void)exclusive;

            dai::gpio::Direction direction;
            switch(dir) {
                case Direction::IN:
                    direction = dai::gpio::Direction::Input;
                    break;
                case Direction::OUT:
                    direction = dai::gpio::Direction::Output;
                    break;
                default:
                    throw std::runtime_error("Invalid direction set for gpio!");
            }

            dai::gpio::Bias bias;
            switch(pud) {
                case PullDownUp::PULL_NONE:
                    bias = dai::gpio::Bias::Disabled;
                    break;
                case PullDownUp::PULL_DOWN:
                    bias = dai::gpio::Bias::PullDown;
                    break;
                case PullDownUp::PULL_UP:
                    bias = dai::gpio::Bias::PullUp;
                    break;
                default:
                    throw std::runtime_error("Invalid pull setting for gpio!");
            }

            gpioHandleMap.erase(gpio);
            gpioHandleMap.emplace(gpio, dai::gpio::Line(GPIO_CHIP_PATH, gpio, direction, bias, "depthai-device"));
            return 0;
        },
        py::arg("gpio"),
        py::arg("dir") = Direction::IN,
        py::arg("pud") = PullDownUp::PULL_NONE,
        py::arg("exclusive") = false);

    m.def("release", [](int gpio) -> int {
        if(gpioHandleMap.count(gpio) > 0) {
            gpioHandleMap.erase(gpio);
            return 0;
        }

        return 1;
    });

    m.def("write", [](int gpio, int value) -> int {
        if(gpioHandleMap.count(gpio) > 0) {
            // TODO(Morato) handle case if pin is in input mode
            gpioHandleMap.at(gpio).setValue(value);
            return 0;
        }
        return 1;
    });

    m.def("read", [](int gpio) -> int {
        if(gpioHandleMap.count(gpio) > 0) {
            // TODO(Morato) handle the case if the pin is in output mode
            return gpioHandleMap.at(gpio).getValue();
        }
        return -1;
    });
}

}  // namespace gpio
}  // namespace python
