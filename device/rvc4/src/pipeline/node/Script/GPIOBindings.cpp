#include "GPIOBindings.hpp"
#include "utilities/AuxConnector.hpp"
#include "utilities/GpioLine.hpp"

// std
#include <atomic>
#include <bitset>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <variant>

// pybind11
#include <pybind11/chrono.h>
#include <pybind11/embed.h>  // everything needed for embedding
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

// // mdk
// #include "DrvGpio.h"
// #include "OsDrvGpio.h"

// // libraries
// #include <spdlog/fmt/fmt.h>

constexpr char GPIO_CHIP_PATH[] = "/dev/gpiochip0";

PYBIND11_EMBEDDED_MODULE(GPIO, m) {
    python::gpio::bind(m);
}

namespace py = pybind11;

namespace python {
namespace gpio {

static std::atomic<bool> initialized{false};

// // Gets the PWM block that the given gpio is connected to. MX has 6 PWM blocks
// int getPwmBlock(int gpio) {
//     switch(gpio) {
//         case 34:
//         case 62:
//             return 0;
//         case 63:
//             return 1;
//         case 45:
//         case 59:
//             return 2;
//         case 37:
//         case 46:
//         case 60:
//             return 3;
//         case 48:
//         case 61:
//             return 4;
//         case 21:
//         case 49:
//         case 58:
//             return 5;
//         default: // This GPIO is not connected to a PWM block
//             return -1;
//     }
// }

// // Gets the mode in which gpio has to be to enable the PWM. We set the gpio mode with `DrvGpioSetMode()`
// int getPwmMode(int gpio) {
//     switch(gpio) {
//         case 62:
//         case 63:
//             return 0;
//         case 34:
//         case 37:
//             return 1;
//         case 21:
//         case 45:
//         case 46:
//         case 48:
//         case 49:
//         case 59:
//         case 60:
//         case 61:
//             return 2;
//         case 58:
//             return 3;
//         default: // This GPIO is not connected to a PWM block
//             return -1;
//     }
// }

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

// POS_EDGE_INT  0
// NEG_EDGE_INT  1
// POS_LEVEL_INT 2
// NEG_LEVEL_INT 3
enum class Edge {
    RISING = 0,
    FALLING = 1,
    LEVEL_HIGH = 2,
    LEVEL_LOW = 3,
};

std::unordered_map<int, dai::gpio::Line> gpioHandleMap;
std::unordered_map<int, py::object> gpioCallbackMap;

struct ConnectorLine {
    uint32_t pin;
    dai::gpio::Line line;
};

dai::aux::GpioPins connectorPins;
std::unordered_map<std::string, ConnectorLine> connectorLineMap;

std::thread interruptHandlerThread;

std::condition_variable gpioCv;
std::mutex gpioMutex;

// Max 256 GPIOs
std::bitset<256> gpioEventMap;
std::bitset<256> gpioEventCallbackMask;

dai::gpio::Direction toDirection(Direction dir) {
    switch(dir) {
        case Direction::IN:
            return dai::gpio::Direction::Input;
        case Direction::OUT:
            return dai::gpio::Direction::Output;
        default:
            throw std::runtime_error("Invalid direction set for gpio!");
    }
}

dai::gpio::Bias toBias(PullDownUp pud) {
    switch(pud) {
        case PullDownUp::PULL_NONE:
            return dai::gpio::Bias::Disabled;
        case PullDownUp::PULL_DOWN:
            return dai::gpio::Bias::PullDown;
        case PullDownUp::PULL_UP:
            return dai::gpio::Bias::PullUp;
        default:
            throw std::runtime_error("Invalid pull setting for gpio!");
    }
}

bool releaseConnectorLine(const std::string& name) {
    auto held = connectorLineMap.find(name);
    if(held == connectorLineMap.end()) {
        return false;
    }
    uint32_t pin = held->second.pin;
    connectorLineMap.erase(held);
    if(auto error = connectorPins.release(pin)) {
        throw std::runtime_error("Cannot release \"" + name + "\": " + error.message());
    }
    return true;
}

// void gpioCallbackFunction(void * vgpio) {
//     int gpio = (int) vgpio;

//     // mark that GPIO interrupt fired
//     rtems_interrupt_level lvl;
//     rtems_interrupt_disable(lvl);

//     std::unique_lock<std::mutex> lock(gpioMutex);
//     gpioEventMap.set(gpio, true);
//     gpioCv.notify_all();

//     rtems_interrupt_enable(lvl);

// }

void bind(py::module_ m) {
    // mark initialized, if already initilized, exit
    if(initialized.exchange(true)) {
        return;
    }

    // Setup the GPIO

    // // Init GPIO
    // OsDrvGpioInit(nullptr);

    // // Set a positive mask
    // gpioEventCallbackMask.reset();

    // // Init interruptHandlerThread
    // interruptHandlerThread = std::thread([](){
    //     // wait for interrupt and do callbacks
    //     auto gpioEventMapCopy = gpioEventMap;
    //     while(true){

    //         {
    //             std::unique_lock<std::mutex> lock(gpioMutex);
    //             gpioCv.wait(lock, [](){ return (gpioEventMap & gpioEventCallbackMask).any(); });
    //             // A new event arrived, make a copy of callback events
    //             gpioEventMapCopy = gpioEventMap & gpioEventCallbackMask;
    //             // Remove all events with callback associated with it
    //             gpioEventMap &= ~gpioEventCallbackMask;
    //         }

    //         // Process events (call callbacks if they exist)
    //         py::gil_scoped_acquire gil;
    //         for(unsigned i = 0; i < gpioEventMapCopy.size(); i++){
    //             if(gpioEventMapCopy.test(i)){
    //                 // check if callback is attached
    //                 if(gpioCallbackMap.count(i) > 0){
    //                     gpioCallbackMap[i](i);
    //                 }
    //             }
    //         }

    //     }
    // });

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
            auto direction = toDirection(dir);
            auto bias = toBias(pud);
            gpioHandleMap.erase(gpio);
            try {
                gpioHandleMap.emplace(gpio, dai::gpio::Line(GPIO_CHIP_PATH, gpio, direction, bias, "depthai-device"));
            } catch(const std::exception& error) {
                auto hint = dai::aux::connectorHintForTlmm(gpio, dai::aux::lineConsumer(GPIO_CHIP_PATH, gpio));
                if(!hint) {
                    throw;
                }
                throw std::runtime_error(std::string(error.what()) + ". " + *hint);
            }
            return 0;
        },
        py::arg("gpio"),
        py::arg("dir") = Direction::IN,
        py::arg("pud") = PullDownUp::PULL_NONE,
        py::arg("exclusive") = false);

    m.def(
        "setup",
        [](const std::string& name, Direction dir, PullDownUp pud, bool exclusive) -> int {
            (void)exclusive;
            auto direction = toDirection(dir);
            auto bias = toBias(pud);
            releaseConnectorLine(name);
            auto acquired = connectorPins.acquire(name, bias);
            if(const auto* error = std::get_if<std::string>(&acquired)) {
                throw std::runtime_error(*error);
            }
            const auto& line = std::get<dai::aux::AcquiredLine>(acquired);
            try {
                connectorLineMap.emplace(
                    name, ConnectorLine{line.pin, dai::gpio::Line(line.chip.string(), line.offset, direction, dai::gpio::Bias::AsIs, "depthai-device")});
            } catch(const std::exception& error) {
                connectorPins.release(line.pin);
                throw std::runtime_error(dai::aux::gpioSetupError(name, error.what()));
            }
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

    m.def("release", [](const std::string& name) -> int { return releaseConnectorLine(name) ? 0 : 1; });

    m.def("write", [](const std::string& name, int value) -> int {
        auto held = connectorLineMap.find(name);
        if(held == connectorLineMap.end()) {
            return 1;
        }
        held->second.line.setValue(value);
        return 0;
    });

    m.def("read", [](const std::string& name) -> int {
        auto held = connectorLineMap.find(name);
        return held == connectorLineMap.end() ? -1 : held->second.line.getValue();
    });

    // m.def("waitInterruptEvent", [](int gpio){
    //     // Waits for interrupt and returns GPIO which triggered it
    //     py::gil_scoped_release release;
    //     {
    //         // lock
    //         std::unique_lock<std::mutex> lock(gpioMutex);
    //         // wait only on events not having a callback associated
    //         gpioCv.wait(lock, [&](){
    //             if(gpio < 0){
    //                 return (gpioEventMap & ~gpioEventCallbackMask).any();
    //             } else {
    //                 return gpioEventMap.test(gpio);
    //             }
    //         });
    //         // Check only events not covered by callbacks
    //         auto events = gpioEventMap &~ gpioEventCallbackMask;

    //         if(gpio < 0){
    //             // check if any events
    //             bool ret = events.any();
    //             // Clear all non callback events
    //             gpioEventMap &= gpioEventCallbackMask;
    //             return ret;
    //         } else {
    //             // check if a specific event
    //             bool ret = events.test(gpio);
    //             gpioEventMap.reset(gpio);
    //             return ret;
    //         }
    //     }
    // }, py::arg() = -1);

    // m.def("hasInterruptEvent", [](int gpio){

    //     {
    //         // lock
    //         std::unique_lock<std::mutex> lock(gpioMutex);

    //         // Check only events not covered by callbacks
    //         auto events = gpioEventMap &~ gpioEventCallbackMask;

    //         if(gpio < 0){
    //             // check if any events
    //             bool ret = events.any();
    //             // Clear all non callback events
    //             gpioEventMap &= gpioEventCallbackMask;
    //             return ret;
    //         } else {
    //             // check if a specific event
    //             bool ret = events.test(gpio);
    //             gpioEventMap.reset(gpio);
    //             return ret;
    //         }
    //     }

    // }, py::arg() = -1);

    // m.def("setInterrupt", [](int gpio, Edge edge, int priority, std::optional<py::function> callback) -> int {
    //     if(gpioHandleMap.count(gpio) > 0){
    //         // Prepare GPIO handle
    //         OsDrvGpioHandleArray array;
    //         array.count = 1;
    //         array.handle[0] = gpioHandleMap[gpio];

    //         // Prepare interrupt
    //         OsDrvGpioIrqHandle irqHandle;
    //         // Convert 0-255 priority to 15-0
    //         irqHandle.priority = static_cast<uint8_t>((255 - priority) / 16);
    //         irqHandle.type = static_cast<int>(edge);
    //         irqHandle.callback = gpioCallbackFunction;
    //         irqHandle.argument = (void*) gpio;
    //         auto code = OsDrvGpioEnableInterrupt(&array, &irqHandle);

    //         if(code != RTEMS_SUCCESSFUL){
    //             return code;
    //         }

    //         // Add to map
    //         if(callback.has_value()){
    //             gpioCallbackMap[gpio] = *callback;
    //             {
    //                 std::unique_lock<std::mutex> l(gpioMutex);
    //                 gpioEventCallbackMask.set(gpio);
    //             }
    //         }

    //         return 0;

    //     } else {
    //         return -1;
    //     }
    // }, py::arg(), py::arg(), py::arg(), py::arg() = std::nullopt);

    // m.def("clearInterrupt", [](int gpio) -> int {

    //     if(gpioHandleMap.count(gpio) > 0){
    //         // Prepare GPIO handle
    //         OsDrvGpioHandleArray array;
    //         array.count = 1;
    //         array.handle[0] = gpioHandleMap[gpio];

    //         auto code = OsDrvGpioDisableInterrupt(&array);

    //         if(code != RTEMS_SUCCESSFUL){
    //             return code;
    //         }

    //         // Clear callback if set
    //         if(gpioCallbackMap.count(gpio) > 0){
    //             {
    //                 std::unique_lock<std::mutex> l(gpioMutex);
    //                 gpioEventCallbackMask.reset(gpio);
    //             }
    //             gpioCallbackMap.erase(gpio);
    //         }

    //         return 0;

    //     } else {
    //         return -1;
    //     }

    // });

    // m.def("setPwm", [](int gpio, int highCount, int lowCount, int repeat){
    //     auto pwmBlock = getPwmBlock(gpio);
    //     if (pwmBlock == -1) {
    //         throw std::invalid_argument(fmt::format("PWM not supported on GPIO {}", gpio));
    //     }
    //     DrvGpioSetPwm(pwmBlock, repeat, 0, highCount, lowCount);

    //     return 0;
    // }, py::arg("gpio"), py::arg("highCount"), py::arg("lowCount"), py::arg("repeat") = 0);
    // m.def("enablePwm", [](int gpio, bool enable){
    //     auto pwmBlock = getPwmBlock(gpio);
    //     auto mode = getPwmMode(gpio);
    //     if (pwmBlock == -1 || mode == -1) {
    //         throw std::invalid_argument(fmt::format("PWM not supported on GPIO {}", gpio));
    //     }
    //     // Set the GPIO to PWM mode
    //     DrvGpioSetMode(gpio, mode);
    //     // Enable the PWM block
    //     DrvGpioEnPwm(pwmBlock, enable);

    //     return 0;
    // });
}

}  // namespace gpio
}  // namespace python
