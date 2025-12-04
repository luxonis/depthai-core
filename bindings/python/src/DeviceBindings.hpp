#pragma once

// pybind
#include "pybind11_common.hpp"

// depthai
#include "depthai/device/Device.hpp"

// Searches for available devices (as Device constructor)
// but pooling, to check for python interrupts, and releases GIL in between

template <typename DEVICE, class... Args>
static dai::DeviceInfo deviceSearchHelper(Args&&... args) {
    bool found;
    dai::DeviceInfo deviceInfo;
    // releases python GIL
    py::gil_scoped_release release;
    std::tie(found, deviceInfo) = DEVICE::getAnyAvailableDevice(DEVICE::getDefaultSearchTime(), []() {
        py::gil_scoped_acquire acquire;
        if(PyErr_CheckSignals() != 0) throw py::error_already_set();
    });

    // if no devices found, then throw
    if(!found) {
        auto numConnected = DEVICE::getAllAvailableDevices().size();
        if(numConnected > 0) {
            throw std::runtime_error("No available devices (" + std::to_string(numConnected) + " connected, but in use)");
        } else {
            throw std::runtime_error("No available devices");
        }
    }

    return deviceInfo;
}

struct DeviceBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};
