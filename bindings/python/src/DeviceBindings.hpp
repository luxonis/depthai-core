#pragma once

#include "pybind11_common.hpp"

#include "depthai/device/Device.hpp"

template <typename DEVICE, class... Args>  // Device search like the Device constructor, but polls for Python interrupts and releases the GIL
static dai::DeviceInfo deviceSearchHelper(Args&&... args) {
    bool found;
    dai::DeviceInfo deviceInfo;
    py::gil_scoped_release release;
    std::tie(found, deviceInfo) = DEVICE::getAnyAvailableDevice(DEVICE::getDefaultSearchTime(), []() {
        py::gil_scoped_acquire acquire;
        if(PyErr_CheckSignals() != 0) throw py::error_already_set();
    });

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
