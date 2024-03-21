#pragma once

// pybind
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/NodeGroup.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"
#include "pybind11_common.hpp"
#include <pybind11/smart_holder.h>
// depthai
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/HostNode.hpp"


struct NodeBindings {
    static void addToCallstack(std::deque<StackFunction>& callstack);
    static std::vector<std::pair<py::handle, std::function<std::shared_ptr<dai::Node>(dai::Pipeline&, py::object class_)>>> getNodeCreateMap();
 private:
    static void bind(pybind11::module& m, void* pCallstack);
};
