#pragma once

// pybind
#include "depthai/pipeline/DeviceNode.hpp"
#include "depthai/pipeline/NodeGroup.hpp"
#include "depthai/pipeline/ThreadedNode.hpp"
#include "pybind11_common.hpp"
// depthai
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

dai::Pipeline* getImplicitPipeline();
void setImplicitPipeline(dai::Pipeline* pipeline);
void delImplicitPipeline();

struct NodeBindings {
    static void addToCallstack(std::deque<StackFunction>& callstack);
    static std::vector<std::pair<py::handle, std::function<std::shared_ptr<dai::Node>(dai::Pipeline&, py::object class_)>>> getNodeCreateMap();

   private:
    static void bind(pybind11::module& m, void* pCallstack);
};
