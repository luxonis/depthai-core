#include "depthai/depthai.hpp"
#include "pybind11/embed.h"
#include "pybind11/pytypes.h"
#include "pybind11/stl.h"

extern "C" PyObject* pybind11_init_impl_depthai(void);

[[maybe_unused]] ::pybind11::detail::embedded_module pybind11_module_depthai("depthai", pybind11_init_impl_depthai);

int main() {
    namespace py = pybind11;
    try {
        auto device = std::make_shared<dai::Device>();
        auto pipeline = std::make_shared<dai::Pipeline>(device);
        std::shared_ptr<dai::MessageQueue> queue;
        {
            py::scoped_interpreter guard{};  // start interpreter, dies when out of scope
            try {
                // Just a simple script with one function that makes a single camera node
                const char* build_script_name = "build_pipeline";
                const char* build_function_name = "build_fn";
                // Note that pybind adds the directory that this executable is running from to PYTHONPATH for you;
                //  else you can add it manually
                const py::module_ build_script = py::module_::import(build_script_name);
                const py::function build_function = build_script.attr(build_function_name);
                // Because the depthai pybind11 bindings do not specify std::shared_ptr as a holding type,
                //  you must be explicit and unwrap the ptr, or pybind will destroy the pipeline
                py::object pipeline_obj = py::cast(*pipeline, py::return_value_policy::reference);
                const py::object ret = build_function(pipeline_obj);
                const auto cam = ret.cast<std::shared_ptr<dai::node::Camera>>();
                queue = cam->requestOutput(std::make_pair(640, 480))->createOutputQueue();
            } catch (const py::error_already_set& e) {
                std::cerr << "Error: " << e.what() << std::endl;
            }
        }
        // Start pipeline
        pipeline->start();

        while(true) {
            auto videoIn = queue->get<dai::ImgFrame>();
            if(videoIn == nullptr) continue;

            cv::imshow("video", videoIn->getCvFrame());

            if(cv::waitKey(1) == 'q') {
                break;
            }
        }
        device->close();
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
