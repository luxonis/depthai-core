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
        // Starting the interpreter explicitly rather than using py::scoped_interpreter, we need to keep it running for the derived HostNode
        py::initialize_interpreter();
        try {
            const char* build_script_name = "build_pipeline";
            // This function builds a pipeline using a custom Python node. The custom node is the distinction between
            //  this script and the build_only_pipeline_from_cpp example
            const char* build_function_name = "build_with_py_nodes";
            // Note that pybind adds the directory that this executable is running from to PYTHONPATH for you;
            //  else you can add it manually via putenv
            const py::module_ build_script = py::module_::import(build_script_name);
            const py::function build_function = build_script.attr(build_function_name);
            // Because the depthai pybind11 bindings do not specify std::shared_ptr as a holder type,
            //  you must be explicit and unwrap the shared_ptr, or pybind11 will destroy the pipeline when finalizing the interpreter
            py::object pipeline_obj = py::cast(*pipeline, py::return_value_policy::reference);
            const py::object ret = build_function(pipeline_obj);
            queue = ret.cast<std::shared_ptr<dai::MessageQueue>>();
        } catch (const py::error_already_set& e) {
            std::cerr << "Py Error: " << e.what() << std::endl;
        }
        assert(queue != nullptr);
        // Start the pipeline
        pipeline->start();

        while(true) {
            auto videoIn = queue->get();
            std::cout << "back" << std::endl;
            if(videoIn == nullptr) continue;

            // cv::imshow("video", videoIn->getCvFrame());
            std::cout << "Got frame" << std::endl;

            if(cv::waitKey(1) == 'q') {
                break;
            }
        }
        device->close();
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        py::finalize_interpreter();
        return 1;
    }
    py::finalize_interpreter();

    return 0;
}
