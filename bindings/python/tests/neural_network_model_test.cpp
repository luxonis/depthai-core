#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai_pybind11_tests.hpp"

TEST_SUBMODULE(neural_network_model, m) {
    py::module_::import("depthai");
    // Model loading only needs a standalone node, without a pipeline or device.
    m.def("create_neural_network", []() { return dai::node::NeuralNetwork::create(); });
}
