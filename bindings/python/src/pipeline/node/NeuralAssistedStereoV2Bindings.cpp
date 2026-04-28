#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralAssistedStereoV2.hpp"

void bind_neuralassistedstereov2(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    py::class_<NeuralAssistedStereoV2Properties> properties(m, "NeuralAssistedStereoV2Properties");
    auto node = ADD_NODE(NeuralAssistedStereoV2);

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    properties
        .def_readwrite("initialConfig", &NeuralAssistedStereoV2Properties::initialConfig)
        .def_readwrite("numFramesPool", &NeuralAssistedStereoV2Properties::numFramesPool);

    node
        .def_property_readonly(
            "left",
            [](const NeuralAssistedStereoV2& n) { return &n.sync->inputs["left"]; },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "right",
            [](const NeuralAssistedStereoV2& n) { return &n.sync->inputs["right"]; },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "rectifiedLeft",
            [](const NeuralAssistedStereoV2& n) { return &n.rectification->output1; },
            py::return_value_policy::reference_internal)
        .def_property_readonly(
            "rectifiedRight",
            [](const NeuralAssistedStereoV2& n) { return &n.rectification->output2; },
            py::return_value_policy::reference_internal)
        .def("setRectification", &NeuralAssistedStereoV2::setRectification, py::arg("enable"))
        .def_readonly("inputConfig", &NeuralAssistedStereoV2::inputConfig)
        .def_readonly("neuralDisparity", &NeuralAssistedStereoV2::neuralDisparity,
                      py::doc("RAW16 neural disparity input (any resolution, upsampled to full-res internally)."))
        .def_readonly("initialConfig", &NeuralAssistedStereoV2::initialConfig)
        .def_readonly("disparity", &NeuralAssistedStereoV2::disparity)
        .def_readonly("depth", &NeuralAssistedStereoV2::depth)
        .def_readonly("debugZnccCurve", &NeuralAssistedStereoV2::debugZnccCurve,
                      py::doc("RAW32 payload: 3 uint32 header (d_min, d_max, n) + n float32 ZNCC costs."))
        .def("build", &NeuralAssistedStereoV2::build, py::arg("leftInput"), py::arg("rightInput"))
        .def_property_readonly(
            "sync", [](NeuralAssistedStereoV2& n) { return &(*n.sync); }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "messageDemux", [](NeuralAssistedStereoV2& n) { return &(*n.messageDemux); }, py::return_value_policy::reference_internal)
        .def_property_readonly(
            "rectification", [](NeuralAssistedStereoV2& n) { return &(*n.rectification); }, py::return_value_policy::reference_internal);

    daiNodeModule.attr("NeuralAssistedStereoV2").attr("Properties") = properties;
}
