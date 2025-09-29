#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/YoloSegmentationParser.hpp"
#include "depthai/properties/YoloSegmentationParserProperties.hpp"

void bind_yolosegmentationparser(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<YoloSegmentationParserProperties> yoloSegmentationParserProperties(
        m, "YoloSegmentationParserProperties", DOC(dai, YoloSegmentationParserProperties));

    auto yoloSegmentationParser = ADD_NODE(YoloSegmentationParser);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////     /////////////////////////

    // Node
    yoloSegmentationParser.def_readonly("input", &YoloSegmentationParser::input, DOC(dai, node, YoloSegmentationParser, input))
        .def_readonly("inputDetections", &YoloSegmentationParser::inputDetections, DOC(dai, node, YoloSegmentationParser, inputDetections))
        .def_readonly("out", &YoloSegmentationParser::out, DOC(dai, node, YoloSegmentationParser, out))
        .def("setNNArchive",
             py::overload_cast<const NNArchive&>(&YoloSegmentationParser::setNNArchive),
             py::arg("nnArchive"),
             DOC(dai, node, YoloSegmentationParser, setNNArchive))
        .def("build", &YoloSegmentationParser::build, DOC(dai, node, YoloSegmentationParser, build));
    daiNodeModule.attr("YoloSegmentationParser").attr("Properties") = yoloSegmentationParserProperties;
}
