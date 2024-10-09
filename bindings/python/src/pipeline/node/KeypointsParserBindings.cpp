#include "Common.hpp"
#include "NodeBindings.hpp"

#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/properties/KeypointsParserProperties.hpp"
#include "depthai/pipeline/node/KeypointsParser.hpp"

void bind_keypointsparser(pybind11::module& m, void* pCallstack){

    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<KeypointsParserProperties> keypointsParserProperties(m, "keypointsParserProperties", DOC(dai, KeypointsParserProperties));
    auto keypointsParser = ADD_NODE(KeypointsParser);

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Properties
    keypointsParserProperties
        .def_readwrite("scaleFactor", &KeypointsParserProperties::scaleFactor)
        .def_readwrite("numKeypoints", &KeypointsParserProperties::numKeypoints)
        ;

    // Node
    keypointsParser
        .def_readonly("input", &KeypointsParser::input, DOC(dai, node, KeypointsParser, input))
        .def_readonly("out", &KeypointsParser::out, DOC(dai, node, KeypointsParser, out))

        .def("build", &KeypointsParser::build, DOC(dai, node, KeypointsParser, build))

        // getters
        .def("runOnHost", &KeypointsParser::runOnHost, DOC(dai, node, KeypointsParser, runOnHost))
        .def("getScaleFactor", &KeypointsParser::getScaleFactor, DOC(dai, node, KeypointsParser, getScaleFactor))
        .def("getNumKeypoints", &KeypointsParser::getNumKeypoints, DOC(dai, node, KeypointsParser, getNumKeypoints))

        // setters
        .def("setRunOnHost", &KeypointsParser::setRunOnHost, DOC(dai, node, KeypointsParser, setRunOnHost))
        .def("setScaleFactor", &KeypointsParser::setScaleFactor, DOC(dai, node, KeypointsParser, setScaleFactor))
        .def("setNumKeypoints", &KeypointsParser::setNumKeypoints, DOC(dai, node, KeypointsParser, setNumKeypoints))
        ;
    daiNodeModule.attr("KeypointsParser").attr("Properties") = keypointsParserProperties;

}
