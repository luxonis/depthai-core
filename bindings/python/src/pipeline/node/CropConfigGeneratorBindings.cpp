#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/CropConfigGenerator.hpp"
#include "depthai/properties/CropConfigGeneratorProperties.hpp"

void bind_cropconfiggenerator(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;
    py::class_<CropConfigGeneratorProperties> cropConfigGeneratorProperties(m, "CropConfigGeneratorProperties", DOC(dai, CropConfigGeneratorProperties));
    auto cropConfigGenerator = ADD_NODE(CropConfigGenerator);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    cropConfigGenerator.def_readonly("inputDetections", &CropConfigGenerator::inputDetections, DOC(dai, node, CropConfigGenerator, inputDetections))
        .def_readonly("inputImage", &CropConfigGenerator::inputImage, DOC(dai, node, CropConfigGenerator, inputImage))
        .def_readonly("outConfig", &CropConfigGenerator::outConfig, DOC(dai, node, CropConfigGenerator, outConfig))
        .def_readonly("outImage", &CropConfigGenerator::outImage, DOC(dai, node, CropConfigGenerator, outImage))
        .def("runOnHost", &CropConfigGenerator::runOnHost, DOC(dai, node, CropConfigGenerator, runOnHost))
        .def("setRunOnHost", &CropConfigGenerator::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, CropConfigGenerator, setRunOnHost));

    daiNodeModule.attr("CropConfigGenerator").attr("Properties") = cropConfigGeneratorProperties;
}
