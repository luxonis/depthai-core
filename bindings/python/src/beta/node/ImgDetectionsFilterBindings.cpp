#include "depthai/beta/node/ImgDetectionsFilter.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_imgdetectionsfilter(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::beta::node;

    auto imgDetectionsFilter = ADD_BETA_NODE_DERIVED(ImgDetectionsFilter, dai::node::ThreadedHostNode);

    ///////////////////////////////////////////////////////////////////////
    // Callstack handling
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////

    imgDetectionsFilter.def_readonly("input", &ImgDetectionsFilter::input, DOC(dai, beta, node, ImgDetectionsFilter, input))
        .def_readonly("output", &ImgDetectionsFilter::output, DOC(dai, beta, node, ImgDetectionsFilter, output));
}
