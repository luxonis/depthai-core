#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/FocusedDepthRoi.hpp"

void bind_focused_depth_roi(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace pybind11::literals;

    py::class_<FocusedDepthRoi, Buffer, std::shared_ptr<FocusedDepthRoi>> roiMsg(m, "FocusedDepthRoi");

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    py::enum_<FocusedDepthRoi::ReferenceFrame>(roiMsg, "ReferenceFrame")
        .value("LEFT_RECTIFIED", FocusedDepthRoi::ReferenceFrame::LEFT_RECTIFIED);

    roiMsg.def(py::init<>())
        .def_readwrite("roi", &FocusedDepthRoi::roi)
        .def_readwrite("normalizedCoords", &FocusedDepthRoi::normalizedCoords)
        .def_readwrite("referenceFrame", &FocusedDepthRoi::referenceFrame)
        .def_readwrite("sequenceHint", &FocusedDepthRoi::sequenceHint)
        .def("getDatatype", &FocusedDepthRoi::getDatatype);
}
