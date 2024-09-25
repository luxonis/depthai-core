#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/Keypoints.hpp"
//pybind
#include <pybind11/chrono.h>
#include <pybind11/stl.h>


void bind_keypoints(pybind11::module& m, void* pCallstack){

    using namespace dai;

    py::class_<Keypoints, Py<Keypoints>, Buffer, std::shared_ptr<Keypoints>> keypoints(m, "Keypoints", DOC(dai, Keypoints));
    py::class_<Keypoint> keypoint(m, "Keypoint", DOC(dai, Keypoint));

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

    // Single point struct
    keypoint
        .def(py::init<>())
        .def_readwrite("x", &Keypoint::x)
        .def_readwrite("y", &Keypoint::y)
        .def_readwrite("z", &Keypoint::z)
        .def_readwrite("confidence", &Keypoint::confidence)
        ;

    // Message
    keypoints
        .def(py::init<>())
        
        // getters
        .def("getTimestamp", &Keypoints::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &Keypoints::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &Keypoints::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getKeypoints", &Keypoints::getKeypoints, DOC(dai, Keypoints, getKeypoints))

        // setters
        .def("setTimestamp", &Keypoints::Buffer::setTimestamp, py::arg("timestamp"), DOC(dai, Buffer, setTimestamp))
        .def("setTimestampDevice", &Keypoints::Buffer::setTimestampDevice, DOC(dai, Buffer, setTimestampDevice))
        .def("setSequenceNum", &Keypoints::Buffer::setSequenceNum, DOC(dai, Buffer, setSequenceNum))
        // Binds only the overload that takes Keypoint objects
        .def("setKeypoints", py::overload_cast<const std::vector<Keypoint>&>(&Keypoints::setKeypoints), DOC(dai, Keypoints, setKeypoints))
        ;

}
