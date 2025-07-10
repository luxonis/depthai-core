#include "depthai/pipeline/datatype/Landmarks.hpp"

#include <pybind11/pybind11.h>

void bind_landmarks(pybind11::module& m, void* pCallstack) {
   using namespace dai;

   py::class_<Landmark> landmark(m, "Landmark", DOC(dai, Landmark));
   py::class_<Landmarks, Py<Landmarks>, Buffer, std::shared_ptr<Landmarks>> landmarks(m, "Landmarks", DOC(dai, Landmarks));

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
   ///////////////////////////////////////////////////////////////////////
   
   landmark.def(py::init<>())
      .def_readwrite("id", &Landmark::id, DOC(dai, Landmark, id))
      .def_readwrite("id", &Landmark::size, DOC(dai, Landmark, size))
      .def_readwrite("id", &Landmark::pose, DOC(dai, Landmark, pose));


   // Message
   landmarks.def(py::init<>())
       .def("__repr__", &Landmarks::str)
       .def_property(
           "aprilTags", [](Landmarks& det) { return &det.landmarks; }, [](Landmarks& det, std::vector<Landmark> val) { det.landmarks = val; })
       .def("getTimestamp", &Landmarks::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
       .def("getTimestampDevice", &Landmarks::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
       .def("getSequenceNum", &Landmarks::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
       // .def("setTimestamp", &AprilTags::setTimestamp, DOC(dai, Buffer, setTimestamp))
       // .def("setTimestampDevice", &AprilTags::setTimestampDevice, DOC(dai, Buffer, setTimestampDevice))
       // .def("setSequenceNum", &AprilTags::setSequenceNum, DOC(dai, Buffer, setSequenceNum))
       ;


}
