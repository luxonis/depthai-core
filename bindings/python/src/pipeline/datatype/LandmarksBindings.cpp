#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/Landmarks.hpp"

//pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>



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
      .def_readwrite("size", &Landmark::size, DOC(dai, Landmark, size))
      .def_readwrite("translation", &Landmark::translation, DOC(dai, Landmark, translation))
      .def_readwrite("quaternion", &Landmark::quaternion, DOC(dai, Landmark, quaternion));


   // Message
   landmarks.def(py::init<>())
       .def("__repr__", &Landmarks::str)
       .def_property(
           "landmarks", [](Landmarks& det) { return &det.landmarks; }, [](Landmarks& det, std::vector<Landmark> val) { det.landmarks = val; })
       .def("getTimestamp", &Landmarks::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
       .def("getTimestampDevice", &Landmarks::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
       .def("getSequenceNum", &Landmarks::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
       // .def("setTimestamp", &AprilTags::setTimestamp, DOC(dai, Buffer, setTimestamp))
       // .def("setTimestampDevice", &AprilTags::setTimestampDevice, DOC(dai, Buffer, setTimestampDevice))
       // .def("setSequenceNum", &AprilTags::setSequenceNum, DOC(dai, Buffer, setSequenceNum))
       ;


}
