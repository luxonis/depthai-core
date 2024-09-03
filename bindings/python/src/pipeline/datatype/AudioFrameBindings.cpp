#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"
#include <unordered_map>
#include <memory>

// depthai
#include "depthai/pipeline/datatype/Buffer.hpp"

//pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_audioframe(pybind11::module& m, void* pCallstack){

    using namespace dai;

    // py::class_<RawBuffer, std::shared_ptr<RawBuffer>> rawBuffer(m, "RawBuffer", DOC(dai, RawBuffer));
    py::class_<AudioFrame, Py<AudioFrame>, Buffer, std::shared_ptr<AudioFrame>> audioFrame(m, "AudioFrame", DOC(dai, AudioFrame));

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

    // // Metadata / raw
    // rawBuffer
    //     .def(py::init<>())
    //     // .def_property("data", [](py::object &obj){
    //     //     dai::RawBuffer &a = obj.cast<dai::RawBuffer&>();
    //     //     return py::array_t<uint8_t>(a.data.size(), a.data.data(), obj);
    //     // }, [](py::object &obj, py::array_t<std::uint8_t, py::array::c_style> array){
    //     //     dai::RawBuffer &a = obj.cast<dai::RawBuffer&>();
    //     //     a.data = {array.data(), array.data() + array.size()};
    //     // })
    //     .def_property("ts",
    //         [](const RawBuffer& o){
    //             double ts = o.ts.sec + o.ts.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawBuffer& o, double ts){
    //             o.ts.sec = ts;
    //             o.ts.nsec = (ts - o.ts.sec) * 1000000000.0;
    //         }
    //     )
    //     .def_property("tsDevice",
    //         [](const RawBuffer& o){
    //             double ts = o.tsDevice.sec + o.tsDevice.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawBuffer& o, double ts){
    //             o.tsDevice.sec = ts;
    //             o.tsDevice.nsec = (ts - o.tsDevice.sec) * 1000000000.0;
    //         }
    //     )
    //     .def_readwrite("sequenceNum", &RawBuffer::sequenceNum)
    //     ;

    // Message
    audioFrame
        .def(py::init<>(), DOC(dai, AudioFrame, AudioFrame))
        .def(py::init<size_t>(), DOC(dai, AudioFrame, AudioFrame, 2))

        // obj is "Python" object, which we used then to bind the numpy arrays lifespan to
        .def("getData", [](py::object &obj){
            // creates numpy array (zero-copy) which holds correct information such as shape, ...
            dai::AudioFrame &a = obj.cast<dai::AudioFrame&>();
            return py::array_t<uint8_t>(a.getData().size(), a.getData().data(), obj);
        }, DOC(dai, AudioFrame, getData))
        .def("setData", py::overload_cast<const std::vector<std::uint8_t>&>(&AudioFrame::setData), DOC(dai, AudioFrame, setData))
        .def("setData", [](AudioFrame& audioFrame, py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast> array){
            audioFrame.setData({array.data(), array.data() + array.nbytes()});
        }, DOC(dai, AudioFrame, setData))
        .def("setData", [](AudioFrame& audioFrame, py::audioFrame data){
            std::string str = data.cast<std::string>();
            audioFrame.setData({str.data(), str.data() + str.size()});
        }, DOC(dai, AudioFrame, setData))
        .def("getTimestamp", &AudioFrame::getTimestamp, DOC(dai, AudioFrame, getTimestamp))
        .def("getFrames", &AudioFrame::getFrames, DOC(dai, AudioFrame, getFrames))
        .def("getBitrate", &AudioFrame::getBitrate, DOC(dai, AudioFrame, getBitrate))
        .def("getChannels", &AudioFrame::getChannels, DOC(dai, AudioFrame, getChannels))
        .def("getFormat", &AudioFrame::getFormat, DOC(dai, AudioFrame, getFormat))
        .def("setTimestamp", &AudioFrame::setTimestamp, DOC(dai, AudioFrame, setTimestamp))
        .def("setFrames", &AudioFrame::setFrames, DOC(dai, AudioFrame, setFrames))
        .def("setBitrate", &AudioFrame::setBitrate, DOC(dai, AudioFrame, setBitrate))
        .def("setChannels", &AudioFrame::setChannels, DOC(dai, AudioFrame, setChannels))
        .def("setFormat", &AudioFrame::setFormat, DOC(dai, AudioFrame, setFormat))
        ;


}
