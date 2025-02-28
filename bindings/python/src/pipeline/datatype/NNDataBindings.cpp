#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

// xtensor
#define FORCE_IMPORT_ARRAY
#include <xtensor-python/pyarray.hpp>

#include "fp16/fp16.h"

// #include "spdlog/spdlog.h"

void bind_nndata(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawNNData, RawBuffer, std::shared_ptr<RawNNData>> rawNnData(m, "RawNNData", DOC(dai, RawNNData));
    py::class_<TensorInfo> tensorInfo(m, "TensorInfo", DOC(dai, TensorInfo));
    py::enum_<TensorInfo::DataType> tensorInfoDataType(tensorInfo, "DataType");
    py::enum_<TensorInfo::StorageOrder> tensorInfoStorageOrder(tensorInfo, "StorageOrder");
    py::class_<NNData, Py<NNData>, Buffer, std::shared_ptr<NNData>> nnData(m, "NNData", DOC(dai, NNData));

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

    // Metadata / raw

    // rawNnData
    //     .def(py::init<>())
    //     .def_readwrite("tensors", &RawNNData::tensors)
    //     .def_readwrite("batchSize", &RawNNData::batchSize)
    //     .def_property("ts",
    //         [](const RawNNData& o){
    //             double ts = o.ts.sec + o.ts.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawNNData& o, double ts){
    //             o.ts.sec = ts;
    //             o.ts.nsec = (ts - o.ts.sec) * 1000000000.0;
    //         }
    //     )
    //     .def_property("tsDevice",
    //         [](const RawNNData& o){
    //             double ts = o.tsDevice.sec + o.tsDevice.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawNNData& o, double ts){
    //             o.tsDevice.sec = ts;
    //             o.tsDevice.nsec = (ts - o.tsDevice.sec) * 1000000000.0;
    //         }
    //     )
    //     .def_readwrite("sequenceNum", &RawNNData::sequenceNum)
    //     ;

    tensorInfo.def(py::init<>())
        .def("getTensorSize", &TensorInfo::getTensorSize)
        .def_readwrite("order", &TensorInfo::order)
        .def_readwrite("dataType", &TensorInfo::dataType)
        .def_readwrite("numDimensions", &TensorInfo::numDimensions)
        .def_readwrite("dims", &TensorInfo::dims)
        .def_readwrite("strides", &TensorInfo::strides)
        .def_readwrite("name", &TensorInfo::name)
        .def_readwrite("offset", &TensorInfo::offset)
        .def_readwrite("quantization", &TensorInfo::quantization)
        .def_readwrite("qpScale", &TensorInfo::qpScale)
        .def_readwrite("qpZp", &TensorInfo::qpZp);

    tensorInfoDataType.value("FP16", TensorInfo::DataType::FP16)
        .value("U8F", TensorInfo::DataType::U8F)
        .value("INT", TensorInfo::DataType::INT)
        .value("FP32", TensorInfo::DataType::FP32)
        .value("I8", TensorInfo::DataType::I8)
        .value("FP64", TensorInfo::DataType::FP64);

    tensorInfoStorageOrder.value("NHWC", TensorInfo::StorageOrder::NHWC)
        .value("NHCW", TensorInfo::StorageOrder::NHCW)
        .value("NCHW", TensorInfo::StorageOrder::NCHW)
        .value("HWC", TensorInfo::StorageOrder::HWC)
        .value("CHW", TensorInfo::StorageOrder::CHW)
        .value("WHC", TensorInfo::StorageOrder::WHC)
        .value("HCW", TensorInfo::StorageOrder::HCW)
        .value("WCH", TensorInfo::StorageOrder::WCH)
        .value("CWH", TensorInfo::StorageOrder::CWH)
        .value("NC", TensorInfo::StorageOrder::NC)
        .value("CN", TensorInfo::StorageOrder::CN)
        .value("C", TensorInfo::StorageOrder::C)
        .value("H", TensorInfo::StorageOrder::H)
        .value("W", TensorInfo::StorageOrder::W);

    // Message

    nnData.def(py::init<>(), DOC(dai, NNData, NNData))
        .def(py::init<size_t>(), DOC(dai, NNData, NNData, 2))
        .def("__repr__", &NNData::str)
        // // setters
        // .def("setLayer", [](NNData& obj, const std::string& name,
        // py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast>
        // data){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'addTensor()'
        //     instead", 1); std::vector<std::uint8_t> vec(data.data(),
        //     data.data() + data.size()); obj.setLayer(name, std::move(vec));
        // }, py::arg("name"), py::arg("data"), DOC(dai, NNData, setLayer))
        // .def("setLayer", [](NNData& obj, const std::string& name, const
        // std::vector<int>& data){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'addTensor()'
        //     instead", 1); return obj.setLayer(name, data);
        // }, py::arg("name"), py::arg("data"), DOC(dai, NNData, setLayer, 2))
        // .def("setLayer", [](NNData& obj, const std::string& name,
        // std::vector<float> data){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'addTensor()'
        //     instead", 1); return obj.setLayer(name, data);
        // }, py::arg("name"), py::arg("data"), DOC(dai, NNData, setLayer, 3))
        // .def("setLayer", [](NNData& obj, const std::string& name,
        // std::vector<double> data){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'addTensor()'
        //     instead", 1); return obj.setLayer(name, data);
        // }, py::arg("name"), py::arg("data"), DOC(dai, NNData, setLayer, 4))
        // .def("getLayer", [](NNData& obj, const std::string& name, TensorInfo&
        // tensor){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'getTensor()'
        //     instead", 1); return obj.getLayer(name, tensor);
        // }, py::arg("name"), py::arg("tensor"), DOC(dai, NNData, getLayer))
        .def("hasLayer", &NNData::hasLayer, py::arg("name"), DOC(dai, NNData, hasLayer))
        .def("getAllLayerNames", &NNData::getAllLayerNames, DOC(dai, NNData, getAllLayerNames))
        .def("getAllLayers", &NNData::getAllLayers, DOC(dai, NNData, getAllLayers))
        .def("getLayerDatatype", &NNData::getLayerDatatype, py::arg("name"), py::arg("datatype"), DOC(dai, NNData, getLayerDatatype))
        // .def("getLayerUInt8", [](NNData& obj, const std::string& name){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'getTensor()'
        //     instead", 1); return obj.getLayerUInt8(name);
        // }, py::arg("name"), DOC(dai, NNData, getLayerUInt8))
        // .def("getLayerFp16", [](NNData& obj, const std::string& name){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'getTensor()'
        //     instead", 1); return obj.getLayerFp16(name);
        // }, py::arg("name"), DOC(dai, NNData, getLayerFp16))
        // .def("getLayerInt32", [](NNData& obj, const std::string& name){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'getTensor()'
        //     instead", 1); return obj.getLayerInt32(name);
        // }, py::arg("name"), DOC(dai, NNData, getLayerInt32))
        // .def("getFirstLayerUInt8", [](NNData& obj){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'getTensor()'
        //     instead", 1); return obj.getFirstLayerUInt8();
        // }, DOC(dai, NNData, getFirstLayerUInt8))
        // .def("getFirstLayerFp16", [](NNData& obj){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'getTensor()'
        //     instead", 1); return obj.getFirstLayerFp16();
        // }, DOC(dai, NNData, getFirstLayerFp16))
        // .def("getFirstLayerInt32", [](NNData& obj){
        //     PyErr_WarnEx(PyExc_DeprecationWarning, "Use 'getTensor()'
        //     instead", 1); return obj.getFirstLayerInt32();
        // }, DOC(dai, NNData, getFirstLayerInt32))
        // TODO(Morato) - is this needed - doesn't get inherited from Buffer?
        .def("getTimestamp", &NNData::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &NNData::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &NNData::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        // .def("setTimestamp", &NNData::setTimestamp, DOC(dai, NNData, setTimestamp))
        // .def("setTimestampDevice", &NNData::setTimestampDevice, DOC(dai, NNData, setTimestampDevice))
        // .def("setSequenceNum", &NNData::setSequenceNum, DOC(dai, NNData, setSequenceNum))

        .def("addTensor",
             static_cast<NNData& (NNData::*)(const std::string&, const std::vector<int>&, TensorInfo::StorageOrder)>(&NNData::addTensor),
             py::arg("name"),
             py::arg("tensor"),
             py::arg("storageOrder"),
             DOC(dai, NNData, addTensor))
        .def("addTensor",
             static_cast<NNData& (NNData::*)(const std::string&, const std::vector<float>&, TensorInfo::StorageOrder)>(&NNData::addTensor),
             py::arg("name"),
             py::arg("tensor"),
             py::arg("storageOrder"),
             DOC(dai, NNData, addTensor))
        .def("addTensor",
             static_cast<NNData& (NNData::*)(const std::string&, const std::vector<double>&, TensorInfo::StorageOrder)>(&NNData::addTensor),
             py::arg("name"),
             py::arg("tensor"),
             py::arg("storageOrder"),
             DOC(dai, NNData, addTensor))

        .def("addTensor",
             static_cast<NNData& (NNData::*)(const std::string&, const xt::xarray<int>&, TensorInfo::StorageOrder)>(&NNData::addTensor),
             py::arg("name"),
             py::arg("tensor"),
             py::arg("storageOrder"),
             DOC(dai, NNData, addTensor, 2))
        .def("addTensor",
             static_cast<NNData& (NNData::*)(const std::string&, const xt::xarray<float>&, TensorInfo::StorageOrder)>(&NNData::addTensor),
             py::arg("name"),
             py::arg("tensor"),
             py::arg("storageOrder"),
             DOC(dai, NNData, addTensor, 2))
        .def("addTensor",
             static_cast<NNData& (NNData::*)(const std::string&, const xt::xarray<double>&, TensorInfo::StorageOrder)>(&NNData::addTensor),
             py::arg("name"),
             py::arg("tensor"),
             py::arg("storageOrder"),
             DOC(dai, NNData, addTensor, 2))

        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, py::object tensor_obj, dai::TensorInfo::DataType dataType) {
                auto tensor = py::array(tensor_obj);
                if(dataType == dai::TensorInfo::DataType::INT)
                    obj.addTensor<int>(name, tensor.cast<xt::xarray<int>>(), dai::TensorInfo::DataType::INT);
                else if(dataType == dai::TensorInfo::DataType::FP32)
                    obj.addTensor<float>(name, tensor.cast<xt::xarray<float>>(), dai::TensorInfo::DataType::FP32);
                else if(dataType == dai::TensorInfo::DataType::FP64)
                    obj.addTensor<double>(name, tensor.cast<xt::xarray<double>>(), dai::TensorInfo::DataType::FP64);
                else if(dataType == dai::TensorInfo::DataType::FP16)
                    obj.addTensor<double>(name, tensor.cast<xt::xarray<float>>(), dai::TensorInfo::DataType::FP16);
                else if(dataType == dai::TensorInfo::DataType::U8F)
                    obj.addTensor<uint8_t>(name, tensor.cast<xt::xarray<uint8_t>>(), dai::TensorInfo::DataType::U8F);
                else if(dataType == dai::TensorInfo::DataType::I8)
                    obj.addTensor<int8_t>(name, tensor.cast<xt::xarray<int8_t>>(), dai::TensorInfo::DataType::I8);
                else
                    throw std::runtime_error("Unsupported datatype");
            },
            py::arg("name"),
            py::arg("tensor"),
            py::arg("dataType"),
            DOC(dai, NNData, addTensor))

        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, py::object tensor_obj) {
                auto tensor = py::array(tensor_obj);
                auto dtype = tensor.dtype();
                if(dtype.is(py::dtype::of<float>()) || dtype.is(py::dtype::of<double>())) {
                    obj.addTensor<float>(name, tensor.cast<xt::xarray<float>>(), dai::TensorInfo::DataType::FP32);
                } else if(dtype.is(py::dtype::of<int>()) || dtype.is(py::dtype::of<int64_t>())) {
                    obj.addTensor<int>(name, tensor.cast<xt::xarray<int>>(), dai::TensorInfo::DataType::INT);
                } else if(dtype.is(py::dtype("float16"))) {
                    obj.addTensor<double>(name, tensor.cast<xt::xarray<float>>(), dai::TensorInfo::DataType::FP16);
                } else if(dtype.is(py::dtype::of<int8_t>())) {
                    obj.addTensor<int8_t>(name, tensor.cast<xt::xarray<int8_t>>(), dai::TensorInfo::DataType::I8);
                } else if(dtype.is(py::dtype::of<uint8_t>())) {
                    obj.addTensor<uint8_t>(name, tensor.cast<xt::xarray<uint8_t>>(), dai::TensorInfo::DataType::U8F);
                } else
                    throw std::runtime_error("Unsupported object type");
            },
            py::arg("name"),
            py::arg("tensor"),
            DOC(dai, NNData, addTensor, 2))

        .def(
            "getTensor",
            [](NNData& obj, const std::string& name, bool dequantize) -> py::object {
                const auto datatype = obj.getTensorDatatype(name);
                if((datatype == dai::TensorInfo::DataType::U8F && !dequantize) || (datatype == dai::TensorInfo::DataType::I8 && !dequantize)
                   || (datatype == dai::TensorInfo::DataType::INT && !dequantize)) {
                    // In case of dequantization, we should always return float
                    return py::cast(obj.getTensor<int>(name));
                } else if(datatype == dai::TensorInfo::DataType::FP64) {
                    return py::cast(obj.getTensor<double>(name, dequantize));
                } else {
                    return py::cast(obj.getTensor<float>(name, dequantize));
                }
            },
            py::arg("name"),
            py::arg("dequantize") = false,
            DOC(dai, NNData, getTensor))
        .def(
            "getTensor",
            [](NNData& obj, const std::string name, TensorInfo::StorageOrder order, bool dequantize) -> py::object {
                const auto datatype = obj.getTensorDatatype(name);
                if(datatype == dai::TensorInfo::DataType::U8F && !dequantize) {
                    // In case of dequantization, we should always return float
                    return py::cast(obj.getTensor<int>(name, order));
                } else {
                    return py::cast(obj.getTensor<float>(name, order, dequantize));
                }
            },
            py::arg("name"),
            py::arg("storageOrder"),
            py::arg("dequantize") = false,
            DOC(dai, NNData, getTensor))
        .def(
            "getFirstTensor",
            [](NNData& obj, bool dequantize) -> py::object {
                const auto datatype = obj.getFirstTensorDatatype();
                if(datatype == dai::TensorInfo::DataType::U8F && !dequantize) {
                    // In case of dequantization, we should always return float
                    return py::cast(obj.getFirstTensor<int>());
                } else {
                    return py::cast(obj.getFirstTensor<float>(dequantize));
                }
            },
            py::arg("dequantize") = false,
            DOC(dai, NNData, getFirstTensor))

        .def(
            "getFirstTensor",
            [](NNData& obj, TensorInfo::StorageOrder order, bool dequantize) -> py::object {
                const auto datatype = obj.getFirstTensorDatatype();
                if(datatype == dai::TensorInfo::DataType::U8F && !dequantize) {
                    // In case of dequantization, we should always return float
                    return py::cast(obj.getFirstTensor<int>(order));
                } else {
                    return py::cast(obj.getFirstTensor<float>(order, dequantize));
                }
            },
            py::arg("storageOrder"),
            py::arg("dequantize") = false,
            DOC(dai, NNData, getFirstTensor, 2))
        // .def("getTensor", static_cast<xt::xarray<double>(NNData::*)(const std::string&)>(&NNData::getTensor<double>), py::arg("name"), DOC(dai, NNData,
        // getTensor)) .def("getTensor", static_cast<xt::xarray<float>(NNData::*)(const std::string&)>(&NNData::getTensor<float>), py::arg("name"), DOC(dai,
        // NNData, getTensor, 2)) .def("getTensor", static_cast<xt::xarray<int>(NNData::*)(const std::string&)>(&NNData::getTensor<int>), py::arg("name"),
        // DOC(dai, NNData, getTensor, 3))
        .def("getTensorDatatype", &NNData::getTensorDatatype, py::arg("name"), DOC(dai, NNData, getTensorDatatype))
        .def("getTensorInfo", &NNData::getTensorInfo, py::arg("name"), DOC(dai, NNData, getTensorInfo))
        .def("getTransformation", [](NNData& msg) { return msg.transformation; })
        .def("setTransformation", [](NNData& msg, const std::optional<ImgTransformation>& transformation) { msg.transformation = transformation; });
}
