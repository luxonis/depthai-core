#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/common/TensorInfo.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

#include <pybind11/cast.h>
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#define FORCE_IMPORT_ARRAY
#include <xtensor-python/pyarray.hpp>

#include "fp16/fp16.h"

void bind_nndata(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<TensorInfo> tensorInfo(m, "TensorInfo", DOC(dai, TensorInfo));
    py::enum_<TensorInfo::DataType> tensorInfoDataType(tensorInfo, "DataType");
    py::enum_<TensorInfo::StorageOrder> tensorInfoStorageOrder(tensorInfo, "StorageOrder");
    py::class_<NNData, Py<NNData>, Buffer, ProtoSerializable, py::smart_holder> nnData(m, "NNData", DOC(dai, NNData));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    tensorInfo.def(py::init<>())
        .def("validateStorageOrder", &dai::TensorInfo::validateStorageOrder, DOC(dai, TensorInfo, validateStorageOrder))
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
        .def_readwrite("qpZp", &TensorInfo::qpZp)
        .def("getDataTypeSize", &TensorInfo::getDataTypeSize)
        .def("getWidth", &TensorInfo::getWidth)
        .def("getHeight", &TensorInfo::getHeight)
        .def("getChannels", &TensorInfo::getChannels)
        .def("getChannelStride", &TensorInfo::getChannelStride)
        .def("getHeightStride", &TensorInfo::getHeightStride)
        .def("getWidthStride", &TensorInfo::getWidthStride);

    tensorInfoDataType.value("FP16", TensorInfo::DataType::FP16)
        .value("U8F", TensorInfo::DataType::U8F)
        .value("U16F", TensorInfo::DataType::U16F)
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

    nnData.def(py::init<>(), DOC(dai, NNData, NNData))
        .def(py::init<size_t>(), py::arg("size"), DOC(dai, NNData, NNData, 2))
        .def("getFirstTensorDatatype", &dai::NNData::getFirstTensorDatatype, DOC(dai, NNData, getFirstTensorDatatype))
        .def("__repr__", &NNData::str)
        .def("hasLayer", &NNData::hasLayer, py::arg("name"), DOC(dai, NNData, hasLayer))
        .def("getAllLayerNames", &NNData::getAllLayerNames, DOC(dai, NNData, getAllLayerNames))
        .def("getAllLayers", &NNData::getAllLayers, DOC(dai, NNData, getAllLayers))
        .def(
            "getLayerDatatype",
            [](const NNData& self, const std::string& name) -> std::optional<TensorInfo::DataType> {
                TensorInfo::DataType datatype;
                if(!self.getLayerDatatype(name, datatype)) return std::nullopt;
                return datatype;
            },
            py::arg("name"),
            DOC(dai, NNData, getLayerDatatype))
        .def("getLayerDatatype", &NNData::getLayerDatatype, py::arg("name"), py::arg("datatype"), DOC(dai, NNData, getLayerDatatype))
        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, const std::vector<int>& tensor, TensorInfo::StorageOrder order) -> NNData& {
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
                return obj.addTensor(name, tensor, order);
            },
            py::arg("name"),
            py::arg("tensor"),
            py::arg("storageOrder"),
            DOC(dai, NNData, addTensor))
        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, const std::vector<float>& tensor, TensorInfo::StorageOrder order) -> NNData& {
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
                return obj.addTensor(name, tensor, order);
            },
            py::arg("name"),
            py::arg("tensor"),
            py::arg("storageOrder"),
            DOC(dai, NNData, addTensor))
        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, const std::vector<double>& tensor, TensorInfo::StorageOrder order) -> NNData& {
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
                return obj.addTensor(name, tensor, order);
            },
            py::arg("name"),
            py::arg("tensor"),
            py::arg("storageOrder"),
            DOC(dai, NNData, addTensor))

        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, const xt::xarray<int>& tensor, TensorInfo::StorageOrder order) -> NNData& {
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
                return obj.addTensor(name, tensor, order);
            },
            py::arg("name"),
            py::arg("tensor"),
            py::arg("storageOrder"),
            DOC(dai, NNData, addTensor, 2))
        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, const xt::xarray<float>& tensor, TensorInfo::StorageOrder order) -> NNData& {
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
                return obj.addTensor(name, tensor, order);
            },
            py::arg("name"),
            py::arg("tensor"),
            py::arg("storageOrder"),
            DOC(dai, NNData, addTensor, 2))
        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, const xt::xarray<double>& tensor, TensorInfo::StorageOrder order) -> NNData& {
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
                return obj.addTensor(name, tensor, order);
            },
            py::arg("name"),
            py::arg("tensor"),
            py::arg("storageOrder"),
            DOC(dai, NNData, addTensor, 2))

        .def(
            "addTensor",
            [](NNData& obj, const std::string& name, py::object tensor_obj, dai::TensorInfo::DataType dataType) {
                auto tensor = py::array(tensor_obj);
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
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
                else if(dataType == dai::TensorInfo::DataType::U16F)
                    obj.addTensor<uint16_t>(name, tensor.cast<xt::xarray<uint16_t>>(), dai::TensorInfo::DataType::U16F);
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
                if(obj.data.use_count() > 1) {
                    const auto previous = obj.getData();
                    obj.setData(std::vector<std::uint8_t>(previous.begin(), previous.end()));
                }
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
                } else if(dtype.is(py::dtype::of<uint16_t>())) {
                    obj.addTensor<uint16_t>(name, tensor.cast<xt::xarray<uint16_t>>(), dai::TensorInfo::DataType::U16F);
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
                if((datatype == dai::TensorInfo::DataType::U8F || datatype == dai::TensorInfo::DataType::I8 || datatype == dai::TensorInfo::DataType::INT)
                   && !dequantize) {
                    return py::cast(obj.getTensor<int>(name, order));
                } else if(datatype == dai::TensorInfo::DataType::FP64) {
                    return py::cast(obj.getTensor<double>(name, order, dequantize));
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
                if((datatype == dai::TensorInfo::DataType::U8F || datatype == dai::TensorInfo::DataType::I8 || datatype == dai::TensorInfo::DataType::INT)
                   && !dequantize) {
                    return py::cast(obj.getFirstTensor<int>());
                } else if(datatype == dai::TensorInfo::DataType::FP64) {
                    return py::cast(obj.getFirstTensor<double>(dequantize));
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
                if((datatype == dai::TensorInfo::DataType::U8F || datatype == dai::TensorInfo::DataType::I8 || datatype == dai::TensorInfo::DataType::INT)
                   && !dequantize) {
                    return py::cast(obj.getFirstTensor<int>(order));
                } else if(datatype == dai::TensorInfo::DataType::FP64) {
                    return py::cast(obj.getFirstTensor<double>(order, dequantize));
                } else {
                    return py::cast(obj.getFirstTensor<float>(order, dequantize));
                }
            },
            py::arg("storageOrder"),
            py::arg("dequantize") = false,
            DOC(dai, NNData, getFirstTensor, 2))
        .def("getTensorDatatype", &NNData::getTensorDatatype, py::arg("name"), DOC(dai, NNData, getTensorDatatype))
        .def("getTensorInfo", &NNData::getTensorInfo, py::arg("name"), DOC(dai, NNData, getTensorInfo))
        .def("getTransformation", [](NNData& msg) { return msg.transformation; })
        .def(
            "setTransformation",
            [](NNData& msg, const std::optional<ImgTransformation>& transformation) { msg.transformation = transformation; },
            py::arg("transformation"),
            DOC(dai, ImgFrame, setTransformation));
}
