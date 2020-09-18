#pragma once

#include <assert.h>

#include <string>
#include <unordered_map>
#include <map>
#include "depthai-shared/cnn_info.hpp"

#include "half.hpp"

using float16 = half_float::half;


const std::map<TensorDataType, unsigned int> c_type_size = {
    {TensorDataType::_fp16,     sizeof(float16)},
    {TensorDataType::_u8f,      sizeof(std::uint8_t)},
    {TensorDataType::_int,      sizeof(std::int32_t)},
    {TensorDataType::_fp32,     sizeof(float)},
    {TensorDataType::_i8,       sizeof(std::int8_t)},
};


const std::map<TensorDataType, std::string> type_to_string = {
    {TensorDataType::_fp16,     "float16"},
    {TensorDataType::_u8f,      "uint8"},
    {TensorDataType::_int,      "int32"},
    {TensorDataType::_fp32,     "float32"},
    {TensorDataType::_i8,       "int8"},
};

unsigned int size_of_type   (const TensorDataType& type);
// std::string type_to_npy_format_descriptor(const TensorDataType& type);
