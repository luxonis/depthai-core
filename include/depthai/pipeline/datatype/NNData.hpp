#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/NNData.hpp"

namespace dai {
namespace tmp {

    struct NNInput : protected dai::NNData {
        // store the data

        // uint8_t
        std::unordered_map<std::string, std::vector<std::uint8_t>> u8Data;
        // FP16
        std::unordered_map<std::string, std::vector<std::uint16_t>> fp16Data;

        // Expose
        void setLayer(const std::string& name, std::vector<std::uint8_t> data) {
            u8Data[name] = std::move(data);
        }
        void setLayer(const std::string& name, const std::vector<int>& data) {
            u8Data[name] = std::vector<std::uint8_t>(data.size());
            for(unsigned i = 0; i < data.size(); i++) {
                u8Data[name][i] = static_cast<std::uint8_t>(data[i]);
            }
        }

        void setLayer(const std::string& name, std::vector<float> data) {
            fp16Data[name] = std::vector<std::uint8_t>(data.size());
            for(unsigned i = 0; i < data.size(); i++) {
                // fp16Data[name][i] = fp16_cast(data[i]);
            }
        }
        void setLayer(const std::string& name, std::vector<double> data) {
            fp16Data[name] = std::vector<std::uint8_t>(data.size());
            for(unsigned i = 0; i < data.size(); i++) {
                // fp16Data[name][i] = fp16_cast(data[i]);
            }
        }

    }

    struct NNOutput : protected dai::NNData {
        // add convinience functions
    }

}  // namespace tmp

}  // namespace dai
