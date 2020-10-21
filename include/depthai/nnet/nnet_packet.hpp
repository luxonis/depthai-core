#pragma once

#include <assert.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "depthai-shared/tensor_info.hpp"
#include "depthai-shared/cnn_info.hpp"
#include "../host_data_packet.hpp"


class NNetPacket
{
public:
    NNetPacket(
              std::shared_ptr<HostDataPacket> &tensors_raw_data,
        const std::vector<dai::TensorInfo>         &input_info,
        const std::vector<dai::TensorInfo>         &tensors_info,
        const std::vector<nlohmann::json>          &NN_config
    )
        : _tensors_raw_data(tensors_raw_data)
        , _input_info(input_info)
        , _tensors_info(tensors_info)
        , _NN_config(NN_config)
    {
        for (size_t i = 0; i < tensors_info.size(); ++i)
        {
            _tensor_name_to_index[ tensors_info[i].name ] = i;
        }

        if (_tensor_name_to_index.size() != tensors_info.size())
        {
            printf("There are duplication in tensor names!\n");
        }
    }

    std::shared_ptr<dai::Detections> getDetectedObjects()
    {       
        if(_NN_config[0].contains("output_format"))
        {
            if(_NN_config[0]["output_format"] != std::string("detection"))
            {
                throw std::runtime_error("getDetectedObjects should be used only when [\"NN_config\"][\"output_format\"] is set to detection! https://docs.luxonis.com/api/#creating-blob-configuration-file");
            }
        }
        std::shared_ptr<std::vector<unsigned char>> data = _tensors_raw_data->data;
        //copy-less return, wrapped in shared_ptr
        std::shared_ptr<dai::Detections> detections;
        detections = std::shared_ptr<dai::Detections>(data, reinterpret_cast<dai::Detections *>(data->data()));
        return detections;
    }

    int getTensorsSize()
    {
        return _tensors_info.size();
    }

    boost::optional<FrameMetadata> getMetadata(){
        // TODO
        return _tensors_raw_data->getMetadata();
    }

    const std::vector<dai::TensorInfo> getInputLayersInfo()
    {
        return _input_info;
    }

    const std::vector<dai::TensorInfo> getOutputLayersInfo()
    {
        return _tensors_info;
    }

protected: 
    std::string getTensorName(int index)
    {
        return _tensors_info[index].name;
    }


          std::shared_ptr<HostDataPacket> _tensors_raw_data;
    const std::vector<dai::TensorInfo>         _input_info, _tensors_info;
    const std::vector<nlohmann::json>          _NN_config;
    std::unordered_map<std::string, unsigned> _tensor_name_to_index;
};
