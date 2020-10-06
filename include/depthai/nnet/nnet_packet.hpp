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
        const std::vector<dai::TensorInfo>         &tensors_info
    )
        : _tensors_raw_data(tensors_raw_data)
        , _input_info(input_info)
        , _tensors_info(tensors_info)
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

    std::unordered_map<std::string, unsigned> _tensor_name_to_index;
};
