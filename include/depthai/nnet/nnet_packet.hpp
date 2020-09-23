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
        const std::vector<TensorInfo>         &input_info,
        const std::vector<TensorInfo>         &tensors_info
    )
        : _tensors_raw_data(tensors_raw_data)
        , _input_info(input_info)
        , _tensors_info(tensors_info)
    {
        for (size_t i = 0; i < tensors_info.size(); ++i)
        {
            _tensor_name_to_index[ tensors_info[i].tensor_name ] = i;
        }

        if (_tensor_name_to_index.size() != tensors_info.size())
        {
            printf("There are duplication in tensor names!\n");
        }
    }

    std::shared_ptr<detection_out_t> getDetectedObjects()
    {
        std::shared_ptr<std::vector<unsigned char>> data = _tensors_raw_data->data;
        //copy-less return, wrapped in shared_ptr
        std::shared_ptr<detection_out_t> detections;
        detections = std::shared_ptr<detection_out_t>(data, reinterpret_cast<detection_out_t *>(data->data()));
        return detections;

        // std::shared_ptr<detection_out_t> a = std::shared_ptr<detection_out_t>(new detection_out_t);
        // memcpy(a.get(), data.get()->data(), data.get()->size());
        // return a;
    }

    int getTensorsSize()
    {
        return _tensors_info.size();
    }

    boost::optional<FrameMetadata> getMetadata(){
        // TODO
        return _tensors_raw_data->getMetadata();
    }

    const std::vector<TensorInfo> getInputLayersInfo()
    {
        return _input_info;
    }

    const std::vector<TensorInfo> getOutputLayersInfo()
    {
        return _tensors_info;
    }

protected: 
    std::string getTensorName(int index)
    {
        return _tensors_info[index].tensor_name;
    }


          std::shared_ptr<HostDataPacket> _tensors_raw_data;
    const std::vector<TensorInfo>         _input_info, _tensors_info;

    std::unordered_map<std::string, unsigned> _tensor_name_to_index;
};
