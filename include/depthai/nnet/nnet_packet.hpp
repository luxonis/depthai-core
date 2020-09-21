#pragma once

#include <assert.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "tensor_info.hpp"
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

    detection_t getDetectedObject(int detected_nr)
    {
        unsigned char * data = _tensors_raw_data->data.data();
        detection_out_t * detections = (detection_out_t *)data;
        assert(detected_nr < detections->detection_count);
        return detections->detections[detected_nr];
    }

    int getTensorsSize()
    {
        return _tensors_info.size();
    }

    int getDetectionCount()
    {
        unsigned char * data = _tensors_raw_data->data.data();
        detection_out_t * detections = (detection_out_t *)data;
        return detections->detection_count;
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
