#pragma once

#include <unordered_map>
#include <vector>

#include "host_pipeline.hpp"
#include "depthai-shared/tensor_info.hpp"
#include "../nnet/nnet_packet.hpp"


class CNNHostPipeline
    : public HostPipeline
{
private:

    const std::string               cnn_result_stream_name = "metaout";

    const std::vector<dai::TensorInfo>   _input_tensors_info;
    const std::vector<dai::TensorInfo>   _output_tensors_info;
    const std::vector<nlohmann::json>    _NN_config;

    std::list<std::shared_ptr<NNetPacket>> getConsumedNNetPackets();

public:
    CNNHostPipeline(const std::vector<dai::TensorInfo>& input_tensors_info, const std::vector<dai::TensorInfo>& output_tensors_info, const std::vector<nlohmann::json>& NN_config)
        : _input_tensors_info(input_tensors_info)
        , _output_tensors_info(output_tensors_info)
        , _NN_config(NN_config)
    {}
    virtual ~CNNHostPipeline() {}


    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>
    >
    getAvailableNNetAndDataPackets(bool blocking = false);


};
