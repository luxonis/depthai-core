#pragma once
// This file is created as temporary solution for calculation
// of distance data from disparity

// Std
#include <vector>

// Shared
#include "depthai-shared/general/data_observer.hpp"
#include "depthai-shared/general/data_subject.hpp"
#include "depthai-shared/stream/stream_info.hpp"
#include "depthai-shared/stream/stream_data.hpp"


class DisparityStreamPostProcessor
    : public DataSubject<StreamInfo, StreamData>
    , public DataObserver<StreamInfo, StreamData>
{
public:
    DisparityStreamPostProcessor(bool produce_d_color);

protected:
    // class DataObserver
    virtual void onNewData(const StreamInfo &data_info, const StreamData &data);


private:
    const std::string c_stream_in        = "disparity";
    const std::string c_stream_out_color = "disparity_color";

    const bool _produce_depth_color = false;

    void prepareDepthColorAndNotifyObservers(const StreamInfo &data_info, const StreamData &data);
};
