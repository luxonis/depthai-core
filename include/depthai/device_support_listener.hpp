#pragma once

#include "depthai-shared/general/data_observer.hpp"
#include "depthai-shared/stream/stream_data.hpp"
#include "depthai-shared/stream/stream_info.hpp"

class DeviceSupportListener : public DataObserver<StreamInfo, StreamData> {
   public:
   protected:
    // class DataObserver
    virtual void onNewData(const StreamInfo& data_info, const StreamData& data) final;
};
