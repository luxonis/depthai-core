#pragma once

#include "depthai-shared/general/data_subject.hpp"
#include "depthai-shared/metadata/capture_metadata.hpp"
#include "depthai-shared/stream/stream_data.hpp"
#include "depthai-shared/stream/stream_info.hpp"

class HostCaptureCommand : public DataSubject<StreamInfo, StreamData> {
   public:
    HostCaptureCommand(const StreamInfo& streamToSendCommand);
    void capture();
    void afMode(CaptureMetadata::AutofocusMode mode);
    void afTrigger();

   private:
    StreamInfo stream;

    void sendCaptureMetadata(CaptureMetadata meta);
};
