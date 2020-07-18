#pragma once

#include "depthai-shared/stream/stream_info.hpp"
#include "depthai-shared/general/data_subject.hpp"
#include "depthai-shared/stream/stream_data.hpp"
#include "depthai-shared/metadata/capture_metadata.hpp"

class HostCaptureCommand
    : public DataSubject<StreamInfo, StreamData>
{
public:
    HostCaptureCommand(const StreamInfo& streamToSendCommand);
    void capture();
    void afMode(CaptureMetadata::AutofocusMode mode);
    void afTrigger();
    void sendDisparityConfidenceThreshold(uint8_t confidence_thr);
    void sendCustomDeviceResetRequest(void);

private:
    StreamInfo stream;

    void sendCaptureMetadata(CaptureMetadata meta);



};
