#pragma once

extern "C" {
#include "uvcgadget/video-source.h"
#include "uvcgadget/depthai-source.h"
}

int depthai_uvc_get_buffer(struct video_source *s, struct video_buffer *buf);
