#ifndef UVCGADGET_DEPTHAI_SOURCE_H
#define UVCGADGET_DEPTHAI_SOURCE_H

#include "video-source.h"

struct depthai_source {
	struct video_source src;

	unsigned int imgsize;
	void *imgdata;

	struct timer *timer;
	bool streaming;
};

struct events;
struct video_source;

struct video_source *depthai_video_source_create();
void depthai_video_source_init(struct video_source *src, struct events *events);

typedef void (*depthai_uvc_get_buffer_cb_t)(struct video_source *src, struct video_buffer *buf);

/* register the application’s buffer‐getter */
void depthai_uvc_register_get_buffer(depthai_uvc_get_buffer_cb_t cb);

#endif /* UVCGADGET_DEPTHAI_SOURCE_H */