/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * JPEG still image video source
 *
 * Copyright (C) 2018 Paul Elder
 *
 * Contact: Paul Elder <paul.elder@ideasonboard.com>
 */

#include <sys/types.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <linux/videodev2.h>

#include "events.h"
#include "timer.h"
#include "tools.h"
#include "v4l2.h"
#include "video-buffers.h"
#include "depthai-source.h"

#define to_depthai_source(s) container_of(s, struct depthai_source, src)

static depthai_uvc_get_buffer_cb_t s_get_buffer_cb = NULL;

static void depthai_source_destroy(struct video_source *s)
{
	struct depthai_source *src = to_depthai_source(s);

	if (src->imgdata)
		free(src->imgdata);

	timer_destroy(src->timer);

	free(src);
}

static int depthai_source_set_format(struct video_source *s __attribute__((unused)),
				  struct v4l2_pix_format *fmt)
{
	if (fmt->pixelformat != v4l2_fourcc('M', 'J', 'P', 'G')) {
		printf("depthai-source: unsupported fourcc\n");
		return -EINVAL;
	}

	return 0;
}

static int depthai_source_set_frame_rate(struct video_source *s, unsigned int fps)
{
	struct depthai_source *src = to_depthai_source(s);

	timer_set_fps(src->timer, fps);

	return 0;
}

static int depthai_source_free_buffers(struct video_source *s __attribute__((unused)))
{
	return 0;
}

static int depthai_source_stream_on(struct video_source *s)
{
	struct depthai_source *src = to_depthai_source(s);
	int ret;

	ret = timer_arm(src->timer);
	if (ret)
		return ret;

	src->streaming = true;
	return 0;
}

static int depthai_source_stream_off(struct video_source *s)
{
	struct depthai_source *src = to_depthai_source(s);
	int ret;

	/*
	 * No error check here, because we want to flag that streaming is over
	 * even if the timer is still running due to the failure.
	 */
	ret = timer_disarm(src->timer);
	src->streaming = false;

	return ret;
}

static void depthai_source_fill_buffer(struct video_source *s,
				   struct video_buffer *buf)
{
	struct depthai_source *src = to_depthai_source(s);
	
	// printf("depthai_source_fill_buffer(): Filling buffer %u\n", buf->index);
	
	if (!s_get_buffer_cb) {
		fprintf(stderr, "depthai_source_fill_buffer: No buffer getter registered\n");
		return;
	}
	
	s_get_buffer_cb(s, buf);
}


void depthai_uvc_register_get_buffer(depthai_uvc_get_buffer_cb_t cb)
{
	printf("%s:%d: Registering buffer getter callback\n", __func__, __LINE__);
	s_get_buffer_cb = cb;
}

static const struct video_source_ops depthai_source_ops = {
	.destroy = depthai_source_destroy,
	.set_format = depthai_source_set_format,
	.set_frame_rate = depthai_source_set_frame_rate,
	.alloc_buffers = NULL,
	.export_buffers = NULL,
	.free_buffers = depthai_source_free_buffers,
	.stream_on = depthai_source_stream_on,
	.stream_off = depthai_source_stream_off,
	.queue_buffer = NULL,
	.fill_buffer = depthai_source_fill_buffer,
};

struct video_source *depthai_video_source_create()
{
	struct depthai_source *src;

	src = malloc(sizeof *src);
	if (!src)
		return NULL;

	memset(src, 0, sizeof *src);
	src->src.ops = &depthai_source_ops;
	src->src.type = VIDEO_SOURCE_STATIC;

	src->timer = timer_new();

	return &src->src;
}

void depthai_video_source_init(struct video_source *s, struct events *events)
{
	struct depthai_source *src = to_depthai_source(s);

	src->src.events = events;
}
