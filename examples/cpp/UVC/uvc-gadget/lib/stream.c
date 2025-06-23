/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * UVC stream handling
 *
 * Copyright (C) 2010-2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "events.h"
#include "stream.h"
#include "uvc.h"
#include "v4l2.h"
#include "video-buffers.h"
#include "video-source.h"

/*
 * struct uvc_stream - Representation of a UVC stream
 * @src: video source
 * @uvc: UVC V4L2 output device
 * @events: struct events containing event information
 */
struct uvc_stream
{
	struct video_source *src;
	struct uvc_device *uvc;

	struct events *events;
};

/* ---------------------------------------------------------------------------
 * Video streaming
 */

static void uvc_stream_source_process(void *d,
				      struct video_source *src __attribute__((unused)),
				      struct video_buffer *buffer)
{
	struct uvc_stream *stream = d;
	struct v4l2_device *sink = uvc_v4l2_device(stream->uvc);

	v4l2_queue_buffer(sink, buffer);
}

static void uvc_stream_uvc_process(void *d)
{
	struct uvc_stream *stream = d;
	struct v4l2_device *sink = uvc_v4l2_device(stream->uvc);
	struct video_buffer buf;
	int ret;

	ret = v4l2_dequeue_buffer(sink, &buf);
	if (ret < 0)
		return;

	video_source_queue_buffer(stream->src, &buf);
}

static void uvc_stream_uvc_process_no_buf(void *d)
{
	struct uvc_stream *stream = d;
	struct v4l2_device *sink = uvc_v4l2_device(stream->uvc);
	struct video_buffer buf;
	int ret;

	ret = v4l2_dequeue_buffer(sink, &buf);
	if (ret < 0)
		return;

	video_source_fill_buffer(stream->src, &buf);

	v4l2_queue_buffer(sink, &buf);
}


static int uvc_stream_start_alloc(struct uvc_stream *stream)
{
	struct v4l2_device *sink = uvc_v4l2_device(stream->uvc);
	struct video_buffer_set *buffers = NULL;
	int ret;

	/* Allocate and export the buffers on the source. */
	ret = video_source_alloc_buffers(stream->src, 4);
	if (ret < 0) {
		printf("Failed to allocate source buffers: %s (%d)\n",
		       strerror(-ret), -ret);
		return ret;
	}

	ret = video_source_export_buffers(stream->src, &buffers);
	if (ret < 0) {
		printf("Failed to export buffers on source: %s (%d)\n",
		       strerror(-ret), -ret);
		goto error_free_source;
	}

	/* Allocate and import the buffers on the sink. */
	ret = v4l2_alloc_buffers(sink, V4L2_MEMORY_DMABUF, buffers->nbufs);
	if (ret < 0) {
		printf("Failed to allocate sink buffers: %s (%d)\n",
		       strerror(-ret), -ret);
		goto error_free_source;
	}

	ret = v4l2_import_buffers(sink, buffers);
	if (ret < 0) {
		printf("Failed to import buffers on sink: %s (%d)\n",
		       strerror(-ret), -ret);
		goto error_free_sink;
	}

	/* Start the source and sink. */
	video_source_stream_on(stream->src);
	v4l2_stream_on(sink);

	events_watch_fd(stream->events, sink->fd, EVENT_WRITE,
			uvc_stream_uvc_process, stream);

	return 0;

error_free_sink:
	v4l2_free_buffers(sink);
error_free_source:
	video_source_free_buffers(stream->src);
	if (buffers)
		video_buffer_set_delete(buffers);
	return ret;
}

static int uvc_stream_start_no_alloc(struct uvc_stream *stream)
{
	struct v4l2_device *sink = uvc_v4l2_device(stream->uvc);
	int ret;
	unsigned int i;

	/* Allocate buffers on the sink. */
	ret = v4l2_alloc_buffers(sink, V4L2_MEMORY_MMAP, 4);
	if (ret < 0) {
		printf("Failed to allocate sink buffers: %s (%d)\n",
		       strerror(-ret), -ret);
		return ret;
	}

	/* mmap buffers. */
	ret = v4l2_mmap_buffers(sink);
	if (ret < 0) {
		printf("Failed to query sink buffers: %s (%d)\n",
				strerror(-ret), -ret);
		return ret;
	}

	/* Queue buffers to sink. */
	for (i = 0; i < sink->buffers.nbufs; ++i) {
		struct video_buffer buf = {
			.index = i,
			.size = sink->buffers.buffers[i].size,
			.mem = sink->buffers.buffers[i].mem,
		};

		video_source_fill_buffer(stream->src, &buf);
		ret = v4l2_queue_buffer(sink, &buf);
		if (ret < 0)
			return ret;
	}

	/* Start the source and sink. */
	video_source_stream_on(stream->src);
	ret = v4l2_stream_on(sink);
	if (ret < 0)
		return ret;

	events_watch_fd(stream->events, sink->fd, EVENT_WRITE,
			uvc_stream_uvc_process_no_buf, stream);

	return 0;
}

static int uvc_stream_start_encoded(struct uvc_stream *stream)
{
	struct v4l2_device *sink = uvc_v4l2_device(stream->uvc);
	int ret;

	/* Allocate the buffers on the source. */
	ret = video_source_alloc_buffers(stream->src, 4);
	if (ret < 0) {
		printf("Failed to allocate source buffers: %s (%d)\n",
		       strerror(-ret), -ret);
		return ret;
	}

	/* Allocate buffers on the sink. */
	ret = v4l2_alloc_buffers(sink, V4L2_MEMORY_MMAP, 4);
	if (ret < 0) {
		printf("Failed to allocate sink buffers: %s (%d)\n",
		       strerror(-ret), -ret);
		goto error_free_source;
	}

	/* mmap buffers. */
	ret = v4l2_mmap_buffers(sink);
	if (ret < 0) {
		printf("Failed to query sink buffers: %s (%d)\n",
				strerror(-ret), -ret);
		goto error_free_sink;
	}

	/* Import the sink's buffers to the source */
	ret = video_source_import_buffers(stream->src, &sink->buffers);
	if (ret) {
		printf("Failed to import sink buffers: %s (%d)\n",
		       strerror(ret), ret);
		goto error_free_sink;
	}

	/* Start the source and sink. */
	video_source_stream_on(stream->src);
	v4l2_stream_on(sink);

	events_watch_fd(stream->events, sink->fd, EVENT_WRITE,
			uvc_stream_uvc_process, stream);

	return 0;

error_free_sink:
	v4l2_free_buffers(sink);
error_free_source:
	video_source_free_buffers(stream->src);

	return ret;
}

static int uvc_stream_start(struct uvc_stream *stream)
{
	printf("Starting video stream.\n");

	switch (stream->src->type) {
	case VIDEO_SOURCE_DMABUF:
		video_source_set_buffer_handler(stream->src, uvc_stream_source_process,
						stream);
		return uvc_stream_start_alloc(stream);
	case VIDEO_SOURCE_STATIC:
		return uvc_stream_start_no_alloc(stream);
	case VIDEO_SOURCE_ENCODED:
		video_source_set_buffer_handler(stream->src, uvc_stream_source_process,
						stream);
		return uvc_stream_start_encoded(stream);
	default:
		fprintf(stderr, "invalid video source type\n");
		break;
	}

	return -EINVAL;
}

static int uvc_stream_stop(struct uvc_stream *stream)
{
	struct v4l2_device *sink = uvc_v4l2_device(stream->uvc);

	printf("Stopping video stream.\n");

	events_unwatch_fd(stream->events, sink->fd, EVENT_WRITE);

	v4l2_stream_off(sink);
	video_source_stream_off(stream->src);

	v4l2_free_buffers(sink);
	video_source_free_buffers(stream->src);

	return 0;
}

void uvc_stream_enable(struct uvc_stream *stream, int enable)
{
	if (enable)
		uvc_stream_start(stream);
	else
		uvc_stream_stop(stream);
}

int uvc_stream_set_format(struct uvc_stream *stream,
			  const struct v4l2_pix_format *format)
{
	struct v4l2_pix_format fmt = *format;
	int ret;

	printf("Setting format to 0x%08x %ux%u\n",
		format->pixelformat, format->width, format->height);

	ret = uvc_set_format(stream->uvc, &fmt);
	if (ret < 0)
		return ret;

	return video_source_set_format(stream->src, &fmt);
}

int uvc_stream_set_frame_rate(struct uvc_stream *stream, unsigned int fps)
{
	printf("=== Setting frame rate to %u fps\n", fps);
	return video_source_set_frame_rate(stream->src, fps);
}

/* ---------------------------------------------------------------------------
 * Stream handling
 */

struct uvc_stream *uvc_stream_new(const char *uvc_device)
{
	struct uvc_stream *stream;

	stream = malloc(sizeof(*stream));
	if (stream == NULL)
		return NULL;

	memset(stream, 0, sizeof(*stream));

	stream->uvc = uvc_open(uvc_device, stream);
	if (stream->uvc == NULL)
		goto error;

	return stream;

error:
	free(stream);
	return NULL;
}

void uvc_stream_delete(struct uvc_stream *stream)
{
	if (stream == NULL)
		return;

	uvc_close(stream->uvc);

	free(stream);
}

void uvc_stream_init_uvc(struct uvc_stream *stream,
			 struct uvc_function_config *fc)
{
	uvc_set_config(stream->uvc, fc);
	uvc_events_init(stream->uvc, stream->events);
}

void uvc_stream_set_event_handler(struct uvc_stream *stream,
				  struct events *events)
{
	stream->events = events;
}

void uvc_stream_set_video_source(struct uvc_stream *stream,
				 struct video_source *src)
{
	stream->src = src;
}
