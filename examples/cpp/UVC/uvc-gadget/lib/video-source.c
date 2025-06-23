/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Abstract video source
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include "video-source.h"

void video_source_set_buffer_handler(struct video_source *src,
				     video_source_buffer_handler_t handler,
				     void *data)
{
	src->handler = handler;
	src->handler_data = data;
}

void video_source_destroy(struct video_source *src)
{
	if (src)
		src->ops->destroy(src);
}

int video_source_set_format(struct video_source *src,
			    struct v4l2_pix_format *fmt)
{
	return src->ops->set_format(src, fmt);
}

int video_source_set_frame_rate(struct video_source *src, unsigned int fps)
{
	return src->ops->set_frame_rate(src, fps);
}

int video_source_alloc_buffers(struct video_source *src, unsigned int nbufs)
{
	return src->ops->alloc_buffers(src, nbufs);
}

int video_source_export_buffers(struct video_source *src,
				struct video_buffer_set **buffers)
{
	return src->ops->export_buffers(src, buffers);
}

int video_source_import_buffers(struct video_source *src,
				struct video_buffer_set *buffers)
{
	return src->ops->import_buffers(src, buffers);
}

int video_source_free_buffers(struct video_source *src)
{
	return src->ops->free_buffers(src);
}

int video_source_stream_on(struct video_source *src)
{
	return src->ops->stream_on(src);
}

int video_source_stream_off(struct video_source *src)
{
	return src->ops->stream_off(src);
}

int video_source_queue_buffer(struct video_source *src,
			      struct video_buffer *buf)
{
	return src->ops->queue_buffer(src, buf);
}

void video_source_fill_buffer(struct video_source *src,
			      struct video_buffer *buf)
{
	src->ops->fill_buffer(src, buf);
}
