/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Video buffers
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */
#ifndef __VIDEO_BUFFERS_H__
#define __VIDEO_BUFFERS_H__

#include <stdbool.h>
#include <stddef.h>
#include <sys/time.h>

/*
 *
 * struct video_buffer - Video buffer information
 * @index: Zero-based buffer index, limited to the number of buffers minus one
 * @size: Size of the video memory, in bytes
 * @bytesused: Number of bytes used by video data, smaller or equal to @size
 * @timestamp: Time stamp at which the buffer has been captured
 * @error: True if an error occured while capturing video data for the buffer
 * @allocated: True if memory for the buffer has been allocated
 * @mem: Video data memory
 * @dmabuf: Video data dmabuf handle
 */
struct video_buffer
{
	unsigned int index;
	unsigned int size;
	unsigned int bytesused;
	struct timeval timestamp;
	bool error;
	void *mem;
	int dmabuf;
};

struct video_buffer_set
{
	struct video_buffer *buffers;
	unsigned int nbufs;
};

struct video_buffer_set *video_buffer_set_new(unsigned int nbufs);
void video_buffer_set_delete(struct video_buffer_set *buffers);

#endif /* __VIDEO_BUFFERS_H__ */
