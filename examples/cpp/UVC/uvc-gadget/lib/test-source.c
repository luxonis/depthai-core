/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Test video source
 *
 * Copyright (C) 2018 Paul Elder
 *
 * Contact: Paul Elder <paul.elder@ideasonboard.com>
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <linux/videodev2.h>

#include "events.h"
#include "test-source.h"
#include "tools.h"
#include "video-buffers.h"

#define BLACK   0x80108010
#define BLUE    0x7620f020
#define CYAN    0x10bc9abc
#define GREEN   0x2aad1aad
#define GREY    0x80b480b4
#define MAGENTA 0xe64ed64e
#define RED     0xf03f663f
#define WHITE   0x80eb80eb
#define YELLOW  0x8adb10db

struct test_source {
	struct video_source src;

	unsigned int width;
	unsigned int height;
	unsigned int pixelformat;
};

#define to_test_source(s) container_of(s, struct test_source, src)

static void test_source_destroy(struct video_source *s)
{
	struct test_source *src = to_test_source(s);

	free(src);
}

static int test_source_set_format(struct video_source *s,
				  struct v4l2_pix_format *fmt)
{
	struct test_source *src = to_test_source(s);

	src->width = fmt->width;
	src->height = fmt->height;
	src->pixelformat = fmt->pixelformat;

	if (src->pixelformat != v4l2_fourcc('Y', 'U', 'Y', 'V'))
		return -EINVAL;

	return 0;
}

static int test_source_set_frame_rate(struct video_source *s __attribute__((unused)),
				      unsigned int fps __attribute__((unused)))
{
	return 0;
}

static int test_source_free_buffers(struct video_source *s __attribute__((unused)))
{
	return 0;
}

static int test_source_stream_on(struct video_source *s __attribute__((unused)))
{
	return 0;
}

static int test_source_stream_off(struct video_source *s __attribute__((unused)))
{
	return 0;
}

static void test_source_fill_buffer(struct video_source *s,
				    struct video_buffer *buf)
{
	struct test_source *src = to_test_source(s);
	unsigned int bpl;
	unsigned int i, j;
	void *mem = buf->mem;

	bpl = src->width * 2;
	for (i = 0; i < src->height; ++i) {
		for (j = 0; j < bpl; j += 4) {
			if (j < bpl * 1 / 8)
				*(unsigned int *)(mem + i*bpl + j) = WHITE;
			else if (j < bpl * 2 / 8)
				*(unsigned int *)(mem + i*bpl + j) = YELLOW;
			else if (j < bpl * 3 / 8)
				*(unsigned int *)(mem + i*bpl + j) = CYAN;
			else if (j < bpl * 4 / 8)
				*(unsigned int *)(mem + i*bpl + j) = GREEN;
			else if (j < bpl * 5 / 8)
				*(unsigned int *)(mem + i*bpl + j) = MAGENTA;
			else if (j < bpl * 6 / 8)
				*(unsigned int *)(mem + i*bpl + j) = RED;
			else if (j < bpl * 7 / 8)
				*(unsigned int *)(mem + i*bpl + j) = BLUE;
			else
				*(unsigned int *)(mem + i*bpl + j) = BLACK;
		}
	}

	buf->bytesused = bpl * src->height;
}

static const struct video_source_ops test_source_ops = {
	.destroy = test_source_destroy,
	.set_format = test_source_set_format,
	.set_frame_rate = test_source_set_frame_rate,
	.free_buffers = test_source_free_buffers,
	.stream_on = test_source_stream_on,
	.stream_off = test_source_stream_off,
	.queue_buffer = NULL,
	.fill_buffer = test_source_fill_buffer,
};

struct video_source *test_video_source_create()
{
	struct test_source *src;

	src = malloc(sizeof *src);
	if (!src)
		return NULL;

	memset(src, 0, sizeof *src);
	src->src.ops = &test_source_ops;
	src->src.type = VIDEO_SOURCE_STATIC;

	return &src->src;
}

void test_video_source_init(struct video_source *s, struct events *events)
{
	struct test_source *src = to_test_source(s);

	src->src.events = events;
}
