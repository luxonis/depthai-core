/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Video buffers
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include "video-buffers.h"

#include <stdlib.h>
#include <string.h>

struct video_buffer_set *video_buffer_set_new(unsigned int nbufs)
{
	struct video_buffer_set *buffers;

	buffers = malloc(sizeof *buffers);
	if (!buffers)
		return NULL;

	buffers->nbufs = nbufs;
	buffers->buffers = calloc(nbufs, sizeof *buffers->buffers);
	if (!buffers->buffers) {
		free(buffers);
		return NULL;
	}

	return buffers;
}

void video_buffer_set_delete(struct video_buffer_set *buffers)
{
	if (!buffers)
		return;

	free(buffers->buffers);
	free(buffers);
}
