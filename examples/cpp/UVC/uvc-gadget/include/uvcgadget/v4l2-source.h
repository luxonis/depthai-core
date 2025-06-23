/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * V4L2 video source
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */
#ifndef __V4L2_VIDEO_SOURCE_H__
#define __V4L2_VIDEO_SOURCE_H__

#include "video-source.h"

struct events;
struct video_source;

struct video_source *v4l2_video_source_create(const char *devname);
void v4l2_video_source_init(struct video_source *src, struct events *events);

#endif /* __VIDEO_SOURCE_H__ */
