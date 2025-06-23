/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * JPEG still image video source
 *
 * Copyright (C) 2018 Paul Elder
 *
 * Contact: Paul Elder <paul.elder@ideasonboard.com>
 */
#ifndef __JPG_VIDEO_SOURCE_H__
#define __JPG_VIDEO_SOURCE_H__

#include "video-source.h"

struct events;
struct video_source;

struct video_source *jpg_video_source_create(const char *img_path);
void jpg_video_source_init(struct video_source *src, struct events *events);

#endif /* __JPG_VIDEO_SOURCE_H__ */
