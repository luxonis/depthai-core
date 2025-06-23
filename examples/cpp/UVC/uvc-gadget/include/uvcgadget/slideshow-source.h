/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Slideshow video source
 *
 * Copyright (C) 2018 Paul Elder
 *
 * Contact: Paul Elder <paul.elder@ideasonboard.com>
 */
#ifndef __SLIDESHOW_VIDEO_SOURCE_H__
#define __SLIDESHOW_VIDEO_SOURCE_H__

#include "video-source.h"

struct events;
struct video_source;

struct video_source *slideshow_video_source_create(const char *img_dir);
void slideshow_video_source_init(struct video_source *src, struct events *events);

#endif /* __SLIDESHOW_VIDEO_SOURCE_H__ */
