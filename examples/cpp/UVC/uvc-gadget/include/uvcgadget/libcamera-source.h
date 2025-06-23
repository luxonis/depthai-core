/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * libcamera video source
 *
 * Copyright (C) 2022 Ideas on Board Oy.
 * Copyright (C) 2022 Kieran Bingham
 *
 * Contact: Kieran Bingham <kieran.bingham@ideasonboard.com>
 */
#ifndef __LIBCAMERA_SOURCE_H__
#define __LIBCAMERA_SOURCE_H__

#include "video-source.h"

struct events;
struct video_source;

#ifdef __cplusplus
extern "C" {
#endif

struct video_source *libcamera_source_create(const char *devname);
void libcamera_source_init(struct video_source *src, struct events *events);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* __LIBCAMERA_SOURCE_H__ */
