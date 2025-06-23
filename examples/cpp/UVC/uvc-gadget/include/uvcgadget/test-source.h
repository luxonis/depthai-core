/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Test video source
 *
 * Copyright (C) 2018 Paul Elder
 *
 * Contact: Paul Elder <paul.elder@ideasonboard.com>
 */
#ifndef __TEST_VIDEO_SOURCE_H__
#define __TEST_VIDEO_SOURCE_H__

#include "video-source.h"

struct events;
struct video_source;

struct video_source *test_video_source_create(void);
void test_video_source_init(struct video_source *src, struct events *events);

#endif /* __TEST_VIDEO_SOURCE_H__ */
