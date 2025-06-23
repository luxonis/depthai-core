/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Slideshow video source
 *
 * Copyright (C) 2018 Paul Elder
 *
 * Contact: Paul Elder <paul.elder@ideasonboard.com>
 */

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <linux/limits.h>
#include <linux/videodev2.h>

#include <sys/types.h>
#include <sys/stat.h>

#include "events.h"
#include "list.h"
#include "slideshow-source.h"
#include "timer.h"
#include "tools.h"
#include "video-buffers.h"

struct slide {
	struct list_entry list;
	unsigned int imgsize;
	void *imgdata;
};

struct slideshow_source {
	struct video_source src;

	char img_dir[NAME_MAX];

	struct slide *cur_slide;
	struct list_entry slides;

	struct timer *timer;
	bool streaming;
};

#define to_slideshow_source(s) container_of(s, struct slideshow_source, src)

static void slideshow_source_destroy(struct video_source *s)
{
	struct slideshow_source *src = to_slideshow_source(s);
	struct slide *slide, *next;

	list_for_each_entry_safe(slide, next, &src->slides, list) {
		list_remove(&slide->list);
		free(slide->imgdata);
		free(slide);
	}
	timer_destroy(src->timer);
	free(src);
}

char *v4l2_fourcc2s(__u32 fourcc, char *buf)
{
	buf[0] = fourcc & 0x7f;
	buf[1] = (fourcc >> 8) & 0x7f;
	buf[2] = (fourcc >> 16) & 0x7f;
	buf[3] = (fourcc >> 24) & 0x7f;
	if (fourcc & (1 << 31)) {
		buf[4] = '-';
		buf[5] = 'B';
		buf[6] = 'E';
		buf[7] = '\0';
	} else {
		buf[4] = '\0';
	}
	return buf;
}

/*
 * slideshow_source_set_format - set the V4L2 format
 *
 * For this source, we require images stored in a directory structure with nodes
 * for each format and framesize, for example:
 *
 * slideshow +
 *	     |
 *	     + MJPG +
 *	     |      |
 *	     |      + 1280x720  +
 *	     |      |           |
 *	     |      |           + 01.jpg
 *	     |      |           |
 *	     |      |           + 02.jpg
 *	     |      |           |
 *	     |      |           + 03.jpg
 *	     |      |
 *	     |      + 1920x1080
 *	     |
 *	     + YUYV +
 *		    |
 *		    + 1280x720
 *		    |
 *		    + 1920x1080
 *
 * The root directory will be passed as an argument to slideshow_source_create()
 * and so is not fixed, but the second level directories must be named with the
 * fourcc of the format the images within represent, and the third level's node
 * names must be in the format "<width>x<height>".
 */
static int slideshow_source_set_format(struct video_source *s,
				       struct v4l2_pix_format *fmt)
{
	struct slideshow_source *src = to_slideshow_source(s);
	char dirname[PATH_MAX];
	struct slide *slide, *next;
	struct dirent *file;
	char fourcc_buf[8];
	int fd = -1;
	char *cwd;
	DIR *dir;
	int ret;

	/*
	 * If the format is changed, we need to clear the existing list of
	 * slides before adding new ones.
	 */
	list_for_each_entry_safe(slide, next, &src->slides, list) {
		list_remove(&slide->list);
		free(slide->imgdata);
		free(slide);
	}

	ret = snprintf(dirname, sizeof(dirname), "%s/%s/%ux%u", src->img_dir,
		       v4l2_fourcc2s(fmt->pixelformat, fourcc_buf),
		       fmt->width, fmt->height);
	if (ret < 0) {
		fprintf(stderr, "failed to store directory name: %s (%d)\n",
			strerror(ret), ret);
		ret = errno;
		goto err_dummy_slide;
	}

	dir = opendir(dirname);
	if (!dir) {
		fprintf(stderr, "unable to find directory %s\n", dirname);
		ret = -ENOENT;
		goto err_dummy_slide;
	}

	cwd = getcwd(NULL, 0);
	if (!cwd) {
		fprintf(stderr, "unable to allocate memory for cwd name\n");
		ret = -ENOMEM;
		goto err_dummy_slide;
	}

	ret = chdir(dirname);
	if (ret) {
		fprintf(stderr, "unable to cd to directory '%s': %s (%d)\n",
			dirname, strerror(ret), ret);
		goto err_close_dir;
	}

	while ((file = readdir(dir))) {
		if (!strcmp(file->d_name, ".") ||
		    !strcmp(file->d_name, ".."))
			continue;

		fd = open(file->d_name, O_RDONLY);
		if (fd == -1) {
			fprintf(stderr, "Unable to open file '%s/%s'\n", dirname,
				file->d_name);
			ret = errno;
			goto err_unwind;
		}

		slide = malloc(sizeof(*slide));
		if (!slide) {
			fprintf(stderr, "failed to allocate memory for slide\n");
			ret = -ENOMEM;
			goto err_close_fd;
		}

		slide->imgsize = lseek(fd, 0, SEEK_END);
		lseek(fd, 0, SEEK_SET);

		slide->imgdata = malloc(slide->imgsize);
		if (!slide->imgdata) {
			fprintf(stderr, "failed to allocate memory for image\n");
			ret = -ENOMEM;
			goto err_free_slide;
		}

		ret = read(fd, slide->imgdata, slide->imgsize);
		if (ret < 0) {
			fprintf(stderr, "failed to read from %s/%s: %u\n",
				dirname, file->d_name, errno);
			ret = errno;
			goto err_free_imgdata;
		}

		list_append(&slide->list, &src->slides);
		close(fd);
	}

	if (list_empty(&src->slides)) {
		fprintf(stderr, "failed to find any images in %s\n", dirname);
		ret = -ENOENT;
		goto err_free_cwd;
	}

	ret = chdir(cwd);
	if (ret) {
		fprintf(stderr, "unable to cd to directory '%s': %s (%d)\n",
			cwd, strerror(ret), ret);
		goto err_free_cwd;
	}

	free(cwd);
	closedir(dir);
	src->cur_slide = list_first_entry(&src->slides, struct slide, list);

	return 0;

err_free_imgdata:
	free(slide->imgdata);
err_free_slide:
	free(slide);
err_close_fd:
	close(fd);
err_unwind:
	list_for_each_entry_safe(slide, next, &src->slides, list) {
		list_remove(&slide->list);
		free(slide->imgdata);
		free(slide);
	}
err_free_cwd:
	chdir(cwd);
	free(cwd);
err_close_dir:
	closedir(dir);
err_dummy_slide:

	/*
	* At present, there is no means of stalling a USB SET_CUR control from
	* the host; this means that the format passed here _must_ be accepted
	* until this issue is resolved. To work around the issue for now simply
	* use a single dummy slide... as long as we manage to allocate the
	* memory for it at least.
	*/

	printf("using dummy slideshow data\n");

	slide = malloc(sizeof(*slide));
	if (!slide) {
		fprintf(stderr, "failed to allocate memory for slide\n");
		return ret;
	}

	slide->imgsize = fmt->width * fmt->height * 2;

	slide->imgdata = malloc(slide->imgsize);
	if (!slide->imgdata) {
		fprintf(stderr, "failed to allocate memory for image\n");
		free(slide);
		return -ENOMEM;
	}

	memset(slide->imgdata, 0, slide->imgsize);
	list_append(&slide->list, &src->slides);
	src->cur_slide = slide;

	return ret;
}

static int slideshow_source_set_frame_rate(struct video_source *s,
					   unsigned int fps)
{
	struct slideshow_source *src = to_slideshow_source(s);

	timer_set_fps(src->timer, fps);

	return 0;
}

static int slideshow_source_free_buffers(struct video_source *s __attribute__((unused)))
{
	return 0;
}

static int slideshow_source_stream_on(struct video_source *s)
{
	struct slideshow_source *src = to_slideshow_source(s);
	int ret;

	ret = timer_arm(src->timer);
	if (ret)
		return ret;

	src->streaming = true;
	return 0;
}

static int slideshow_source_stream_off(struct video_source *s)
{
	struct slideshow_source *src = to_slideshow_source(s);

	/*
	 * No error check here, because we want to flag that streaming is over
	 * even if the timer is still running due to the failure.
	 */
	timer_disarm(src->timer);
	src->streaming = false;

	return 0;
}

static void slideshow_source_fill_buffer(struct video_source *s,
					 struct video_buffer *buf)
{
	struct slideshow_source *src = to_slideshow_source(s);
	unsigned int size;

	/*
	 * Nothing currently stops the user from providing a source file which is
	 * larger than the buffer size calculated from the format we have set.
	 * Clamp the size of the buffer we copy to be sure we don't overflow the
	 * buffer we was allocated to receive it.
	 */
	size = min(src->cur_slide->imgsize, buf->size);
	memcpy(buf->mem, src->cur_slide->imgdata, size);
	buf->bytesused = size;

	if (src->cur_slide == list_last_entry(&src->slides, struct slide, list))
		src->cur_slide = list_first_entry(&src->slides, struct slide, list);
	else
		src->cur_slide = list_next_entry(&src->cur_slide->list, struct slide, list);

	/*
	 * Wait for the timer to elapse to ensure that our configured frame rate
	 * is adhered to.
	 */
	if (src->streaming)
		timer_wait(src->timer);
}

static const struct video_source_ops slideshow_source_ops = {
	.destroy = slideshow_source_destroy,
	.set_format = slideshow_source_set_format,
	.set_frame_rate = slideshow_source_set_frame_rate,
	.free_buffers = slideshow_source_free_buffers,
	.stream_on = slideshow_source_stream_on,
	.stream_off = slideshow_source_stream_off,
	.queue_buffer = NULL,
	.fill_buffer = slideshow_source_fill_buffer,
};

struct video_source *slideshow_video_source_create(const char *img_dir)
{
	struct slideshow_source *src;

	if (img_dir == NULL)
		return NULL;

	if (strlen(img_dir) > 31)
		return NULL;

	src = malloc(sizeof *src);
	if (!src)
		return NULL;

	memset(src, 0, sizeof *src);
	src->src.ops = &slideshow_source_ops;
	src->src.type = VIDEO_SOURCE_STATIC;

	strncpy(src->img_dir, img_dir, sizeof(src->img_dir));

	src->timer = timer_new();
	if (!src->timer)
		goto err_free_src;

	list_init(&src->slides);

	return &src->src;

err_free_src:
	free(src);
	return NULL;
}

void slideshow_video_source_init(struct video_source *s, struct events *events)
{
	struct slideshow_source *src = to_slideshow_source(s);

	src->src.events = events;
}
