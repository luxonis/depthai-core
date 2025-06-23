/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * V4L2 Devices
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * This file originally comes from the omap3-isp-live project
 * (git://git.ideasonboard.org/omap3-isp-live.git)
 *
 * Copyright (C) 2010-2011 Ideas on board SPRL
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/time.h>

#include "list.h"
#include "tools.h"
#include "v4l2.h"
#include "video-buffers.h"

#ifndef V4L2_BUF_FLAG_ERROR
#define V4L2_BUF_FLAG_ERROR	0x0040
#endif

struct v4l2_ival_desc {
	struct v4l2_fract min;
	struct v4l2_fract max;
	struct v4l2_fract step;

	struct list_entry list;
};

struct v4l2_frame_desc {
	unsigned int min_width;
	unsigned int min_height;
	unsigned int max_width;
	unsigned int max_height;
	unsigned int step_width;
	unsigned int step_height;

	struct list_entry list;
	struct list_entry ivals;
};

struct v4l2_format_desc {
	unsigned int pixelformat;

	struct list_entry list;
	struct list_entry frames;
};

/* -----------------------------------------------------------------------------
 * Formats enumeration
 */

static int
v4l2_enum_frame_intervals(struct v4l2_device *dev, struct v4l2_format_desc *format,
	struct v4l2_frame_desc *frame)
{
	struct v4l2_ival_desc *ival;
	unsigned int i;
	int ret;

	for (i = 0; ; ++i) {
		struct v4l2_frmivalenum ivalenum;

		memset(&ivalenum, 0, sizeof ivalenum);
		ivalenum.index = i;
		ivalenum.pixel_format = format->pixelformat;
		ivalenum.width = frame->min_width;
		ivalenum.height = frame->min_height;
		ret = ioctl(dev->fd, VIDIOC_ENUM_FRAMEINTERVALS, &ivalenum);
		if (ret < 0)
			break;

		if (i != ivalenum.index)
			printf("Warning: driver returned wrong ival index "
				"%u.\n", ivalenum.index);
		if (format->pixelformat != ivalenum.pixel_format)
			printf("Warning: driver returned wrong ival pixel "
				"format %08x.\n", ivalenum.pixel_format);
		if (frame->min_width != ivalenum.width)
			printf("Warning: driver returned wrong ival width "
				"%u.\n", ivalenum.width);
		if (frame->min_height != ivalenum.height)
			printf("Warning: driver returned wrong ival height "
				"%u.\n", ivalenum.height);

		ival = malloc(sizeof *ival);
		if (ival == NULL)
			return -ENOMEM;

		memset(ival, 0, sizeof *ival);

		switch (ivalenum.type) {
		case V4L2_FRMIVAL_TYPE_DISCRETE:
			ival->min = ivalenum.discrete;
			ival->max = ivalenum.discrete;
			ival->step.numerator = 1;
			ival->step.denominator = 1;
			break;

		case V4L2_FRMIVAL_TYPE_STEPWISE:
			ival->min = ivalenum.stepwise.min;
			ival->max = ivalenum.stepwise.max;
			ival->step = ivalenum.stepwise.step;
			break;

		default:
			printf("Error: driver returned invalid frame ival "
				"type %u\n", ivalenum.type);
			return -EINVAL;
		}

		list_append(&ival->list, &frame->ivals);
	}

	return 0;
}

static int
v4l2_enum_frame_sizes(struct v4l2_device *dev, struct v4l2_format_desc *format)
{
	struct v4l2_frame_desc *frame;
	unsigned int i;
	int ret;

	for (i = 0; ; ++i) {
		struct v4l2_frmsizeenum frmenum;

		memset(&frmenum, 0, sizeof frmenum);
		frmenum.index = i;
		frmenum.pixel_format = format->pixelformat;

		ret = ioctl(dev->fd, VIDIOC_ENUM_FRAMESIZES, &frmenum);
		if (ret < 0)
			break;

		if (i != frmenum.index)
			printf("Warning: driver returned wrong frame index "
				"%u.\n", frmenum.index);
		if (format->pixelformat != frmenum.pixel_format)
			printf("Warning: driver returned wrong frame pixel "
				"format %08x.\n", frmenum.pixel_format);

		frame = malloc(sizeof *frame);
		if (frame == NULL)
			return -ENOMEM;

		memset(frame, 0, sizeof *frame);

		list_init(&frame->ivals);
		frame->step_width = 1;
		frame->step_height = 1;

		switch (frmenum.type) {
		case V4L2_FRMSIZE_TYPE_DISCRETE:
			frame->min_width = frmenum.discrete.width;
			frame->min_height = frmenum.discrete.height;
			frame->max_width = frmenum.discrete.width;
			frame->max_height = frmenum.discrete.height;
			break;

		case V4L2_FRMSIZE_TYPE_STEPWISE:
			frame->step_width = frmenum.stepwise.step_width;
			frame->step_height = frmenum.stepwise.step_height;
			/* fallthrough */
		case V4L2_FRMSIZE_TYPE_CONTINUOUS:
			frame->min_width = frmenum.stepwise.min_width;
			frame->min_height = frmenum.stepwise.min_height;
			frame->max_width = frmenum.stepwise.max_width;
			frame->max_height = frmenum.stepwise.max_height;
			break;

		default:
			printf("Error: driver returned invalid frame size "
				"type %u\n", frmenum.type);
			return -EINVAL;
		}

		list_append(&frame->list, &format->frames);

		ret = v4l2_enum_frame_intervals(dev, format, frame);
		if (ret < 0)
			return ret;
	}

	return 0;
}
static int v4l2_enum_formats(struct v4l2_device *dev)
{
	struct v4l2_format_desc *format;
	unsigned int i;
	int ret;

	for (i = 0; ; ++i) {
		struct v4l2_fmtdesc fmtenum;

		memset(&fmtenum, 0, sizeof fmtenum);
		fmtenum.index = i;
		fmtenum.type = dev->type;

		ret = ioctl(dev->fd, VIDIOC_ENUM_FMT, &fmtenum);
		if (ret < 0)
			break;

		if (i != fmtenum.index)
			printf("Warning: driver returned wrong format index "
				"%u.\n", fmtenum.index);
		if (dev->type != fmtenum.type)
			printf("Warning: driver returned wrong format type "
				"%u.\n", fmtenum.type);

		format = malloc(sizeof *format);
		if (format == NULL)
			return -ENOMEM;

		memset(format, 0, sizeof *format);

		list_init(&format->frames);
		format->pixelformat = fmtenum.pixelformat;

		list_append(&format->list, &dev->formats);

		ret = v4l2_enum_frame_sizes(dev, format);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Open/close
 */

struct v4l2_device *v4l2_open(const char *devname)
{
	struct v4l2_device *dev;
	struct v4l2_capability cap;
	__u32 capabilities;
	int ret;

	dev = malloc(sizeof *dev);
	if (dev == NULL)
		return NULL;

	memset(dev, 0, sizeof *dev);
	dev->fd = -1;
	dev->name = strdup(devname);
	list_init(&dev->formats);

	dev->fd = open(devname, O_RDWR | O_NONBLOCK);
	if (dev->fd < 0) {
		printf("Error opening device %s: %d.\n", devname, errno);
		v4l2_close(dev);
		return NULL;
	}

	memset(&cap, 0, sizeof cap);
	ret = ioctl(dev->fd, VIDIOC_QUERYCAP, &cap);
	if (ret < 0) {
		printf("Error opening device %s: unable to query "
			"device.\n", devname);
		v4l2_close(dev);
		return NULL;
	}

	/* 
	 * If the device_caps field is set use it, otherwise use the older
	 * capabilities field.
	 */
	capabilities = cap.device_caps ? : cap.capabilities;

	if (capabilities & V4L2_CAP_VIDEO_CAPTURE)
		dev->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	else if (capabilities & V4L2_CAP_VIDEO_OUTPUT)
		dev->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	else {
		printf("Error opening device %s: neither video capture "
			"nor video output supported.\n", devname);
		v4l2_close(dev);
		return NULL;
	}

	ret = v4l2_enum_formats(dev);
	if (ret < 0) {
		printf("Error opening device %s: unable to enumerate "
			"formats.\n", devname);
		v4l2_close(dev);
		return NULL;
	}

	printf("Device %s opened: %s (%s).\n", devname, cap.card, cap.bus_info);

	return dev;
}

void v4l2_close(struct v4l2_device *dev)
{
	struct v4l2_format_desc *format, *next_fmt;
	struct v4l2_frame_desc *frame, *next_frm;
	struct v4l2_ival_desc *ival, *next_ival;

	if (dev == NULL)
		return;

	list_for_each_entry_safe(format, next_fmt, &dev->formats, list) {
		list_for_each_entry_safe(frame, next_frm, &format->frames, list) {
			list_for_each_entry_safe(ival, next_ival, &frame->ivals, list) {
				free(ival);
			}
			free(frame);
		}
		free(format);
	}

	free(dev->name);
	close(dev->fd);
	free(dev);
}

/* -----------------------------------------------------------------------------
 * Controls
 */

int v4l2_get_control(struct v4l2_device *dev, unsigned int id, int32_t *value)
{
	struct v4l2_control ctrl;
	int ret;

	ctrl.id = id;

	ret = ioctl(dev->fd, VIDIOC_G_CTRL, &ctrl);
	if (ret < 0) {
		printf("%s: unable to get control (%d).\n", dev->name, errno);
		return -errno;
	}

	*value = ctrl.value;
	return 0;
}

int v4l2_set_control(struct v4l2_device *dev, unsigned int id, int32_t *value)
{
	struct v4l2_control ctrl;
	int ret;

	ctrl.id = id;
	ctrl.value = *value;

	ret = ioctl(dev->fd, VIDIOC_S_CTRL, &ctrl);
	if (ret < 0) {
		printf("%s: unable to set control (%d).\n", dev->name, errno);
		return -errno;
	}

	*value = ctrl.value;
	return 0;
}

int v4l2_get_controls(struct v4l2_device *dev, unsigned int count,
		      struct v4l2_ext_control *ctrls)
{
	struct v4l2_ext_controls controls;
	int ret;

	memset(&controls, 0, sizeof controls);
	controls.count = count;
	controls.controls = ctrls;

	ret = ioctl(dev->fd, VIDIOC_G_EXT_CTRLS, &controls);
	if (ret < 0)
		printf("%s: unable to get multiple controls (%d).\n", dev->name,
		       errno);

	return ret;
}

int v4l2_set_controls(struct v4l2_device *dev, unsigned int count,
		      struct v4l2_ext_control *ctrls)
{
	struct v4l2_ext_controls controls;
	int ret;

	memset(&controls, 0, sizeof controls);
	controls.count = count;
	controls.controls = ctrls;

	ret = ioctl(dev->fd, VIDIOC_S_EXT_CTRLS, &controls);
	if (ret < 0)
		printf("%s: unable to set multiple controls (%d).\n", dev->name,
		       errno);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Formats and frame rates
 */

int v4l2_get_crop(struct v4l2_device *dev, struct v4l2_rect *rect)
{
	struct v4l2_crop crop;
	int ret;

	memset(&crop, 0, sizeof crop);
	crop.type = dev->type;

	ret = ioctl(dev->fd, VIDIOC_G_CROP, &crop);
	if (ret < 0) {
		printf("%s: unable to get crop rectangle (%d).\n", dev->name,
		       errno);
		return -errno;
	}

	dev->crop = crop.c;
	*rect = crop.c;

	return 0;
}

int v4l2_set_crop(struct v4l2_device *dev, struct v4l2_rect *rect)
{
	struct v4l2_crop crop;
	int ret;

	memset(&crop, 0, sizeof crop);
	crop.type = dev->type;
	crop.c = *rect;

	ret = ioctl(dev->fd, VIDIOC_S_CROP, &crop);
	if (ret < 0) {
		printf("%s: unable to set crop rectangle (%d).\n", dev->name,
		       errno);
		return -errno;
	}

	dev->crop = crop.c;
	*rect = crop.c;

	return 0;
}

int v4l2_get_format(struct v4l2_device *dev, struct v4l2_pix_format *format)
{
	struct v4l2_format fmt;
	int ret;

	memset(&fmt, 0, sizeof fmt);
	fmt.type = dev->type;

	ret = ioctl(dev->fd, VIDIOC_G_FMT, &fmt);
	if (ret < 0) {
		printf("%s: unable to get format (%d).\n", dev->name, errno);
		return -errno;
	}

	dev->format = fmt.fmt.pix;
	*format = fmt.fmt.pix;

	return 0;
}

int v4l2_set_format(struct v4l2_device *dev, struct v4l2_pix_format *format)
{
	struct v4l2_format fmt;
	int ret;

	memset(&fmt, 0, sizeof fmt);
	fmt.type = dev->type;
	fmt.fmt.pix.width = format->width;
	fmt.fmt.pix.height = format->height;
	fmt.fmt.pix.pixelformat = format->pixelformat;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;
	fmt.fmt.pix.sizeimage = format->sizeimage;

	ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt);
	if (ret < 0) {
		printf("%s: unable to set format (%d).\n", dev->name, errno);
		return -errno;
	}

	dev->format = fmt.fmt.pix;
	*format = fmt.fmt.pix;

	return 0;
}

int v4l2_set_frame_rate(struct v4l2_device *dev, unsigned int fps)
{
	struct v4l2_streamparm parm;
	int ret;

	memset(&parm, 0, sizeof parm);
	parm.type = dev->type;
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = fps;

	ret = ioctl(dev->fd, VIDIOC_S_PARM, &parm);
	if (ret < 0) {
		printf("%s: unable to set frame rate (%d).\n", dev->name, errno);
		return -errno;
	}

	dev->fps = fps;
	return 0;
}

/* -----------------------------------------------------------------------------
 * Buffers management
 */

int v4l2_alloc_buffers(struct v4l2_device *dev, enum v4l2_memory memtype,
		       unsigned int nbufs)
{
	struct v4l2_requestbuffers rb;
	unsigned int i;
	int ret;

	if (dev->buffers.nbufs != 0)
		return -EBUSY;

	if (memtype != V4L2_MEMORY_MMAP && memtype != V4L2_MEMORY_DMABUF)
		return -EINVAL;

	/* Request the buffers from the driver. */
	memset(&rb, 0, sizeof rb);
	rb.count = nbufs;
	rb.type = dev->type;
	rb.memory = memtype;

	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		printf("%s: unable to request buffers (%d).\n", dev->name,
		       errno);
		ret = -errno;
		goto done;
	}

	if (rb.count > nbufs) {
		printf("%s: driver needs more buffers (%u) than available (%u).\n",
		       dev->name, rb.count, nbufs);
		ret = -E2BIG;
		goto done;
	}

	printf("%s: %u buffers requested.\n", dev->name, rb.count);

	/* Allocate the buffer objects. */
	dev->memtype = memtype;
	dev->buffers.nbufs = rb.count;

	dev->buffers.buffers = calloc(nbufs, sizeof *dev->buffers.buffers);
	if (dev->buffers.buffers == NULL) {
		ret = -ENOMEM;
		goto done;
	}

	for (i = 0; i < dev->buffers.nbufs; ++i) {
		dev->buffers.buffers[i].index = i;
		dev->buffers.buffers[i].dmabuf = -1;
	}

	ret = 0;

done:
	if (ret < 0)
		v4l2_free_buffers(dev);

	return ret;
}

int v4l2_free_buffers(struct v4l2_device *dev)
{
	struct v4l2_requestbuffers rb;
	unsigned int i;
	int ret;

	if (dev->buffers.nbufs == 0)
		return 0;

	for (i = 0; i < dev->buffers.nbufs; ++i) {
		struct video_buffer *buffer = &dev->buffers.buffers[i];

		if (buffer->mem) {
			ret = munmap(buffer->mem, buffer->size);
			if (ret < 0) {
				printf("%s: unable to unmap buffer %u (%d)\n",
				       dev->name, i, errno);
				return -errno;
			}

			buffer->mem = NULL;
		}

		if (buffer->dmabuf != -1) {
			close(buffer->dmabuf);
			buffer->dmabuf = -1;
		}

		buffer->size = 0;
	}

	memset(&rb, 0, sizeof rb);
	rb.count = 0;
	rb.type = dev->type;
	rb.memory = dev->memtype;

	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		printf("%s: unable to release buffers (%d)\n", dev->name,
		       errno);
		return -errno;
	}

	free(dev->buffers.buffers);
	dev->buffers.buffers = NULL;
	dev->buffers.nbufs = 0;

	return 0;
}

int v4l2_export_buffers(struct v4l2_device *dev)
{
	unsigned int i;
	int ret;

	if (dev->buffers.nbufs == 0)
		return -EINVAL;

	if (dev->memtype != V4L2_MEMORY_MMAP)
		return -EINVAL;

	for (i = 0; i < dev->buffers.nbufs; ++i) {
		struct v4l2_exportbuffer expbuf = {
			.type = dev->type,
			.index = i,
		};
		struct v4l2_buffer buf = {
			.index = i,
			.type = dev->type,
			.memory = dev->memtype,
		};

		ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			printf("%s: unable to query buffer %u (%d).\n",
			       dev->name, i, errno);
			return -errno;
		}

		ret = ioctl(dev->fd, VIDIOC_EXPBUF, &expbuf);
		if (ret < 0) {
			printf("Failed to export buffer %u.\n", i);
			return -errno;
		}

		dev->buffers.buffers[i].size = buf.length;
		dev->buffers.buffers[i].dmabuf = expbuf.fd;

		printf("%s: buffer %u exported with fd %u.\n",
		       dev->name, i, dev->buffers.buffers[i].dmabuf);
	}

	return 0;
}

int v4l2_import_buffers(struct v4l2_device *dev,
			const struct video_buffer_set *buffers)
{
	unsigned int i;
	int ret;

	if (dev->buffers.nbufs == 0 || dev->buffers.nbufs > buffers->nbufs)
		return -EINVAL;

	if (dev->memtype != V4L2_MEMORY_DMABUF)
		return -EINVAL;

	for (i = 0; i < dev->buffers.nbufs; ++i) {
		const struct video_buffer *buffer = &buffers->buffers[i];
		struct v4l2_buffer buf = {
			.index = i,
			.type = dev->type,
			.memory = dev->memtype,
		};
		int fd;

		ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			printf("%s: unable to query buffer %u (%d).\n",
			       dev->name, i, errno);
			return -errno;
		}

		if (buffer->size < buf.length) {
			printf("%s: buffer %u too small (%u bytes required, %u bytes available).\n",
			       dev->name, i, buf.length, buffer->size);
			return -EINVAL;
		}

		fd = dup(buffer->dmabuf);
		if (fd < 0) {
			printf("%s: failed to duplicate dmabuf fd %d.\n",
			       dev->name, buffer->dmabuf);
			return ret;
		}

		printf("%s: buffer %u valid.\n", dev->name, i);

		dev->buffers.buffers[i].dmabuf = fd;
		dev->buffers.buffers[i].size = buffer->size;
	}

	return 0;
}

int v4l2_mmap_buffers(struct v4l2_device *dev)
{
	unsigned int i;
	int ret;

	if (dev->memtype != V4L2_MEMORY_MMAP)
		return -EINVAL;

	for (i = 0; i < dev->buffers.nbufs; ++i) {
		struct video_buffer *buffer = &dev->buffers.buffers[i];
		struct v4l2_buffer buf = {
			.index = i,
			.type = dev->type,
			.memory = dev->memtype,
		};
		void *mem;

		ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			printf("%s: unable to query buffer %u (%d).\n",
			       dev->name, i, errno);
			return -errno;
		}

		mem = mmap(0, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
			   dev->fd, buf.m.offset);
		if (mem == MAP_FAILED) {
			printf("%s: unable to map buffer %u (%d)\n",
			       dev->name, i, errno);
			return -errno;
		}

		buffer->mem = mem;
		buffer->size = buf.length;

		printf("%s: buffer %u mapped at address %p.\n", dev->name, i,
		       mem);
	}

	return 0;
}

int v4l2_dequeue_buffer(struct v4l2_device *dev, struct video_buffer *buffer)
{
	struct v4l2_buffer buf;
	int ret;

	memset(&buf, 0, sizeof buf);
	buf.type = dev->type;
	buf.memory = dev->memtype;

	ret = ioctl(dev->fd, VIDIOC_DQBUF, &buf);
	if (ret < 0) {
		printf("%s: unable to dequeue buffer index %u/%u (%d)\n",
		       dev->name, buf.index, dev->buffers.nbufs, errno);
		return -errno;
	}

	buffer->index = buf.index;
	buffer->size = buf.length;
	buffer->mem = dev->buffers.buffers[buf.index].mem;
	buffer->bytesused = buf.bytesused;
	buffer->timestamp = buf.timestamp;
	buffer->error = !!(buf.flags & V4L2_BUF_FLAG_ERROR);

	return 0;
}

int v4l2_queue_buffer(struct v4l2_device *dev, struct video_buffer *buffer)
{
	struct v4l2_buffer buf;
	int ret;

	if (buffer->index >= dev->buffers.nbufs)
		return -EINVAL;

	memset(&buf, 0, sizeof buf);
	buf.index = buffer->index;
	buf.type = dev->type;
	buf.memory = dev->memtype;

	if (dev->memtype == V4L2_MEMORY_DMABUF)
		buf.m.fd = (unsigned long)dev->buffers.buffers[buffer->index].dmabuf;

	if (dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		buf.bytesused = buffer->bytesused;

	ret = ioctl(dev->fd, VIDIOC_QBUF, &buf);
	if (ret < 0) {
		printf("%s: unable to queue buffer index %u/%u (%d)\n",
		       dev->name, buf.index, dev->buffers.nbufs, errno);
		return -errno;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Stream management
 */

int v4l2_stream_on(struct v4l2_device *dev)
{
	int type = dev->type;
	int ret;

	ret = ioctl(dev->fd, VIDIOC_STREAMON, &type);
	if (ret < 0)
		return -errno;

	return 0;
}

int v4l2_stream_off(struct v4l2_device *dev)
{
	int type = dev->type;
	int ret;

	ret = ioctl(dev->fd, VIDIOC_STREAMOFF, &type);
	if (ret < 0)
		return -errno;

	return 0;
}
