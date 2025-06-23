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
#ifndef __V4L2_H
#define __V4L2_H

#include <linux/videodev2.h>
#include <stdint.h>

#include "list.h"
#include "video-buffers.h"

struct v4l2_device
{
	int fd;
	char *name;

	enum v4l2_buf_type type;
	enum v4l2_memory memtype;

	struct list_entry formats;
	struct v4l2_pix_format format;
	struct v4l2_rect crop;
	unsigned int fps;

	struct video_buffer_set buffers;
};

/*
 * v4l2_open - Open a V4L2 device
 * @devname: Name (including path) of the device node
 *
 * Open the V4L2 device referenced by @devname for video capture or display in
 * non-blocking mode.
 *
 * If the device can be opened, query its capabilities and enumerates frame
 * formats, sizes and intervals.
 *
 * Return a pointer to a newly allocated v4l2_device structure instance on
 * success and NULL on failure. The returned pointer must be freed with
 * v4l2_close when the device isn't needed anymore.
 */
struct v4l2_device *v4l2_open(const char *devname);

/*
 * v4l2_close - Close a V4L2 device
 * @dev: Device instance
 *
 * Close the device instance given as argument and free allocated resources.
 * Access to the device instance is forbidden after this function returns.
 */
void v4l2_close(struct v4l2_device *dev);

/*
 * v4l2_get_format - Retrieve the current pixel format
 * @dev: Device instance
 * @format: Pixel format structure to be filled
 *
 * Query the device to retrieve the current pixel format and frame size and fill
 * the @format structure.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_get_format(struct v4l2_device *dev, struct v4l2_pix_format *format);

/*
 * v4l2_set_format - Set the pixel format
 * @dev: Device instance
 * @format: Pixel format structure to be set
 *
 * Set the pixel format and frame size stored in @format. The device can modify
 * the requested format and size, in which case the @format structure will be
 * updated to reflect the modified settings.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_set_format(struct v4l2_device *dev, struct v4l2_pix_format *format);

/*
 * v4l2_set_frame_rate - Set the frame rate
 * @dev: Device instance
 * @fps: Frame rate
 *
 * Set the frame rate specified by @fps.
 * The device can modify the requested format and size, in which case @dev->fps
 * will be updated to reflect the modified setting.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_set_frame_rate(struct v4l2_device *dev, unsigned int fps);

/*
 * v4l2_get_crop - Retrieve the current crop rectangle
 * @dev: Device instance
 * @rect: Crop rectangle structure to be filled
 *
 * Query the device to retrieve the current crop rectangle and fill the @rect
 * structure.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_get_crop(struct v4l2_device *dev, struct v4l2_rect *rect);

/*
 * v4l2_set_crop - Set the crop rectangle
 * @dev: Device instance
 * @rect: Crop rectangle structure to be set
 *
 * Set the crop rectangle stored in @rect. The device can modify the requested
 * rectangle, in which case the @rect structure will be updated to reflect the
 * modified settings.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_set_crop(struct v4l2_device *dev, struct v4l2_rect *rect);

/*
 * v4l2_alloc_buffers - Allocate buffers for video frames
 * @dev: Device instance
 * @memtype: Type of buffers
 * @nbufs: Number of buffers to allocate
 *
 * Request the driver to allocate @nbufs buffers. The driver can modify the
 * number of buffers depending on its needs. The number of allocated buffers
 * will be stored in the @dev->buffers.nbufs field.
 *
 * When @memtype is set to V4L2_MEMORY_MMAP the buffers are allocated by the
 * driver. They can then be mapped to userspace by calling v4l2_mmap_buffers().
 * When @memtype is set to V4L2_MEMORY_DMABUF the driver only allocates buffer
 * objects and relies on the application to provide memory storage for video
 * frames.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_alloc_buffers(struct v4l2_device *dev, enum v4l2_memory memtype,
		       unsigned int nbufs);

/*
 * v4l2_free_buffers - Free buffers
 * @dev: Device instance
 *
 * Free buffers previously allocated by v4l2_alloc_buffers(). If the buffers
 * have been allocated with the V4L2_MEMORY_DMABUF memory type only the buffer
 * objects are freed, and the caller is responsible for freeing the video frames
 * memory if required.
 *
 * When successful this function sets the @dev->buffers.nbufs field to zero.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_free_buffers(struct v4l2_device *dev);

/*
 * v4l2_export_buffers - Export buffers as dmabuf objects
 * @dev: Device instance
 *
 * Export all the buffers previously allocated by v4l2_alloc_buffers() as dmabuf
 * objects. The dmabuf objects handles can be accessed through the dmabuf field
 * of each entry in the @dev::buffers array.
 *
 * The dmabuf objects handles will be automatically closed when the buffers are
 * freed with v4l2_free_buffers().
 *
 * This function can only be called when buffers have been allocated with the
 * memory type set to V4L2_MEMORY_MMAP. If the memory type is different, or if
 * no buffers have been allocated, it will return -EINVAL.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_export_buffers(struct v4l2_device *dev);

/*
 * v4l2_import_buffers - Import buffer backing store as dmabuf objects
 * @dev: Device instance
 * @buffers: Buffers to be imported
 *
 * Import the dmabuf objects from @buffers as backing store for the device
 * buffers previously allocated by v4l2_alloc_buffers().
 *
 * The dmabuf file handles are duplicated and stored in the dmabuf field of the
 * @dev::buffers array. The handles from the @buffers set can thus be closed
 * independently without impacting usage of the imported dmabuf objects. The
 * duplicated file handles will be automatically closed when the buffers are
 * freed with v4l2_free_buffers().
 *
 * This function can only be called when buffers have been allocated with the
 * memory type set to V4L2_MEMORY_DMABUF. If the memory type is different, if no
 * buffers have been allocated, or if the number of allocated buffers is larger
 * than @buffers->nbufs, it will return -EINVAL.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_import_buffers(struct v4l2_device *dev,
			const struct video_buffer_set *buffers);

/*
 * v4l2_mmap_buffers - Map buffers to application memory space
 * @dev: Device instance
 *
 * Map all the buffers previously allocated by v4l2_alloc_buffers() to the
 * application memory space. The buffer memory can be accessed through the mem
 * field of each entry in the @dev::buffers array.
 *
 * Buffers will be automatically unmapped when freed with v4l2_free_buffers().
 *
 * This function can only be called when buffers have been allocated with the
 * memory type set to V4L2_MEMORY_MMAP. If the memory type is different, or if
 * no buffers have been allocated, it will return -EINVAL.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_mmap_buffers(struct v4l2_device *dev);

/*
 * v4l2_queue_buffer - Queue a buffer for video capture/output
 * @dev: Device instance
 * @buffer: Buffer to be queued
 *
 * Queue the buffer identified by @buffer for video capture or output, depending
 * on the device type.
 *
 * The caller must initialize the @buffer::index field with the index of the
 * buffer to be queued. The index is zero-based and must be lower than the
 * number of allocated buffers.
 *
 * For V4L2_MEMORY_DMABUF buffers, the caller must initialize the @buffer::dmabuf
 * field with the address of the video frame memory, and the @buffer:length
 * field with its size in bytes. For optimal performances the address and length
 * should be identical between v4l2_queue_buffer() calls for a given buffer
 * index.
 *
 * For video output, the caller must initialize the @buffer::bytesused field
 * with the size of video data. The value should differ from the buffer length
 * for variable-size video formats only.
 *
 * Upon successful return the buffer ownership is transferred to the driver. The
 * caller must not touch video memory for that buffer before calling
 * v4l2_dequeue_buffer(). Attempting to queue an already queued buffer will
 * fail.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_queue_buffer(struct v4l2_device *dev, struct video_buffer *buffer);

/*
 * v4l2_dequeue_buffer - Dequeue the next buffer
 * @dev: Device instance
 * @buffer: Dequeued buffer data to be filled
 *
 * Dequeue the next buffer processed by the driver and fill all fields in
 * @buffer. 
 *
 * This function does not block. If no buffer is ready it will return
 * immediately with -EAGAIN.
 *
 * If an error occured during video capture or display, the @buffer::error field
 * is set to true. Depending on the device the video data can be partly
 * corrupted or complete garbage.
 *
 * Once dequeued the buffer ownership is transferred to the caller. Video memory
 * for that buffer can be safely read from and written to.
 *
 * Return 0 on success or a negative error code on failure. An error that
 * results in @buffer:error being set is not considered as a failure condition
 * for the purpose of the return value.
 */
int v4l2_dequeue_buffer(struct v4l2_device *dev, struct video_buffer *buffer);

/*
 * v4l2_stream_on - Start video streaming
 * @dev: Device instance
 *
 * Start video capture or output on the device. For video output devices at
 * least one buffer must be queued before starting the stream.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_stream_on(struct v4l2_device *dev);

/*
 * v4l2_stream_off - Stop video streaming
 * @dev: Device instance
 *
 * Stop video capture or output on the device. Upon successful return ownership
 * of all buffers is returned to the caller.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_stream_off(struct v4l2_device *dev);

/*
 * v4l2_get_control - Read the value of a control
 * @dev: Device instance
 * @id: Control ID
 * @value: Control value to be filled
 *
 * Retrieve the current value of control @id and store it in @value.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_get_control(struct v4l2_device *dev, unsigned int id, int32_t *value);

/*
 * v4l2_set_control - Write the value of a control
 * @dev: Device instance
 * @id: Control ID
 * @value: Control value
 *
 * Set control @id to @value. The device is allowed to modify the requested
 * value, in which case @value is updated to the modified value.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_set_control(struct v4l2_device *dev, unsigned int id, int32_t *value);

/*
 * v4l2_get_controls - Read the value of multiple controls
 * @dev: Device instance
 * @count: Number of controls
 * @ctrls: Controls to be read
 *
 * Retrieve the current value of controls identified by @ctrls.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_get_controls(struct v4l2_device *dev, unsigned int count,
		      struct v4l2_ext_control *ctrls);

/*
 * v4l2_set_controls - Write the value of multiple controls
 * @dev: Device instance
 * @count: Number of controls
 * @ctrls: Controls to be written
 *
 * Set controls identified by @ctrls. The device is allowed to modify the
 * requested values, in which case @ctrls is updated to the modified value.
 *
 * Return 0 on success or a negative error code on failure.
 */
int v4l2_set_controls(struct v4l2_device *dev, unsigned int count,
		      struct v4l2_ext_control *ctrls);

#endif
