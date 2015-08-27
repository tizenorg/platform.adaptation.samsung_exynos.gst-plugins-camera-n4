/*
 * camerasrc
 *
 * Copyright (c) 2000 - 2011 Samsung Electronics Co., Ltd. All rights reserved.
 *
 * Contact: Jeongmo Yang <jm80.yang@samsung.com>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#ifndef __GSTCAMERASRC_H__
#define __GSTCAMERASRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include <gst/video/colorbalance.h>
#include <gst/video/cameracontrol.h>
#include <gst/video/video-format.h>

#include "camerasrc.h"

#ifndef _SPEED_UP_RAW_CAPTURE
#define _SPEED_UP_RAW_CAPTURE
#endif /* _SPEED_UP_RAW_CAPTURE */

G_BEGIN_DECLS
#define GST_TYPE_CAMERA_SRC             (gst_camerasrc_get_type())
#define GST_CAMERA_SRC(obj)             (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_CAMERA_SRC,GstCameraSrc))
#define GST_CAMERA_SRC_CLASS(klass)     (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_CAMERA_SRC,GstCameraSrcClass))
#define GST_IS_CAMERA_SRC(obj)          (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_CAMERA_SRC))
#define GST_IS_CAMERA_SRC_CLASS(obj)    (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_CAMERA_SRC))


#define FORMAT_NAME_LENGTH              10


typedef struct _GstCameraSrc GstCameraSrc;
typedef struct _GstCameraSrcClass GstCameraSrcClass;
typedef struct _GstCameraBuffer GstCameraBuffer;
typedef struct _BufferInfo BufferInfo;

/* global info */
struct _GstCameraBuffer {
	GstBuffer *buffer;
	int v4l2_buffer_index;
	GstCameraSrc *camerasrc;
};

struct _BufferInfo {
	unsigned char *vaddr[MAX_PLANE_NUM];
	uint64_t size[MAX_PLANE_NUM];
	int dma_buf_fd[MAX_PLANE_NUM];
	tbm_bo bo[MAX_PLANE_NUM];
	unsigned int bytesused;
};

struct _GstCameraSrc
{
	GstPushSrc element;

	/*private*/
	void *v4l2_handle;                      /**< video4linux2 handle */
	int mode;
	gboolean vflip;                         /**< flip camera input vertically */
	gboolean hflip;                         /**< flip camera input horizontally */
	gboolean firsttime;

	int main_buf_sz;
	int cap_count_current;                  /**< current capture count */
	int cap_count_reverse;                  /**< current capture count (reverse counting) */
	unsigned long cap_next_time;            /**< next shot time for capture */
	GQueue *capture_cmd_list;               /**< Command list(Start capture, Stop capture) queue */

	GCond cond;
	GMutex mutex;

	/*camera property*/
	int width;                              /**< Width */
	int height;                             /**< Height */
	int fps;                                /**< Video source fps */

	gchar format_name[FORMAT_NAME_LENGTH];  /**< Format name */
	int pix_format;                         /**< Image format of video source */
	int colorspace;                         /**< Colorspace of video source */
	int high_speed_fps;                     /**< Video source fps for high speed recording */
	gboolean fps_auto;                      /**< Auto Video source fps */

	int camera_id;
	int rotate;                             /**< Video source rotate */
	gboolean use_rotate_caps;               /**< Use or not rotate value in caps */

	GCond buffer_cond;                     /**< condition for buffer control */
	GMutex buffer_lock;                    /**< lock for buffer control */
	gboolean buffer_running;                /**< with lock */
	gint num_live_buffers;                  /**< with lock */
	guint buffer_count;
	gboolean bfirst;                        /**< temp */
#ifdef _SPEED_UP_RAW_CAPTURE
	gboolean cap_stream_diff;               /**< whether preview and capture streams are different each other */
#endif
	GQueue *pad_alloc_list;
	GMutex pad_alloc_mutex;
	gint current_buffer_data_index;
	gboolean first_invokation;

	/* Colorbalance , CameraControl interface */
	GList *colors;
	GList *camera_controls;


	/*capture property*/
	guint32 cap_fourcc;                     /**< gstreamer fourcc value(GST_MAKE_FOURCC format) for raw capturing */
	int cap_width;                          /**< Capture width */
	int cap_height;                         /**< Capture height */
	int cap_interval;                       /**< Capture interval */
	int cap_count;                          /**< Capture count */
	int cap_jpg_quality;                    /**< Capture quality for jpg compress ratio */
	gboolean cap_provide_exif;              /**< Is exif provided? */
	gboolean flash_activated;               /**< Is flash activated when AF? - this can be used in case of zero shutter lag capture */
	gboolean is_hybrid_running;             /**< Is Hybrid mode running? */
	gboolean is_hybrid_capture;             /**< Is Hybrid mode capture frame? */

	BufferInfo *buffer_info;                /**< Buffer info - fd, vaddr, tbm_bo, size */

	/*etc property*/
	gboolean create_jpeg;
	GMutex jpg_mutex;
	GCond capture_cond;                    /**< cond for capture thread */
	GMutex capture_mutex;                  /**< mutex for capture thread */
	GQueue *capture_buffer_list;            /**< queue for buffer to capture */
	GThread *capture_thread;                /**< capture thread */
	gboolean quit_capture_thread;           /**< flag to quit capture thread */
	GQueue *restart_cmd_list;               /**< command list for restarting preview */
	gboolean flush_cache;                   /**< flush cache after YUV buffer is copied */
	GMutex restart_mutex;                  /**< mutex for preview restart */
	GCond restart_cond;                    /**< cond for preview restart */
};

struct _GstCameraSrcClass {
	GstPushSrcClass parent_class;
	/* signals */
	void (*still_capture) (GstElement *element, GstBuffer *main, GstBuffer *sub, GstBuffer *scrnl);
	void (*nego_complete) (GstElement *element);
	void (*register_trouble) (GstElement *element);
};

typedef enum {
	INTERFACE_NONE,
	INTERFACE_COLOR_BALANCE,
	INTERFACE_CAMERA_CONTROL,
} GstInterfaceType;


void gst_camerasrc_set_capture_command(GstCameraSrc* camerasrc, GstCameraControlCaptureCommand cmd);


GType gst_camerasrc_get_type(void);

G_END_DECLS

#endif /* __GSTCAMERASRC_H__ */
