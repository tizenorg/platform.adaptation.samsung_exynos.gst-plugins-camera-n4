/*
 * camerasrc
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd. All rights reserved.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include <gst/gstutils.h>
#include <gst/video/video-info.h>
#include <glib-object.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <mm_error.h>
#include <mm_types.h>
#include <mm_util_jpeg.h>

#include "gstcamerasrc.h"
#include "gstcamerasrccontrol.h"
#include "gstcamerasrccolorbalance.h"


/******************************************************************************
 * Definitions
 *******************************************************************************/
GST_DEBUG_CATEGORY (camerasrc_debug);
#define GST_CAT_DEFAULT camerasrc_debug

#ifndef YUV422_SIZE
#define YUV422_SIZE(width,height) ( ((width)*(height)) << 1 )
#endif

#ifndef YUV420_SIZE
#define YUV420_SIZE(width,height) ( ((width)*(height)*3) >> 1 )
#endif

#define ALIGN_SIZE_I420                 (1024<<2)
#define ALIGN_SIZE_NV12                 (1024<<6)
#define CAMERASRC_ALIGN(addr,size)      (((addr)+((size)-1))&(~((size)-1)))

#if !defined (CLEAR)
    #define CLEAR(x)                    memset(&(x), 0, sizeof(x))
#endif

/* Enables */
#define _ENABLE_CAMERASRC_DEBUG                 0

/* Local definitions */
#define _DEFAULT_WIDTH                          320
#define _DEFAULT_HEIGHT                         240
#define _DEFAULT_FPS                            30
#define _DEFAULT_HIGH_SPEED_FPS                 0
#define _DEFAULT_FPS_AUTO                       FALSE
#define _DEFAULT_PIX_FORMAT                     CAMERASRC_PIX_SN12
#define _DEFAULT_PIX_FORMAT_NAME                "SN12"
#define _DEFAULT_CAPTURE_FORMAT_NAME            "JPEG"
#define _DEFAULT_COLORSPACE                     CAMERASRC_COL_RAW
#define _DEFAULT_CAMERA_ID                      CAMERASRC_DEV_ID_PRIMARY

/* mmap/pad-alloc related definition */
#define _DEFAULT_BUFFER_COUNT                   6
#define _DEFAULT_DEQUE_WAITINGTIME              200     /* msec */

#define _FD_DEFAULT     (-1)
#define _FD_MIN         (-1)
#define _FD_MAX         (1<<15) /* 2^15 == 32768 */

#define _DEFAULT_CAP_JPG_QUALITY                95
#define _DEFAULT_CAP_WIDTH                      640
#define _DEFAULT_CAP_HEIGHT                     480
#define _DEFAULT_CAP_COUNT                      1
#define _DEFAULT_CAP_INTERVAL                   0
#define _DEFAULT_CAP_PROVIDE_EXIF               FALSE
#define _DEFAULT_CAP_TPS_NUMERATOR              1
#define _DEFAULT_CAP_TPS_DENOMINATOR            30
#define _DEFAULT_KEEPING_BUFFER                 0
#define _DEFAULT_SCRNL_FOURCC                   GST_MAKE_FOURCC('N','V','1','2')
#define _MAX_TRIAL_WAIT_FRAME                   15
#define _CONTINUOUS_SHOT_MARGIN                 17      /* msec */
#define _PREVIEW_BUFFER_WAIT_TIMEOUT            2000000 /* usec */
#define _DEFAULT_RAW_DATA_FOURCC                "NV12"

/*FIXME*/
#define _THUMBNAIL_WIDTH                        320
#define _THUMBNAIL_HEIGHT                       240
#define _THUMBNAIL_DEFAULT_RATIO                1.34

#define SAFE_FREE_GQUEUE(gqueue) \
	if (gqueue) { \
		g_queue_free(gqueue); \
		gqueue = NULL; \
	}

#define MAKE_FOURCC_FROM_STRING(string) ((guint32)(string[0] | (string[1] << 8) | (string[2] << 16) | (string[3] << 24)))


/* Enumerations */
enum {
	/*signal*/
	SIGNAL_STILL_CAPTURE,

	/*SIGNAL_REGISTER_TROUBLE,*/
	LAST_SIGNAL
};

enum {
	ARG_0,
	/* camera */
	ARG_CAMERA_HIGH_SPEED_FPS,
	ARG_CAMERA_AUTO_FPS,
	ARG_CAMERA_ID,

	/* capture */
	ARG_CAMERA_CAPTURE_FOURCC,
	ARG_CAMERA_CAPTURE_WIDTH,
	ARG_CAMERA_CAPTURE_HEIGHT,
	ARG_CAMERA_CAPTURE_INTERVAL,
	ARG_CAMERA_CAPTURE_COUNT,
	ARG_CAMERA_CAPTURE_JPG_QUALITY,
	ARG_CAMERA_CAPTURE_PROVIDE_EXIF,

	/* etc */
	ARG_VFLIP,
	ARG_HFLIP,
	ARG_NUM,
};

enum {
	VIDEO_IN_MODE_UNKNOWN,
	VIDEO_IN_MODE_PREVIEW,
	VIDEO_IN_MODE_VIDEO,
	VIDEO_IN_MODE_CAPTURE,
};

/* restart preview command */
enum {
	RESTART_PREVIEW_CMD_NORMAL = 0,
	RESTART_PREVIEW_CMD_NUM,
};


static void gst_camerasrc_uri_handler_init (gpointer g_iface, gpointer iface_data);

static guint gst_camerasrc_signals[LAST_SIGNAL] = { 0 };

/* Element template variables */
static GstStaticPadTemplate src_factory =
	GST_STATIC_PAD_TEMPLATE("src",
	                        GST_PAD_SRC,
	                        GST_PAD_ALWAYS,
	                        GST_STATIC_CAPS("video/x-raw,"
	                                        "format = (string) { SN12 }, "
	                                        "width = (int) [ 1, 4096 ], "
	                                        "height = (int) [ 1, 4096 ]; "
	                                        "video/x-raw,"
	                                        "format = (string) { NV12 }, "
	                                        "width = (int) [ 1, 4096 ], "
	                                        "height = (int) [ 1, 4096 ]; "
	                                        "video/x-raw,"
	                                        "format = (string) { SN21 }, "
	                                        "width = (int) [ 1, 4096 ], "
	                                        "height = (int) [ 1, 4096 ]; "
	                                        "video/x-raw,"
	                                        "format = (string) { NV21 }, "
	                                        "width = (int) [ 1, 4096 ], "
	                                        "height = (int) [ 1, 4096 ]; "));

/* Local static functions */
static void gst_camerasrc_set_property(GObject *object, guint prop_id, const GValue *value, GParamSpec *pspec);
static void gst_camerasrc_get_property(GObject *object, guint prop_id, GValue *value, GParamSpec *pspec);

static gboolean gst_camerasrc_negotiate(GstBaseSrc *basesrc);
static gboolean gst_camerasrc_src_start(GstBaseSrc *src);
static gboolean gst_camerasrc_src_stop(GstBaseSrc *src);
static gboolean gst_camerasrc_start(GstCameraSrc *camerasrc);

static GstFlowReturn gst_camerasrc_src_create(GstPushSrc *src, GstBuffer **buffer);
static GstFlowReturn gst_camerasrc_read_capture(GstCameraSrc *camerasrc, GstBuffer **buffer, int command);
static GstFlowReturn gst_camerasrc_read_preview(GstCameraSrc *camerasrc, GstBuffer **buffer);
static GstMemory *gst_camerasrc_get_zero_copy_data(GstCameraSrc *camerasrc, guint32 index, guint32 fourcc);
static void gst_camerasrc_buffer_qbuf(GstCameraSrc *camerasrc, int buffer_index);

static GstStateChangeReturn gst_camerasrc_change_state(GstElement *element, GstStateChange transition);
static GstCaps *gst_camerasrc_get_caps(GstBaseSrc *src, GstCaps *filter);
static gboolean gst_camerasrc_set_caps(GstBaseSrc *src, GstCaps *caps);
static gboolean gst_camerasrc_get_caps_info(GstCameraSrc *camerasrc, GstCaps *caps, guint *size);
static gboolean gst_camerasrc_fill_ctrl_list(GstCameraSrc *camerasrc);
static gboolean gst_camerasrc_empty_ctrl_list(GstCameraSrc *camerasrc);
static void gst_camerasrc_finalize(GObject *object);

static gboolean gst_camerasrc_get_timeinfo(GstCameraSrc *camerasrc, GstBuffer *buffer);
static gboolean gst_camerasrc_capture_start(GstCameraSrc *camerasrc);
static gboolean gst_camerasrc_capture_stop(GstCameraSrc *camerasrc);
static gboolean gst_camerasrc_jpeg_capture(GstCameraSrc *camerasrc, gint buffer_index, GstCameraBuffer *buffer);
static gboolean gst_camerasrc_emit_capture_signal(GstCameraSrc *camerasrc,
                                                  camerasrc_buffer_t *main,
                                                  camerasrc_buffer_t *thumb,
                                                  camerasrc_buffer_t *scrnl);
static gpointer _gst_camerasrc_capture_thread_func(gpointer data);

static GstCameraBuffer *gst_camerasrc_buffer_new(GstCameraSrc *camerasrc);
static void gst_camerasrc_buffer_finalize(GstCameraBuffer *buffer);
static void gst_camerasrc_error_handler(GstCameraSrc *camerasrc, int ret);

/* Util functions */
static unsigned long gst_get_current_time(void);
static gboolean _gst_camerasrc_get_frame_size(int fourcc, int width, int height, unsigned int *outsize);
static gboolean _gst_camerasrc_get_raw_pixel_info(int fourcc, int *pix_format, int *colorspace);
static gboolean _gst_camerasrc_get_normal_buffer(GstCameraSrc *camerasrc, int fourcc,
                                                 unsigned char *base_buf, int width, int height,
                                                 GstMemory **new_buf);
static void _gst_camerasrc_restart_preview(GstCameraSrc *camerasrc);

#if _ENABLE_CAMERASRC_DEBUG
static int __util_write_file(char *filename, void *data, int size);
#endif /* _ENABLE_CAMERASRC_DEBUG */
static void _gst_camerasrc_post_message_int(GstCameraSrc *camerasrc, const char *msg_name, const char *field_name, int value);

GST_IMPLEMENT_CAMERASRC_COLOR_BALANCE_METHODS(GstCameraSrc, gst_camera_src);
GST_IMPLEMENT_CAMERASRC_CONTROL_METHODS(GstCameraSrc, gst_camera_src);


/******************************************************************************
 * Implementations
 *******************************************************************************/
static void gst_camerasrc_error_handler(GstCameraSrc *camerasrc, int ret)
{
	switch (ret) {
	case CAMERASRC_SUCCESS:
		break;
	case CAMERASRC_ERR_IO_CONTROL:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, FAILED, ("IO control error"), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_DEVICE_OPEN:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, OPEN_READ_WRITE, ("camera open failed"), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_DEVICE_BUSY:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, BUSY, ("camera device busy"), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_DEVICE_NOT_FOUND:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, NOT_FOUND, ("camera device not found"), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_DEVICE_UNAVAILABLE:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, OPEN_READ, ("camera device unavailable"), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_DEVICE_WAIT_TIMEOUT:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, TOO_LAZY, (("Timeout[live_buffers=%d]"), camerasrc->num_live_buffers), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_DEVICE_NOT_SUPPORT:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, SETTINGS, ("Not supported"), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_ALLOCATION:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, SETTINGS, ("memory allocation failed"), GST_ERROR_SYSTEM);
		break;
	case CAMERASRC_ERR_SECURITY_SERVICE:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, FAILED, ("Security service failed"), GST_ERROR_SYSTEM);
		break;
	default:
		GST_ELEMENT_ERROR(camerasrc, RESOURCE, SEEK, (("General video device error[ret=%x]"), ret), GST_ERROR_SYSTEM);
		break;
	}

	return;
}


/* VOID:OBJECT,OBJECT,OBJECT (generated by 'glib-genmarshal') */
#define g_marshal_value_peek_pointer(v)  (v)->data[0].v_pointer
#define g_marshal_value_peek_object(v)   (v)->data[0].v_pointer
static void
g_cclosure_user_marshal_VOID__OBJECT_OBJECT_OBJECT (GClosure     *closure,
                                                    GValue       *return_value G_GNUC_UNUSED,
                                                    guint         n_param_values,
                                                    const GValue *param_values,
                                                    gpointer      invocation_hint G_GNUC_UNUSED,
                                                    gpointer      marshal_data)
{
  typedef void (*GMarshalFunc_VOID__OBJECT_OBJECT_OBJECT) (gpointer     data1,
                                                           gpointer     arg_1,
                                                           gpointer     arg_2,
                                                           gpointer     arg_3,
                                                           gpointer     data2);
  register GMarshalFunc_VOID__OBJECT_OBJECT_OBJECT callback;
  register GCClosure *cc = (GCClosure*) closure;
  register gpointer data1, data2;

  g_return_if_fail (n_param_values == 4);

  if (G_CCLOSURE_SWAP_DATA (closure))
    {
      data1 = closure->data;
      data2 = g_value_peek_pointer (param_values + 0);
    }
  else
    {
      data1 = g_value_peek_pointer (param_values + 0);
      data2 = closure->data;
    }
  callback = (GMarshalFunc_VOID__OBJECT_OBJECT_OBJECT) (marshal_data ? marshal_data : cc->callback);

  callback (data1,
            g_marshal_value_peek_object (param_values + 1),
            g_marshal_value_peek_object (param_values + 2),
            g_marshal_value_peek_object (param_values + 3),
            data2);
}

/* use following BOILERPLATE MACRO as _get_type entry */
G_DEFINE_TYPE_WITH_CODE(GstCameraSrc, gst_camerasrc, GST_TYPE_PUSH_SRC,
                        G_IMPLEMENT_INTERFACE(GST_TYPE_URI_HANDLER, gst_camerasrc_uri_handler_init)
                        G_IMPLEMENT_INTERFACE(GST_TYPE_CAMERA_CONTROL, gst_camera_src_control_interface_init)
                        G_IMPLEMENT_INTERFACE(GST_TYPE_COLOR_BALANCE, gst_camera_src_color_balance_interface_init));

static int gst_camerasrc_af_cb(camsrc_handle_t handle, int state, void *usr_param)
{
	GstCameraSrc *camerasrc = (GstCameraSrc *)usr_param;

	_gst_camerasrc_post_message_int(camerasrc, "camerasrc-AF", "focus-state", state);

	return CAMERASRC_SUCCESS;
}


static gboolean gst_camerasrc_create(GstCameraSrc *camerasrc)
{
	int ret = 0;

	/* create capture thread */
	g_mutex_lock(&camerasrc->capture_mutex);

	camerasrc->quit_capture_thread = FALSE;
	camerasrc->capture_thread = g_thread_new("capture_thread", (GThreadFunc)_gst_camerasrc_capture_thread_func, (gpointer)camerasrc);

	g_mutex_unlock(&camerasrc->capture_mutex);

	if (camerasrc->capture_thread == NULL) {
		GST_ERROR("create capture thread failed");
		ret = CAMERASRC_ERR_INTERNAL;
		goto _ERROR;
	}

	/*create handle*/
	GST_INFO("camerasrc_create");
	ret = camerasrc_create(&(camerasrc->v4l2_handle));
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR("camerasrc_create() failed. errcode = 0x%08X", ret);
		goto _ERROR;
	}

	GST_INFO("camerasrc_create() done");

	/*CAMERASRC CAM: realize*/
	GST_INFO("camerasrc_realize");
	ret = camerasrc_realize(camerasrc->v4l2_handle, camerasrc->camera_id, CAMERASRC_SENSOR_MODE_CAMERA);
	if (ret != CAMERASRC_SUCCESS) {
		goto _ERROR;
	}

	/*CAMERASRC CAM: start*/
	GST_INFO("camerasrc_start");
	ret = camerasrc_start(camerasrc->v4l2_handle);
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR("camerasrc_start() failed. errcode = 0x%x", ret);
		goto _ERROR;
	}

	if (!gst_camerasrc_fill_ctrl_list(camerasrc)) {
		GST_WARNING("Can't fill v4l2 control list.");
	}

	return TRUE;

 _ERROR:
	gst_camerasrc_error_handler(camerasrc, ret);

	/* release capture thread */
	if (camerasrc->capture_thread) {
		g_mutex_lock(&camerasrc->capture_mutex);

		camerasrc->quit_capture_thread = TRUE;
		g_cond_signal(&camerasrc->capture_cond);

		g_mutex_unlock(&camerasrc->capture_mutex);

		g_thread_join(camerasrc->capture_thread);

		camerasrc->capture_thread = NULL;
	}

	return FALSE;
}


static gboolean gst_camerasrc_destroy(GstCameraSrc *camerasrc)
{
	GST_INFO_OBJECT (camerasrc, "ENTERED");

	if (camerasrc->v4l2_handle) {
		/*Empty control list */
		gst_camerasrc_empty_ctrl_list(camerasrc);

		/*CAMERASRC CAM: stop stream*/
		/*CAMERASRC CAM: unrealize*/
		GST_INFO_OBJECT(camerasrc, "camerasrc_unrealize() calling...");
		camerasrc_unrealize(camerasrc->v4l2_handle);

		/*CAMERASRC CAM: destroy*/
		GST_INFO_OBJECT(camerasrc, "camerasrc_destroy() calling...");
		camerasrc_destroy(camerasrc->v4l2_handle);
		camerasrc->v4l2_handle = NULL;
		GST_INFO_OBJECT(camerasrc, "AV cam destroyed.");
		camerasrc->mode = VIDEO_IN_MODE_UNKNOWN;
	}

	/* release capture thread */
	if (camerasrc->capture_thread) {
		g_mutex_lock(&camerasrc->capture_mutex);

		camerasrc->quit_capture_thread = TRUE;
		g_cond_signal(&camerasrc->capture_cond);

		g_mutex_unlock(&camerasrc->capture_mutex);

		g_thread_join(camerasrc->capture_thread);

		camerasrc->capture_thread = NULL;
	}

	GST_INFO_OBJECT(camerasrc, "LEAVED");

	return TRUE;
}


static gboolean gst_camerasrc_fill_ctrl_list(GstCameraSrc *camerasrc)
{
	int n = 0;
	camerasrc_ctrl_info_t ctrl_info;

	char *camerasrc_ctrl_label[CAMERASRC_CTRL_NUM] =
		{
		"brightness",               /**< label for CAMERASRC_CTRL_BRIGHTNESS */
		"contrast",                 /**< label for CAMERASRC_CTRL_CONTRAST */
		"digital zoom",             /**< label for CAMERASRC_CTRL_DIGITAL_ZOOM */
		"optical zoom",             /**< label for CAMERASRC_CTRL_OPTICAL_ZOOM */
		"white balance",            /**< label for CAMERASRC_CTRL_WHITE_BALANCE */
		"color tone",               /**< label for CAMERASRC_CTRL_COLOR_TONE */
		"program mode",             /**< label for CAMERASRC_CTRL_PROGRAM_MODE */
		"flip",                     /**< label for CAMERASRC_CTRL_FLIP */
		"anti handshake",           /**< label for CAMERASRC_CTRL_ANTI_HANDSHAKE */
		"wide dynamic range",       /**< label for CAMERASRC_CTRL_WIDE_DYNAMIC_RANGE */
		"saturation",               /**< label for CAMERASRC_CTRL_SATURATION */
		"sharpness",                /**< label for CAMERASRC_CTRL_SHARPNESS */
		"iso",                      /**< label for CAMERASRC_CTRL_ISO */
		"photometry",               /**< label for CAMERASRC_CTRL_PHOTOMETRY */
		};

	g_return_val_if_fail(camerasrc, FALSE);
	g_return_val_if_fail(camerasrc->v4l2_handle, FALSE);

	GST_DEBUG_OBJECT(camerasrc, "ENTERED");

	for (n = CAMERASRC_CTRL_BRIGHTNESS ; n < CAMERASRC_CTRL_NUM ; n++) {
		GstCameraSrcColorBalanceChannel *camerasrc_color_channel = NULL;
		GstColorBalanceChannel *color_channel = NULL;

		GstCamerasrcControlChannel *camerasrc_control_channel = NULL;
		GstCameraControlChannel *control_channel = NULL;

		gint channel_type;

		memset(&ctrl_info, 0x0, sizeof(camerasrc_ctrl_info_t));
		/*TODO: queryctrl */

		switch (n) {
			case CAMERASRC_CTRL_BRIGHTNESS:
			case CAMERASRC_CTRL_CONTRAST:
			case CAMERASRC_CTRL_WHITE_BALANCE:
			case CAMERASRC_CTRL_COLOR_TONE:
			case CAMERASRC_CTRL_SATURATION:
			case CAMERASRC_CTRL_SHARPNESS:
				channel_type = INTERFACE_COLOR_BALANCE;
				break;
			case CAMERASRC_CTRL_DIGITAL_ZOOM:
			case CAMERASRC_CTRL_OPTICAL_ZOOM:
			case CAMERASRC_CTRL_PROGRAM_MODE:
			case CAMERASRC_CTRL_FLIP:
			case CAMERASRC_CTRL_ANTI_HANDSHAKE:
			case CAMERASRC_CTRL_WIDE_DYNAMIC_RANGE:
				channel_type = INTERFACE_CAMERA_CONTROL;
				break;
			default:
				channel_type = INTERFACE_NONE;
				continue;
		}

		if (channel_type == INTERFACE_COLOR_BALANCE) {
			camerasrc_color_channel = g_object_new(GST_TYPE_CAMERASRC_COLOR_BALANCE_CHANNEL, NULL);
			color_channel = GST_COLOR_BALANCE_CHANNEL(camerasrc_color_channel);

			color_channel->label = g_strdup((const gchar *)camerasrc_ctrl_label[n]);
			camerasrc_color_channel->id = n;
			color_channel->min_value = ctrl_info.min;
			color_channel->max_value = ctrl_info.max;

			camerasrc->colors = g_list_append(camerasrc->colors, (gpointer)color_channel);
			GST_INFO_OBJECT(camerasrc, "Adding Color Balance Channel %s (%x)",
			                           color_channel->label, camerasrc_color_channel->id);
		} else { /* if( channel_type == INTERFACE_CAMERA_CONTROL ) */
			camerasrc_control_channel = g_object_new(GST_TYPE_CAMERASRC_CONTROL_CHANNEL, NULL);
			control_channel = GST_CAMERA_CONTROL_CHANNEL(camerasrc_control_channel);

			control_channel->label = g_strdup((const gchar *)camerasrc_ctrl_label[n]);
			camerasrc_control_channel->id = n;
			control_channel->min_value = ctrl_info.min;
			control_channel->max_value = ctrl_info.max;

			camerasrc->camera_controls = g_list_append(camerasrc->camera_controls, (gpointer)control_channel);
			GST_INFO_OBJECT(camerasrc, "Adding Camera Control Channel %s (%x)",
			                           control_channel->label, camerasrc_control_channel->id);
		}
	}

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return TRUE;
}


static gboolean gst_camerasrc_empty_ctrl_list(GstCameraSrc *camerasrc)
{
	g_return_val_if_fail(camerasrc, FALSE);

	GST_DEBUG_OBJECT (camerasrc, "ENTERED");

	g_list_foreach(camerasrc->colors, (GFunc)g_object_unref, NULL);
	g_list_free(camerasrc->colors);
	camerasrc->colors = NULL;

	g_list_foreach(camerasrc->camera_controls, (GFunc)g_object_unref, NULL);
	g_list_free(camerasrc->camera_controls);
	camerasrc->camera_controls = NULL;

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return TRUE;
}


static gboolean gst_camerasrc_start(GstCameraSrc *camerasrc)
{
	int ret = 0;

	camerasrc_format_t fmt;
	camerasrc_frac_t frac;

	GST_DEBUG_OBJECT(camerasrc, "ENTERED");

#ifdef _SPEED_UP_RAW_CAPTURE
	/* check if from no stream change capture */
	if (camerasrc->mode == VIDEO_IN_MODE_CAPTURE &&
	    camerasrc->cap_stream_diff == FALSE) {
		GST_INFO("keep current stream for preview");
		goto _READY_DONE;
	}

	camerasrc->cap_stream_diff = FALSE;
#endif

	/*CAMERASRC CAM: callback*/
	camerasrc_set_focused_callback(camerasrc->v4l2_handle, gst_camerasrc_af_cb, camerasrc);

	CLEAR(fmt);
	CLEAR(frac);

	/*CAMERASRC CAM: format*/
	fmt.pix_format = camerasrc->pix_format;
	fmt.colorspace = camerasrc->colorspace;
	fmt.rotation = camerasrc->rotate;
	fmt.quality = camerasrc->cap_jpg_quality;

	/*CAMERASRC CAM: set resolution - Do not care about rotation */
	/* set original preview size */
	CAMERASRC_SET_SIZE_BY_DIMENSION(fmt, camerasrc->width, camerasrc->height);

	/*CAMERASRC CAM: set capture resolution - this is effective if fourcc is ITLV */
	CAMERASRC_SET_CAPTURE_SIZE_BY_DIMENSION(fmt, camerasrc->cap_width, camerasrc->cap_height);

	/*CAMERASRC CAM: set format*/
	GST_INFO_OBJECT(camerasrc, "pix [%dx%d] fps %d, format %d, colorspace %d, rotation %d",
	                           camerasrc->width, camerasrc->height, camerasrc->fps,
	                           fmt.pix_format, fmt.colorspace, fmt.rotation);
	ret = camerasrc_set_format(camerasrc->v4l2_handle, &fmt);
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT (camerasrc, "camerasrc_set_format() failed. errcode = 0x%08X", ret);
		goto _ERROR;
	}

	/*CAMERASRC CAM: set fps*/
	if (camerasrc->fps_auto) {
		/*if fps is zero, auto fps mode*/
		frac.numerator = 0;
		frac.denominator = 1;
		GST_INFO_OBJECT (camerasrc, "FPS auto(%d)", camerasrc->fps_auto);
	} else if (camerasrc->high_speed_fps <= 0) {
		if (camerasrc->fps <= 0) {
			/*if fps is zero, auto fps mode*/
			frac.numerator   = 0;
			frac.denominator = 1;
		} else {
			frac.numerator   = 1;
			frac.denominator = camerasrc->fps;
		}
	} else {
		GST_INFO_OBJECT(camerasrc, "high speed recording(%d)", camerasrc->high_speed_fps);
		frac.numerator = 1;
		frac.denominator = camerasrc->high_speed_fps;
	}

	ret = camerasrc_set_timeperframe(camerasrc->v4l2_handle, &frac);
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT(camerasrc, "camerasrc_set_timeperframe() failed. errcode = 0x%x", ret);
		goto _ERROR;
	}

	GST_INFO_OBJECT (camerasrc, "camerasrc_set_timeperframe() done");

	/*CAMERASRC CAM: Set flip*/
	camerasrc_set_vflip(camerasrc->v4l2_handle, camerasrc->vflip);
	camerasrc_set_hflip(camerasrc->v4l2_handle, camerasrc->hflip);

	GST_INFO("VFLIP : %d, HFLIP : %d", camerasrc->vflip, camerasrc->hflip);

	ret = camerasrc_get_num_buffer(camerasrc->v4l2_handle, &(camerasrc->buffer_count));
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT(camerasrc, "camerasrc_get_num_buffer() failed. errcode = 0x%x", ret);
		goto _ERROR;
	}

	GST_INFO_OBJECT(camerasrc, "buffer number %d", camerasrc->buffer_count);

	if (!camerasrc_tbm_init(camerasrc->v4l2_handle)) {
		GST_ERROR_OBJECT(camerasrc, "camerasrc_tbm_init() failed");
		goto _ERROR;
	}

	/* create buffer */
	ret = camerasrc_create_buffer(camerasrc->v4l2_handle);
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT(camerasrc, "camerasrc_create_buffer() failed. errcode = 0x%x", ret);
		camerasrc->buffer_running = FALSE;
		goto _ERROR;
	}

	/*CAMERASRC CAM: start video preview*/
	GST_INFO("camerasrc_start_preview_stream");
	ret = camerasrc_start_preview_stream(camerasrc->v4l2_handle);
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT(camerasrc, "camerasrc_start_preview_stream() failed. errcode = 0x%x", ret);
		camerasrc->buffer_running = FALSE;
		goto _ERROR;
	}

	GST_INFO_OBJECT(camerasrc, "camerasrc_start_preview_stream() done");

	camerasrc->num_live_buffers = 0;

_READY_DONE:
	camerasrc->mode = VIDEO_IN_MODE_PREVIEW;
	camerasrc->current_buffer_data_index = (camerasrc->current_buffer_data_index + 1)%10;
	camerasrc->buffer_running = TRUE;

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return TRUE;

_ERROR:
	gst_camerasrc_error_handler(camerasrc, ret);

	/* Stop stream and release GEM */
	camerasrc_stop_stream(camerasrc->v4l2_handle);
	camerasrc_destroy_buffer(camerasrc->v4l2_handle);

	camerasrc_tbm_deinit(camerasrc->v4l2_handle);

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return FALSE;
}


static gboolean gst_camerasrc_stop(GstCameraSrc *camerasrc)
{
	GST_DEBUG_OBJECT (camerasrc, "ENTERED");

	if (camerasrc->v4l2_handle) {
		/* CAMERASRC CAM: stop stream */
		/* To guarantee buffers are valid before finishing */
		GMutex *lock_mutex = NULL;
		int keeping_buffer_num = 0;

		lock_mutex = &camerasrc->buffer_lock;

		keeping_buffer_num = _DEFAULT_KEEPING_BUFFER;

		g_mutex_lock(lock_mutex);

		while (camerasrc->num_live_buffers > keeping_buffer_num) {
			gint64 end_time;

			GST_INFO_OBJECT(camerasrc, "Wait until all live buffers are relased. (Tot=%d, Live=%d)",
			                           camerasrc->buffer_count, camerasrc->num_live_buffers);

			end_time = g_get_monotonic_time () + _PREVIEW_BUFFER_WAIT_TIMEOUT;

			if (!g_cond_wait_until(&camerasrc->buffer_cond, lock_mutex, end_time)) {
				GST_ERROR_OBJECT(camerasrc, "Buffer wait timeout[%d usec].(Live=%d) Skip waiting...",
				                            _PREVIEW_BUFFER_WAIT_TIMEOUT, camerasrc->num_live_buffers);
				break;
			} else {
				GST_INFO_OBJECT(camerasrc, "Signal received.");
			}
		}

		GST_INFO_OBJECT(camerasrc, "Waiting free buffer finished. (Live=%d)", camerasrc->num_live_buffers);

		g_mutex_unlock(lock_mutex);

		camerasrc_stop_stream(camerasrc->v4l2_handle);

		camerasrc_destroy_buffer(camerasrc->v4l2_handle);

		camerasrc->buffer_running = FALSE;
		camerasrc->mode = VIDEO_IN_MODE_UNKNOWN;
	}

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return TRUE;
}


static gboolean gst_camerasrc_capture_start(GstCameraSrc *camerasrc)
{
	/*CAMERASRC CAM*/
	int ret = 0;
	char *pfourcc = NULL;
	camerasrc_format_t fmt;
	camerasrc_frac_t frac;

	GST_INFO_OBJECT(camerasrc, "ENTERED");

	if (camerasrc->mode == VIDEO_IN_MODE_PREVIEW) {
		/* To guarantee buffers are valid before finishing. */
		GST_INFO("wait for buffer in gst_camerasrc_capture_start");

		g_mutex_lock(&camerasrc->buffer_lock);

		while (camerasrc->num_live_buffers > _DEFAULT_KEEPING_BUFFER) {
			gint64 end_time;

			GST_INFO_OBJECT(camerasrc, "Wait until all live buffers are relased. (Tot=%d, Live=%d)",
			                           camerasrc->buffer_count, camerasrc->num_live_buffers);

			end_time = g_get_monotonic_time() + _PREVIEW_BUFFER_WAIT_TIMEOUT;

			if (!g_cond_wait_until(&camerasrc->buffer_cond, &camerasrc->buffer_lock, end_time)) {
				GST_ERROR_OBJECT(camerasrc, "Buffer wait timeout[%d usec].(Live=%d) Skip waiting...",
				                            _PREVIEW_BUFFER_WAIT_TIMEOUT, camerasrc->num_live_buffers);
				break;
			} else {
				GST_INFO_OBJECT(camerasrc, "Signal received.");
			}
		}

		GST_INFO_OBJECT(camerasrc, "Waiting free buffer is finished. (Live=%d)", camerasrc->num_live_buffers);

		g_mutex_unlock(&camerasrc->buffer_lock);

#ifdef _SPEED_UP_RAW_CAPTURE
		/* Skip restart stream if format/width/height are all same */
		if (MAKE_FOURCC_FROM_STRING(camerasrc->format_name) == camerasrc->cap_fourcc &&
		    camerasrc->width == camerasrc->cap_width &&
		    camerasrc->height == camerasrc->cap_height) {
			GST_INFO_OBJECT(camerasrc, "fourcc, width and height is same. skip restart stream...");
			goto _CAPTURE_READY_DONE;
		}

		camerasrc->cap_stream_diff = TRUE;
#endif

		/*CAMERASRC CAM: stop stream*/
		GST_INFO_OBJECT(camerasrc, "            camerasrc_stop_stream in gst_camerasrc_capture_start");

		camerasrc_stop_stream(camerasrc->v4l2_handle);

		camerasrc_destroy_buffer(camerasrc->v4l2_handle);

		GST_INFO_OBJECT (camerasrc, "camerasrc_stop_stream() done");
		camerasrc->buffer_running = FALSE;

		pfourcc = (char*)&camerasrc->cap_fourcc;
		GST_INFO_OBJECT(camerasrc, "CAPTURE: Size[%dx%d], fourcc(%c%c%c%c) quality[%d] interval[%d] count[%d]",
		                           camerasrc->cap_width, camerasrc->cap_height,
		                           pfourcc[0], pfourcc[1], pfourcc[2], pfourcc[3],
		                           camerasrc->cap_jpg_quality, camerasrc->cap_interval, camerasrc->cap_count);

		/**
		 * START STILL CAPTURE
		 */
		memset(&fmt, 0x00, sizeof (camerasrc_format_t));

		/*CAMERASRC CAM: set format*/
		CAMERASRC_SET_SIZE_BY_DIMENSION(fmt, camerasrc->cap_width, camerasrc->cap_height);

		/*CAMERASRC CAM: set capture resolution - this is effective if fourcc is ITLV */
		CAMERASRC_SET_CAPTURE_SIZE_BY_DIMENSION(fmt, camerasrc->cap_width, camerasrc->cap_height);

		_gst_camerasrc_get_raw_pixel_info(camerasrc->cap_fourcc, &(fmt.pix_format), &(fmt.colorspace));
		fmt.rotation = 0;

		if (camerasrc->cap_fourcc == GST_MAKE_FOURCC('J', 'P', 'E', 'G')) {
			fmt.quality = camerasrc->cap_jpg_quality;
		}

		/*CAMERASRC CAM: format*/
		ret = camerasrc_set_format(camerasrc->v4l2_handle, &fmt);
		if (ret != CAMERASRC_SUCCESS) {
			GST_ERROR_OBJECT(camerasrc, "camerasrc_set_format() failed. errcode = 0x%x", ret);
			goto _ERROR;
		}

		GST_INFO_OBJECT(camerasrc, "camerasrc_set_format done");

		/* Set TimePerFrame */
		frac.numerator   = _DEFAULT_CAP_TPS_NUMERATOR;
		if (camerasrc->cap_width > _DEFAULT_CAP_WIDTH) {
			frac.denominator = 10;
		} else {
			frac.denominator = _DEFAULT_CAP_TPS_DENOMINATOR;
		}

		ret = camerasrc_set_timeperframe(camerasrc->v4l2_handle, &frac);
		if (ret != CAMERASRC_SUCCESS) {
			GST_ERROR_OBJECT(camerasrc, "camerasrc_set_timeperframe() failed. errcode = 0x%x", ret);
			goto _ERROR;
		}

		GST_INFO_OBJECT(camerasrc, "camerasrc_set_timeperframe done");

		/*CAMERASRC CAM: start stream*/
		GST_INFO_OBJECT(camerasrc, "camerasrc_start_preview_stream");
		ret = camerasrc_start_preview_stream(camerasrc->v4l2_handle);
		if (ret != CAMERASRC_SUCCESS) {
			GST_ERROR_OBJECT(camerasrc, "camerasrc_start_still_stream() failed. errcode = 0x%x", ret);
			goto _ERROR;
		}

		GST_INFO_OBJECT(camerasrc, "camerasrc_start_still_stream done");

		camerasrc->buffer_running = TRUE;

_CAPTURE_READY_DONE:

		g_mutex_lock(&camerasrc->jpg_mutex);
		camerasrc->cap_next_time = gst_get_current_time();
		g_mutex_unlock(&camerasrc->jpg_mutex);
		camerasrc->cap_count_current = 0;

		/* end change to capture mode*/
		camerasrc->mode = VIDEO_IN_MODE_CAPTURE;

		GST_INFO_OBJECT(camerasrc, "CAPTURE STARTED!");
	} else {
		GST_WARNING_OBJECT(camerasrc, "Wrong state[%d]!", camerasrc->mode);
	}

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return TRUE;

_ERROR:
	gst_camerasrc_error_handler(camerasrc, ret);

	return FALSE;
}


static gboolean gst_camerasrc_capture_stop(GstCameraSrc *camerasrc)
{
	GST_DEBUG_OBJECT(camerasrc, "ENTERED");

	if (camerasrc->mode == VIDEO_IN_MODE_CAPTURE) {
		/* To guarantee buffers are valid before finishing */
		GMutex *lock_mutex = NULL;
		int keeping_buffer_num = 0;

		/* keep current stream for preview */
		if (strcmp(camerasrc->format_name, "ITLV") == 0 &&
		    camerasrc->cap_fourcc == GST_MAKE_FOURCC('I','T','L','V')) {
			GST_INFO("ITLV capture mode");
			return TRUE;
		}

		lock_mutex = &camerasrc->buffer_lock;

		keeping_buffer_num = _DEFAULT_KEEPING_BUFFER;

		g_mutex_lock(lock_mutex);

		while (camerasrc->num_live_buffers > keeping_buffer_num) {
			gint64 end_time;

			GST_INFO_OBJECT(camerasrc, "Wait until all live buffers are relased. (Tot=%d, Live=%d)",
			                           camerasrc->buffer_count, camerasrc->num_live_buffers);

			end_time = g_get_monotonic_time () + _PREVIEW_BUFFER_WAIT_TIMEOUT;

			if (!g_cond_wait_until(&camerasrc->buffer_cond, lock_mutex, end_time)) {
				GST_ERROR_OBJECT(camerasrc, "Buffer wait timeout[%d usec].(Live=%d) Skip waiting...",
				                            _PREVIEW_BUFFER_WAIT_TIMEOUT, camerasrc->num_live_buffers);
				break;
			} else {
				GST_INFO_OBJECT(camerasrc, "Signal received.");
			}
		}

		GST_INFO_OBJECT(camerasrc, "Waiting free buffer finished. (Live=%d)", camerasrc->num_live_buffers);

		g_mutex_unlock(lock_mutex);

#ifdef _SPEED_UP_RAW_CAPTURE
		if (camerasrc->cap_stream_diff) {
			/*CAMERASRC CAM: stop stream*/
			camerasrc_stop_stream(camerasrc->v4l2_handle);
			camerasrc->buffer_running = FALSE;

			GST_INFO_OBJECT(camerasrc, "camerasrc_stop_stream() done");
		} else {
			GST_INFO_OBJECT(camerasrc, "no need to stop stream(capture format==preview format)");
		}
#else
		/*CAMERASRC CAM: stop stream*/
		camerasrc_stop_stream(camerasrc->v4l2_handle);
		camerasrc->buffer_running = FALSE;

		GST_INFO_OBJECT(camerasrc, "camerasrc_stop_stream() done");
#endif
		GST_INFO_OBJECT(camerasrc, "CAPTURE STOPPED!");
	}

	GST_DEBUG_OBJECT (camerasrc, "LEAVED");

	return TRUE;
}


static gpointer _gst_camerasrc_capture_thread_func(gpointer data)
{
	int ret = FALSE;
	GstCameraSrc *camerasrc = (GstCameraSrc *)data;
	camerasrc_capture_data_info *capture_data = NULL;

	/* buffers for captured data */
	camerasrc_buffer_t main_buf;
	camerasrc_buffer_t thumb_buf;
	camerasrc_buffer_t scrnl_buf;

	if (camerasrc == NULL) {
		GST_ERROR("camerasrc is NULL");
		return NULL;
	}

	memset(&main_buf, 0x0, sizeof(camerasrc_buffer_t));
	memset(&thumb_buf, 0x0, sizeof(camerasrc_buffer_t));
	memset(&scrnl_buf, 0x0, sizeof(camerasrc_buffer_t));

	GST_INFO("start capture thread");

	while (!camerasrc->quit_capture_thread) {
		g_mutex_lock(&camerasrc->capture_mutex);

		if (g_queue_is_empty(camerasrc->capture_buffer_list)) {
			GST_INFO("buffer list is empty. wait capture signal...");
			g_cond_wait(&camerasrc->capture_cond, &camerasrc->capture_mutex);
			GST_INFO("capture signal received");

			g_mutex_unlock(&camerasrc->capture_mutex);
			continue;
		}

		capture_data = (camerasrc_capture_data_info *)g_queue_pop_head(camerasrc->capture_buffer_list);
		GST_INFO("pop capture_data %p", capture_data);

		g_mutex_unlock(&camerasrc->capture_mutex);

		if (capture_data == NULL) {
			GST_WARNING("capture_data is NULL. skip this...");
			continue;
		}

		GST_INFO("gst_camerasrc_emit_capture_signal");

		ret = gst_camerasrc_emit_capture_signal(camerasrc, &main_buf, &thumb_buf, &scrnl_buf);

		GST_INFO("Zero Shutter Lag Capture Done - QBUF index %d, ret %d", capture_data->buffer_index, ret);

		gst_camerasrc_buffer_qbuf(camerasrc, capture_data->buffer_index);

		/* free allocated memory
		   - main, thumbnail buffer and capture_data
		   - screennail buffer is NOT allocated memory */
		if (main_buf.planes[0].start) {
			free(main_buf.planes[0].start);
			main_buf.planes[0].start = NULL;
			main_buf.planes[0].length = 0;
		}
		if (thumb_buf.planes[0].start) {
			free(thumb_buf.planes[0].start);
			thumb_buf.planes[0].start = NULL;
			thumb_buf.planes[0].length = 0;
		}

		free(capture_data);
		capture_data = NULL;
	}

	GST_INFO("quit while loop and clear buffer list...");

	/* clear buffer list */
	g_mutex_lock(&camerasrc->capture_mutex);
	while (!g_queue_is_empty(camerasrc->capture_buffer_list)) {
		capture_data = (camerasrc_capture_data_info *)g_queue_pop_head(camerasrc->capture_buffer_list);
		if (capture_data) {
			gst_camerasrc_buffer_qbuf(camerasrc, capture_data->buffer_index);
			free(capture_data);
			capture_data = NULL;
		} else {
			GST_WARNING("capture_data is NULL");
		}
	}
	g_mutex_unlock(&camerasrc->capture_mutex);

	GST_INFO("quit capture thread");

	return NULL;
}


static GstFlowReturn gst_camerasrc_read_preview(GstCameraSrc *camerasrc, GstBuffer **buffer)
{
	int ret = 0;
	int v4l2_buffer_index = 0;
	int i = 0;
	GstMemory *mem_data = NULL; /* for virtual address */
	GstMemory *mem_zc_data = NULL; /* for zero copy data */
	GstMemory *mem_camerabuf = NULL; /* for camerasrc buffer */
	camerasrc_buffer_t main_buf;
	GstCameraBuffer *vid_buf = NULL;

	/* alloc main buffer */
	vid_buf = gst_camerasrc_buffer_new(camerasrc);

	/* check restart preview command */
	if (!g_queue_is_empty(camerasrc->restart_cmd_list)) {
		int restart_cmd = (int)g_queue_pop_head(camerasrc->restart_cmd_list);

		GST_INFO("popped cmd : %d", restart_cmd);

		switch (restart_cmd) {
		case RESTART_PREVIEW_CMD_NORMAL:
			_gst_camerasrc_restart_preview(camerasrc);

			/* send signal for restart waiting thread */
			g_mutex_lock(&camerasrc->restart_mutex);
			g_cond_signal(&camerasrc->restart_cond);
			GST_INFO("send restart completed signal");
			g_mutex_unlock(&camerasrc->restart_mutex);
			break;
		default:
			GST_WARNING("unknown cmd %d", restart_cmd);
			break;
		}
	}

	GST_LOG_OBJECT(camerasrc, "start SELECT call");

	for (i = 0 ; i < _MAX_TRIAL_WAIT_FRAME ; i++) {
		/* Wait frame */
		//ret = camerasrc_wait_frame_available(camerasrc->v4l2_handle, _DEFAULT_DEQUE_WAITINGTIME);
		ret = CAMERASRC_SUCCESS;
		if (ret != CAMERASRC_SUCCESS) {
			if (ret == CAMERASRC_ERR_DEVICE_WAIT_TIMEOUT && i < (_MAX_TRIAL_WAIT_FRAME - 1)) {
				/* wait until any live buffer is finalized if all buffers are LIVE */
				g_mutex_lock(&camerasrc->buffer_lock);

				GST_WARNING_OBJECT(camerasrc, "SELECT TIMEOUT!!! Retry..(live %d)", camerasrc->num_live_buffers);

				if (camerasrc->buffer_count <= camerasrc->num_live_buffers) {
					gint64 end_time;

					GST_INFO("All buffers are LIVE. wait buffer finalize...");

					end_time = g_get_monotonic_time () + _PREVIEW_BUFFER_WAIT_TIMEOUT;

					if (!g_cond_wait_until(&camerasrc->buffer_cond, &camerasrc->buffer_lock, end_time)) {
						GST_ERROR_OBJECT(camerasrc, "Buffer wait timeout[%d usec]. Return ERROR", _PREVIEW_BUFFER_WAIT_TIMEOUT);
						return GST_FLOW_ERROR;
					} else {
						GST_INFO("Signal received. Retry...");
					}
				}

				GST_INFO_OBJECT(camerasrc, "buffer wait done. current live %d", camerasrc->num_live_buffers);

				g_mutex_unlock(&camerasrc->buffer_lock);

				/* retry waiting */
				continue;
			}

			if (ret == CAMERASRC_ERR_DEVICE_UNAVAILABLE) {
				GST_ERROR_OBJECT(camerasrc,  "register trouble error!! [%x]", ret);
				/*g_signal_emit (G_OBJECT (camerasrc), gst_camerasrc_signals[SIGNAL_REGISTER_TROUBLE], (GQuark)NULL);*/
				gst_camerasrc_error_handler(camerasrc, ret);

				return GST_FLOW_ERROR;
			} else if (ret == CAMERASRC_ERR_INVALID_STATE && (i < _MAX_TRIAL_WAIT_FRAME - 1)) {
				GST_WARNING_OBJECT(camerasrc, "try again...");
			} else {
				GST_ERROR_OBJECT(camerasrc, "Frame waiting error[%x]", ret);
				gst_camerasrc_error_handler(camerasrc, ret);

				return GST_FLOW_ERROR;
			}
		} else {
			GST_LOG_OBJECT(camerasrc, "select success, do DQBUF");
			break;
		}
	}

	/* Buffer DQ */
	GST_DEBUG_OBJECT(camerasrc, "camerasrc_dequeue_buffer");
	ret = camerasrc_dequeue_buffer(camerasrc->v4l2_handle, &v4l2_buffer_index, &main_buf, NULL);
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT(camerasrc, "Dequeue frame error[%x]", ret);
		gst_camerasrc_error_handler(camerasrc, ret);

		return GST_FLOW_ERROR;
	}

	g_mutex_lock(&camerasrc->buffer_lock);

	camerasrc->num_live_buffers++;

	GST_DEBUG_OBJECT(camerasrc, "after : DQBUF (index %d, live bufs %d)",
	                            v4l2_buffer_index, camerasrc->num_live_buffers);

	g_mutex_unlock(&camerasrc->buffer_lock);

	/* set timestamp and duration */
	gst_camerasrc_get_timeinfo(camerasrc, vid_buf->buffer);

	/* set buffer index */
	vid_buf->v4l2_buffer_index = v4l2_buffer_index;

	if (strcmp(camerasrc->format_name, "SN12") == 0 ||
	    strcmp(camerasrc->format_name, "SN21") == 0 ||
	    strcmp(camerasrc->format_name, "SUYV") == 0 ||
	    strcmp(camerasrc->format_name, "SUY2") == 0 ||
	    strcmp(camerasrc->format_name, "SYVY") == 0 ||
	    strcmp(camerasrc->format_name, "S420") == 0) {
		/* for physical address */
		mem_zc_data = gst_camerasrc_get_zero_copy_data(camerasrc,
		                                               v4l2_buffer_index,
		                                               MAKE_FOURCC_FROM_STRING(camerasrc->format_name));
		if (mem_zc_data == NULL) {
			GST_ERROR_OBJECT(camerasrc, "failed to get zero copy data");
			ret = CAMERASRC_ERR_INTERNAL;
			goto PREVIEW_ERROR;
		}

		/* for virtual address - GST_BUFFER_DATA and SIZE */
		mem_data = gst_memory_new_wrapped(0,
		                                  main_buf.planes[0].start,
		                                  main_buf.planes[0].length,
		                                  0,
		                                  main_buf.planes[0].length,
		                                  NULL,
		                                  NULL);
	} else if (strcmp(camerasrc->format_name, "NV12") == 0 ||
	           strcmp(camerasrc->format_name, "NV21") == 0 ||
	           strcmp(camerasrc->format_name, "I420") == 0 ||
	           strcmp(camerasrc->format_name, "YV12") == 0) {
		ret = _gst_camerasrc_get_normal_buffer(camerasrc,
		                                       MAKE_FOURCC_FROM_STRING(camerasrc->format_name),
		                                       main_buf.planes[0].start,
		                                       camerasrc->width, camerasrc->height,
		                                       &mem_data);
		if (ret == FALSE) {
			GST_ERROR_OBJECT(camerasrc, "_gst_camerasrc_get_normal_buffer failed");
		}
	} else {
		mem_data = gst_memory_new_wrapped(0,
		                                  main_buf.planes[0].start,
		                                  main_buf.planes[0].length,
		                                  0,
		                                  main_buf.planes[0].length,
		                                  NULL,
		                                  NULL);
	}

	/* set data memory */
	if (mem_data) {
		gst_buffer_append_memory(vid_buf->buffer, mem_data);
	} else {
		if (mem_zc_data) {
			gst_memory_unref(mem_zc_data);
			mem_zc_data = NULL;
		}

		GST_ERROR_OBJECT(camerasrc, "no mem_data");
		ret = CAMERASRC_ERR_INTERNAL;
		goto PREVIEW_ERROR;
	}

	/* set zero copy memory */
	if (mem_zc_data) {
		gst_buffer_append_memory(vid_buf->buffer, mem_zc_data);
	}

	mem_camerabuf = gst_memory_new_wrapped(GST_MEMORY_FLAG_NOT_MAPPABLE,
	                                       vid_buf,
	                                       sizeof(*vid_buf),
	                                       0,
	                                       sizeof(*vid_buf),
	                                       vid_buf,
	                                       (GDestroyNotify)gst_camerasrc_buffer_finalize);

	/* set GstCameraBuffer memory */
	if (mem_camerabuf) {
		gst_buffer_append_memory(vid_buf->buffer, mem_camerabuf);
	} else {
		GST_ERROR_OBJECT(camerasrc, "mem_camerabuf failed");
		ret = CAMERASRC_ERR_INTERNAL;
		goto PREVIEW_ERROR;
	}

	*buffer = vid_buf->buffer;

	if (camerasrc->firsttime) {
		camerasrc->firsttime = FALSE;
	}

	/*GST_DEBUG_OBJECT(camerasrc, "refcount: %d", GST_OBJECT_REFCOUNT(*buffer));*/

	return GST_FLOW_OK;

PREVIEW_ERROR:
	if (vid_buf) {
		gst_camerasrc_buffer_finalize(vid_buf);
		gst_buffer_unref((GstBuffer *)vid_buf);
		vid_buf = NULL;
	}

	gst_camerasrc_error_handler(camerasrc, ret);

	return GST_FLOW_ERROR;
}


static GstFlowReturn gst_camerasrc_read_capture(GstCameraSrc *camerasrc, GstBuffer **buffer, int command)
{
	int ret;
	int buffer_index = 0;
	unsigned long cur_time;
	gboolean is_jpeg = FALSE;

	static gboolean get_stop_command = FALSE;
	static gboolean get_stop_multi_command = FALSE;

	GstCameraBuffer *buf = NULL;            /*output buffer for preview*/
	GstMapInfo buf_info = GST_MAP_INFO_INIT; /*Output info */
	GstBuffer *gst_buffer = NULL;
	GstMemory *buf_memory = NULL;           /*Output memory*/
	GstSample *buf_sample1 = NULL;
	GstSample *buf_sample2 = NULL;
	GstSample *buf_sample3 = NULL;
	GstCaps *sample_caps = NULL;
	const gchar *string_fourcc = NULL;;

	camerasrc_buffer_t main_buf;            /*original size buffer*/
	camerasrc_buffer_t thumb_buf;           /*thumbnail size buffer*/
	camerasrc_buffer_t scrnl_buf;           /*screennail size buffer*/

	GST_DEBUG_OBJECT(camerasrc, "ENTERED. Command[%d]", command);

	GST_INFO_OBJECT(camerasrc, "src size[%dx%d], capture size[%dx%d]",
	                           camerasrc->width, camerasrc->height,
	                           camerasrc->cap_width, camerasrc->cap_height );

	memset(&main_buf, 0x0, sizeof(camerasrc_buffer_t));
	memset(&thumb_buf, 0x0, sizeof(camerasrc_buffer_t));
	memset(&scrnl_buf, 0x0, sizeof(camerasrc_buffer_t));

	if (command == GST_CAMERA_CONTROL_CAPTURE_COMMAND_STOP) {
		get_stop_command = TRUE;
	} else if (command == GST_CAMERA_CONTROL_CAPTURE_COMMAND_STOP_MULTISHOT) {
		get_stop_multi_command = TRUE;
	}

	/* check whether JPEG capture */
	if (camerasrc->cap_fourcc == GST_MAKE_FOURCC('J','P','E','G')) {
		is_jpeg = TRUE;
	} else {
		is_jpeg = FALSE;
	}

	GST_INFO_OBJECT(camerasrc, "cnt current:%d, reverse:%d, stop cmd:%d, multi stop cmd:%d",
	                           camerasrc->cap_count_reverse, camerasrc->cap_count_current,
	                           get_stop_command, get_stop_multi_command);

	while (TRUE) {
		if (camerasrc->cap_count_reverse == 0 ||
		    (camerasrc->cap_count_current != 0 && (get_stop_command || get_stop_multi_command))) {
			g_mutex_lock(&camerasrc->mutex);

			GST_INFO_OBJECT(camerasrc, "Capture finished.");

			GST_INFO("        capture: gst_camerasrc_capture_stop");
			gst_camerasrc_capture_stop(camerasrc);

			GST_INFO("        capture: gst_camerasrc_start");
			gst_camerasrc_start(camerasrc);

			GST_INFO("        capture: one gst_camerasrc_read_preview");
			ret = gst_camerasrc_read_preview(camerasrc, buffer);

			if (get_stop_command == FALSE) {
				if (!g_queue_is_empty(camerasrc->capture_cmd_list)) {
					command = (int)g_queue_pop_head(camerasrc->capture_cmd_list);
					GST_INFO_OBJECT(camerasrc, "Pop command [%d]", command);
					if (command == GST_CAMERA_CONTROL_CAPTURE_COMMAND_STOP) {
						get_stop_command = TRUE;
					}
				}

				if (get_stop_command == FALSE) {
					GST_INFO_OBJECT(camerasrc, "Start : Wait for Capture stop signal");
					GST_INFO( "            capture: wait for cond after image capture");
					g_cond_wait(&camerasrc->cond, &camerasrc->mutex);
					GST_INFO_OBJECT(camerasrc, "End   : Wait for Capture stop signal");
				}
			}

			get_stop_command = FALSE;
			get_stop_multi_command = FALSE;

			g_mutex_unlock(&camerasrc->mutex);

			return ret;
		}

		GST_INFO("            camerasrc_read_frame:select,DQ");
		ret = camerasrc_read_frame(camerasrc->v4l2_handle, &main_buf, &thumb_buf, &buffer_index);

		if (ret != CAMERASRC_SUCCESS) {
			if (ret == CAMERASRC_ERR_DEVICE_UNAVAILABLE) {
				GST_ERROR_OBJECT (camerasrc, "Video src device return register trouble error!! [%x]", ret);
				/*g_signal_emit (G_OBJECT (camerasrc), gst_camerasrc_signals[SIGNAL_REGISTER_TROUBLE], (GQuark)NULL);*/
			} else {
				GST_ERROR_OBJECT (camerasrc, "camerasrc_read_frame() failed. [ret = 0x%08X]", ret);
				GST_ERROR_OBJECT (camerasrc, "return GST_FLOW_ERROR");
				/* should stop capture; */
			}

			*buffer = NULL;
			gst_camerasrc_error_handler(camerasrc, ret);

			return GST_FLOW_ERROR;
		}

		if (is_jpeg) {
			/* get screennail buffer */
			camerasrc_get_screennail_buffer(camerasrc->v4l2_handle, &scrnl_buf);
		} else {

		}

		GST_INFO("main(%p,%d), thumb(%p,%d), scrnl(%p,%d)",
		         main_buf.planes[0].start, main_buf.planes[0].length,
		         thumb_buf.planes[0].start, thumb_buf.planes[0].length,
		         scrnl_buf.planes[0].start, scrnl_buf.planes[0].length);

CHECK_CAPTURE_INTERVAL:
		/* get shot time */
		cur_time = gst_get_current_time();

		if (camerasrc->cap_count_reverse > 0 && camerasrc->cap_next_time <= cur_time) {
			GST_INFO_OBJECT(camerasrc, "CHECK: reverse capture count: %d, next time:%lu current time:%lu",
			                           camerasrc->cap_count_reverse, camerasrc->cap_next_time, cur_time);

			camerasrc->cap_next_time = cur_time + camerasrc->cap_interval;
			camerasrc->cap_count_reverse--;
			camerasrc->cap_count_current++;

			/* make buffers for capture callback and display(raw format) */
			if (is_jpeg) {
				/* alloc buffer for capture callback */
				GST_INFO_OBJECT (camerasrc, "JPEG CAPTURE MODE");

				gst_buffer = gst_buffer_new_wrapped_full(0, main_buf.planes[0].start,
				                                         main_buf.planes[0].length,
				                                         0,
				                                         main_buf.planes[0].length,
				                                         NULL, NULL);

				sample_caps = gst_caps_new_simple("image/jpeg",
				                                  "width", G_TYPE_INT, camerasrc->cap_width,
				                                  "height", G_TYPE_INT, camerasrc->cap_height,
				                                  NULL);

				buf_sample1 = gst_sample_new(buf->buffer, sample_caps, NULL, NULL);

				gst_buffer_unref(gst_buffer);
				gst_buffer = NULL;

				gst_caps_unref(sample_caps);
				sample_caps = NULL;

				if (thumb_buf.planes[0].start) {
					gst_buffer = gst_buffer_new_wrapped_full(0, thumb_buf.planes[0].start,
					                                         thumb_buf.planes[0].length,
					                                         0,
					                                         thumb_buf.planes[0].length,
					                                         NULL, NULL);

					sample_caps = gst_caps_new_simple("image/jpeg",
					                                  "width", G_TYPE_INT, _THUMBNAIL_WIDTH,
					                                  "height", G_TYPE_INT, _THUMBNAIL_HEIGHT,
					                                  NULL);

					buf_sample2 = gst_sample_new(buf->buffer, sample_caps, NULL, NULL);

					gst_buffer_unref(gst_buffer);
					gst_buffer = NULL;

					gst_caps_unref(sample_caps);
					sample_caps = NULL;
				} else {
					buf_sample2 = NULL;
				}

				if (scrnl_buf.planes[0].start) {
					gst_buffer = gst_buffer_new_wrapped_full(0, scrnl_buf.planes[0].start,
					                                         scrnl_buf.planes[0].length,
					                                         0,
					                                         scrnl_buf.planes[0].length,
					                                         NULL, NULL);

					string_fourcc = gst_video_format_to_string(gst_video_format_from_fourcc(_DEFAULT_SCRNL_FOURCC));

					sample_caps = gst_caps_new_simple("video/x-raw",
					                                  "format", G_TYPE_STRING, string_fourcc,
					                                  "width", G_TYPE_INT, camerasrc->width,
					                                  "height", G_TYPE_INT, camerasrc->height,
					                                  NULL);

					buf_sample3 = gst_sample_new(buf->buffer, sample_caps, NULL, NULL);

					gst_buffer_unref(gst_buffer);
					gst_buffer = NULL;

					gst_caps_unref(sample_caps);
					sample_caps = NULL;

					string_fourcc = NULL;
				} else {
					buf_sample3 = NULL;
				}
			} else {
				camerasrc_capture_data_info capture_data;

				GST_INFO_OBJECT (camerasrc, "RAW or ITLV CAPTURE MODE");

				/*alloc main buffer*/
				buf = gst_camerasrc_buffer_new(camerasrc);;
				if (buf == NULL) {
					GST_ERROR_OBJECT(camerasrc, "Buffer alloc failed.");
					*buffer = NULL;
					gst_camerasrc_error_handler(camerasrc, CAMERASRC_ERR_ALLOCATION);
					return GST_FLOW_ERROR;
				}

				memset(&capture_data, 0x0, sizeof(camerasrc_capture_data_info));

				capture_data.buffer_index = buffer_index;
				capture_data.make_thumbnail = FALSE;
				capture_data.flash_activated = camerasrc->flash_activated;

				GST_INFO("                read_capture:camerasrc_extract_exif_info_from_capture_data");
				camerasrc_extract_exif_info_from_capture_data(camerasrc->v4l2_handle, &capture_data);

				if (camerasrc->cap_fourcc == GST_MAKE_FOURCC('I','T','L','V')) {
					/* for virtual address - GST_BUFFER_DATA and SIZE */
					gst_buffer_append_memory(buf->buffer, gst_memory_new_wrapped(0,
					                                                             camerasrc->buffer_info[buffer_index].vaddr[2],
					                                                             camerasrc->buffer_info[buffer_index].bytesused,
					                                                             0,
					                                                             camerasrc->buffer_info[buffer_index].bytesused,
					                                                             NULL,
					                                                             NULL));

					/* emit capture signal */
					GST_INFO("                read_capture:gst_camerasrc_emit_capture_signal");
					ret = gst_camerasrc_emit_capture_signal(camerasrc, &main_buf, &thumb_buf, &scrnl_buf);

					/* free allocated memory
					- interleaved data capture mode
					- main and thumbnail buffer
					- screennail buffer is NOT allocated memory */
					if (main_buf.planes[0].start) {
						free(main_buf.planes[0].start);
						main_buf.planes[0].start = NULL;
						main_buf.planes[0].length = 0;
					}
					if (thumb_buf.planes[0].start) {
						free(thumb_buf.planes[0].start);
						thumb_buf.planes[0].start = NULL;
						thumb_buf.planes[0].length = 0;
					}
				} else {
					if (camerasrc->cap_fourcc == GST_MAKE_FOURCC('N','V','1','2') ||
					    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','N','1','2') ||
					    camerasrc->cap_fourcc == GST_MAKE_FOURCC('N','V','2','1') ||
					    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','N','2','1') ||
					    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','4','2','0') ||
					    camerasrc->cap_fourcc == GST_MAKE_FOURCC('I','4','2','0') ||
					    camerasrc->cap_fourcc == GST_MAKE_FOURCC('Y','V','1','2')) {
						ret = _gst_camerasrc_get_normal_buffer(camerasrc, camerasrc->cap_fourcc,
						                                       main_buf.planes[0].start, camerasrc->cap_width, camerasrc->cap_height,
						                                       &buf_memory);
						if (ret == FALSE) {
							return GST_FLOW_ERROR;
						}
						gst_buffer_append_memory(buf->buffer, buf_memory);
						buf_memory = NULL;
					} else {
						buf_memory = gst_memory_new_wrapped(0,
						                                    main_buf.planes[0].start,
						                                    main_buf.planes[0].length,
						                                    0,
						                                    main_buf.planes[0].length,
						                                    NULL,
						                                    NULL);
						gst_buffer_append_memory(buf->buffer, buf_memory);
						buf_memory = NULL;
					}

					string_fourcc = gst_video_format_to_string(gst_video_format_from_fourcc(camerasrc->cap_fourcc));

					sample_caps = gst_caps_new_simple("video/x-raw",
					                                  "format", G_TYPE_STRING, string_fourcc,
					                                  "width", G_TYPE_INT, camerasrc->cap_width,
					                                  "height", G_TYPE_INT, camerasrc->cap_height,
					                                  NULL);

					/* alloc buffer for capture callback */
					buf_sample1 = gst_sample_new(buf->buffer, sample_caps, NULL, NULL);
					buf_sample2 = NULL;
					buf_sample3 = NULL;

					gst_caps_unref(sample_caps);
					sample_caps = NULL;

					string_fourcc = NULL;
				}

				/* get meta data */
				if (camerasrc->cap_fourcc == GST_MAKE_FOURCC('I','T','L','V') ||
				    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','N','1','2') ||
				    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','T','1','2') ||
				    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','4','2','0') ||
				    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','U','Y','V') ||
				    camerasrc->cap_fourcc == GST_MAKE_FOURCC('S','Y','V','Y')) {
					gst_buffer_insert_memory(buf->buffer, 0, gst_camerasrc_get_zero_copy_data(camerasrc, buffer_index, camerasrc->cap_fourcc));
				}

				buf->v4l2_buffer_index = buffer_index;

				*buffer = buf->buffer;
				gst_buffer_map(*buffer, &buf_info, GST_MAP_READ);
				GST_INFO_OBJECT(camerasrc, "BUF for PREVIEW: addr %p size %d, format %c%c%c%c",
				                           buf_info.data, buf_info.maxsize,
				                           camerasrc->cap_fourcc, camerasrc->cap_fourcc>>8, camerasrc->cap_fourcc>>16, camerasrc->cap_fourcc>>24);
				gst_buffer_unmap(*buffer, &buf_info);
			}

			if (camerasrc->cap_fourcc != GST_MAKE_FOURCC('I','T','L','V')) {
				/*call signal*/
				GST_INFO_OBJECT (camerasrc, "CALL: capture callback");
				g_signal_emit(G_OBJECT (camerasrc),
				              gst_camerasrc_signals[SIGNAL_STILL_CAPTURE],
				              0,
				              buf_sample1,
				              buf_sample2,
				              buf_sample3);
				GST_INFO_OBJECT (camerasrc, "RETURN: capture callback");
			}

			GST_INFO("fourcc[preview %c%c%c%c,capture %c%c%c%c], size[preview %dx%d, capture %dx%d]",
			         camerasrc->format_name[0], camerasrc->format_name[1], camerasrc->format_name[2], camerasrc->format_name[3],
			         camerasrc->cap_fourcc, camerasrc->cap_fourcc>>8, camerasrc->cap_fourcc>>16, camerasrc->cap_fourcc>>24,
			         camerasrc->width, camerasrc->height, camerasrc->cap_width, camerasrc->cap_height);

			if ((is_jpeg || camerasrc->cap_count == 1 ||
			     MAKE_FOURCC_FROM_STRING(camerasrc->format_name) != camerasrc->cap_fourcc ||
			     camerasrc->width != camerasrc->cap_width ||
			     camerasrc->height != camerasrc->cap_height) &&
			    camerasrc->cap_fourcc != GST_MAKE_FOURCC('I','T','L','V')) {
				/* release allocated buffer */
				if (buf) {
					g_mutex_lock(&camerasrc->buffer_lock);
					camerasrc->num_live_buffers++;
					g_mutex_unlock(&camerasrc->buffer_lock);
					gst_buffer_map(buf->buffer, &buf_info, GST_MAP_READ);
					if (main_buf.planes[0].start != buf_info.data) {
						GST_INFO("release allocated buffer");
					}
					gst_buffer_unmap(buf->buffer, &buf_info);
					gst_buffer_set_size(buf->buffer, 0);
					GST_INFO (" refcount = %d",GST_MINI_OBJECT_REFCOUNT_VALUE(GST_MINI_OBJECT_CAST (buf)));
					gst_buffer_unref(buf->buffer);
					GST_INFO (" refcount = %d",GST_MINI_OBJECT_REFCOUNT_VALUE(GST_MINI_OBJECT_CAST (buf)));
					buf = NULL;
					*buffer = NULL;
				}

				GST_INFO("skip this buffer");

				/* Queue buffer */
				camerasrc_queue_buffer(camerasrc->v4l2_handle, buffer_index);
			} else {
				GST_INFO("send buffer to next element");
				g_mutex_lock(&camerasrc->buffer_lock);
				camerasrc->num_live_buffers++;
				g_mutex_unlock(&camerasrc->buffer_lock);
				/*escape loop for passing buffer to videosink*/
				break;
			}
		} else {
			/* check again  */
			if (camerasrc->cap_next_time < cur_time + _CONTINUOUS_SHOT_MARGIN) {
				GST_DEBUG_OBJECT(camerasrc, "check again time");
				usleep((camerasrc->cap_next_time - cur_time) * 1000);
				goto CHECK_CAPTURE_INTERVAL;
			}

			if (is_jpeg == FALSE &&
			    camerasrc->cap_fourcc == GST_MAKE_FOURCC('I','T','L','V')) {
				/* free allocated memory
				- interleaved data capture mode
				- main and thumbnail buffer
				- screennail buffer is NOT allocated memory */
				if (main_buf.planes[0].start) {
					free(main_buf.planes[0].start);
					main_buf.planes[0].start = NULL;
					main_buf.planes[0].length = 0;
				}
				if (thumb_buf.planes[0].start) {
					free(thumb_buf.planes[0].start);
					thumb_buf.planes[0].start = NULL;
					thumb_buf.planes[0].length = 0;
				}
			}

			/* RAW capture buffer should be reach here */
			camerasrc_queue_buffer(camerasrc->v4l2_handle, buffer_index);

			/* Skip passing this buffer */
			*buffer = NULL;

			GST_INFO("send NULL buffer");
			break;
		}
	}

	GST_DEBUG_OBJECT (camerasrc, "LEAVED");

	return GST_FLOW_OK;
}


static GstFlowReturn gst_camerasrc_read(GstCameraSrc *camerasrc, GstBuffer **buffer)
{
	int command = GST_CAMERA_CONTROL_CAPTURE_COMMAND_NONE;
	GstFlowReturn ret = GST_FLOW_OK;

	g_mutex_lock(&camerasrc->mutex);

	if (!g_queue_is_empty(camerasrc->capture_cmd_list)) {
		command = (int)g_queue_pop_head(camerasrc->capture_cmd_list);
		GST_INFO_OBJECT(camerasrc, "popped cmd : %d", command);
	}

	/* Normal Capture Routine */
	if (command == GST_CAMERA_CONTROL_CAPTURE_COMMAND_START) {
		GST_DEBUG_OBJECT(camerasrc, "gst_camerasrc_capture_start");
		gst_camerasrc_capture_start(camerasrc);
	}

	g_mutex_unlock(&camerasrc->mutex);

	switch (camerasrc->mode) {
	case VIDEO_IN_MODE_PREVIEW:
	case VIDEO_IN_MODE_VIDEO:
		GST_DEBUG_OBJECT(camerasrc, "gst_camerasrc_read_preview");
		ret = gst_camerasrc_read_preview(camerasrc, buffer);
		break;
	case VIDEO_IN_MODE_CAPTURE:
		GST_DEBUG_OBJECT(camerasrc, "gst_camerasrc_read_capture");
		ret = gst_camerasrc_read_capture(camerasrc, buffer, command);
		break;
	case VIDEO_IN_MODE_UNKNOWN:
	default:
		ret = GST_FLOW_ERROR;
		GST_ERROR_OBJECT (camerasrc, "can't reach statement.[camerasrc->mode=%d]", camerasrc->mode);
		break;
	}

	if (!buffer || !(*buffer) || !GST_IS_BUFFER(*buffer)) {
		/* To avoid seg fault, make dummy buffer. */
		GST_WARNING_OBJECT (camerasrc, "Make a dummy buffer");
		*buffer = gst_buffer_new();
	}

	return ret;
}


static void gst_camerasrc_buffer_qbuf(GstCameraSrc *camerasrc, int buffer_index)
{
	int ret = 0;

	if (camerasrc == NULL) {
		GST_ERROR("camerasrc is NULL");
		return;
	}

	ret = camerasrc_queue_buffer(camerasrc->v4l2_handle, buffer_index);
	if (ret != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT(camerasrc, "QBUF error, [0x%x]", ret);
	} else {
		GST_LOG_OBJECT(camerasrc, "QBUF : [idx=%d]", buffer_index);
	}

	return;
}


static void gst_camerasrc_buffer_finalize(GstCameraBuffer *buffer)
{
	int index = 0;
	gboolean ret = FALSE;
	unsigned long cur_time = 0;
	GstCameraSrc *camerasrc = NULL;
	camerasrc_capture_data_info *capture_data = NULL;
	gboolean capture_done = FALSE;

	camerasrc = buffer->camerasrc;
	index = buffer->v4l2_buffer_index;

	GST_DEBUG_OBJECT(camerasrc, "finalizing buffer %p, index %d [%"GST_TIME_FORMAT " dur %"GST_TIME_FORMAT"]",
	                            buffer, index,
	                            GST_TIME_ARGS(GST_BUFFER_TIMESTAMP(buffer->buffer)),
	                            GST_TIME_ARGS(GST_BUFFER_DURATION(buffer->buffer)));

	/*
	 * Try zero system lag capture when it meets conditions
	 */
	cur_time = gst_get_current_time();

	g_mutex_lock(&camerasrc->jpg_mutex);

	if (camerasrc->cap_fourcc == GST_MAKE_FOURCC('J','P','E','G') &&
	    camerasrc->create_jpeg == TRUE &&
	    (camerasrc->cap_count_reverse > 0 && camerasrc->cap_next_time <= cur_time) ) {

		capture_data = (camerasrc_capture_data_info *)malloc(sizeof(camerasrc_capture_data_info));
		if (capture_data) {
			memset(capture_data, 0x0, sizeof(camerasrc_capture_data_info));

			capture_data->buffer_index = index;
			capture_data->make_thumbnail = TRUE;
			capture_data->flash_activated = camerasrc->flash_activated;

			GST_INFO_OBJECT(camerasrc, "camerasrc_extract_exif_info_from_capture_data");
			camerasrc_extract_exif_info_from_capture_data(camerasrc->v4l2_handle, capture_data);

			free(capture_data);
			capture_data = NULL;
		}

		GST_INFO_OBJECT(camerasrc, "gst_camerasrc_jpeg_capture");
		ret = gst_camerasrc_jpeg_capture(camerasrc, index, buffer);
		if (ret == FALSE) {
			GST_ERROR_OBJECT(camerasrc, "gst_camerasrc_jpeg_capture failed");
			gst_camerasrc_error_handler(camerasrc, CAMERASRC_ERR_INTERNAL);
			capture_done = FALSE;
		} else {
			capture_done = TRUE;
		}

		if (capture_done) {
			if ((--camerasrc->cap_count_reverse) != 0) {
				camerasrc->cap_next_time = cur_time + camerasrc->cap_interval;
			} else {
				camerasrc->cap_next_time = 0;
				camerasrc->create_jpeg = FALSE;
			}

			GST_INFO_OBJECT(camerasrc, "count %d", camerasrc->cap_count_current);
		}
	}

	g_mutex_unlock(&camerasrc->jpg_mutex);

	if (camerasrc->buffer_running) {
		/* Buffer Q again */
		gst_camerasrc_buffer_qbuf(camerasrc, index);
	} else {
		GST_INFO_OBJECT(camerasrc, "It is not running. skip QBUF");
	}

	g_mutex_lock(&camerasrc->buffer_lock);

	camerasrc->num_live_buffers--;

	GST_DEBUG_OBJECT(camerasrc, "QBUF : [idx=%d, lvn=%d]", index, camerasrc->num_live_buffers);

	/* send buffer_cond signal for waiting thread */
	g_cond_signal(&camerasrc->buffer_cond);

	g_mutex_unlock(&camerasrc->buffer_lock);

	gst_object_unref(camerasrc);

	free(buffer);

	return;
}


G_DEFINE_BOXED_TYPE(GstCameraBuffer, gst_camerasrc_buffer, NULL, gst_camerasrc_buffer_finalize);

static GstCameraBuffer *gst_camerasrc_buffer_new(GstCameraSrc *camerasrc)
{
	GstCameraBuffer *ret = NULL;

	ret = (GstCameraBuffer *)malloc(sizeof(*ret));
	ret->buffer = gst_buffer_new();

	GST_LOG_OBJECT(camerasrc, "creating buffer : %p", ret);

	ret->camerasrc = gst_object_ref(GST_OBJECT(camerasrc));

	return ret;
}


static gboolean _gst_camerasrc_NV21_to_I420(GstCameraSrc *camerasrc, unsigned char *src_Y, unsigned char *src_UV, unsigned char **dest, int width, int height)
{
	int i = 0;
	int loop_count = 0;
	int length_total = 0;
	int length_Y = 0;
	int length_UV = 0;
	unsigned char *result = NULL;
	unsigned char *result_U = NULL;
	unsigned char *result_V = NULL;

	if (camerasrc == NULL || src_Y == NULL || src_UV == NULL || dest == NULL) {
		GST_ERROR_OBJECT(camerasrc, "NULL pointer %p %p %p %p", camerasrc, src_Y, src_UV, dest);
		return FALSE;
	}

	length_Y = width * height;
	length_UV = length_Y >> 1;
	length_total = length_Y + length_UV;

	GST_INFO_OBJECT(camerasrc, "%dx%d, length total %d, Y %d, UV %d",
	                           width, height, length_total, length_Y, length_UV);

	result = (unsigned char *)malloc(length_total);
	if (result == NULL) {
		GST_ERROR_OBJECT(camerasrc, "NULL pointer %p %p %p", src_Y, src_UV, dest);
		return FALSE;
	}

	GST_INFO_OBJECT(camerasrc, "src Y memcpy start (size %d)", length_Y);

	memcpy(result, src_Y, length_Y);

	GST_INFO_OBJECT(camerasrc, "src Y memcpy done (size %d)", length_Y);

	result_U = result + length_Y;
	result_V = result_U + (length_UV >> 1);
	loop_count = length_UV >> 1;

	for (i = 0 ; i < loop_count ; i++) {
		result_U[i] = src_UV[(i<<1) + 1];
		result_V[i] = src_UV[i<<1];
	}

	*dest = result;

	GST_INFO_OBJECT(camerasrc, "done : %p", *dest);

	if (0) {
		FILE *fp_src = NULL;
		FILE *fp_dest = NULL;

		fp_src = fopen("/opt/usr/media/NV12.yuv", "wb");
		if (fp_src) {
			fwrite(src_Y, 1, length_Y, fp_src);
			fwrite(src_UV, 1, length_UV, fp_src);
			fclose(fp_src);
			fp_src = NULL;
		}

		fp_dest = fopen("/opt/usr/media/I420.yuv", "wb");
		if (fp_dest) {
			fwrite(result, 1, length_total, fp_dest);
			fclose(fp_dest);
			fp_dest = NULL;
		}
	}

	return TRUE;
}


static gboolean gst_camerasrc_jpeg_capture(GstCameraSrc *camerasrc, gint buffer_index, GstCameraBuffer *buffer)
{
	int ret = 0;
	unsigned char *buffer_I420 = NULL;
	unsigned char *jpeg_result = NULL;
	int jpeg_size = 0;

	GstCaps *main_caps = NULL;
	GstBuffer *main_buffer = NULL;
	GstSample *main_sample = NULL;

	camerasrc_frame_data_t frame_data;

	GST_INFO_OBJECT(camerasrc, "Start to encode image");

	CLEAR(frame_data);

	frame_data.index = buffer_index;
	camerasrc_get_frame_data(camerasrc->v4l2_handle, &frame_data);

	if (!_gst_camerasrc_NV21_to_I420(camerasrc,
	                                 frame_data.buffer.planes[0].start,
	                                 frame_data.buffer.planes[1].start,
	                                 &buffer_I420,
	                                 camerasrc->width,
	                                 camerasrc->height)) {
		GST_ERROR_OBJECT(camerasrc, "failed to convert I420");
		return FALSE;
	}

	if (buffer_I420 == NULL) {
		GST_ERROR_OBJECT(camerasrc, "buffer_I420 is NULL");
		return FALSE;
	}

	ret = mm_util_jpeg_encode_to_memory((void **)&jpeg_result, &jpeg_size,
	                                    buffer_I420, camerasrc->width, camerasrc->height,
	                                    MM_UTIL_JPEG_FMT_YUV420, camerasrc->cap_jpg_quality);

	/* release converted src buffer */
	free(buffer_I420);
	buffer_I420 = NULL;

	if (ret != MM_ERROR_NONE || jpeg_result == NULL) {
		GST_ERROR_OBJECT(camerasrc, "jpeg encoding failed %p, 0x%x", jpeg_result, ret);

		if (jpeg_result) {
			free(jpeg_result);
			jpeg_result = NULL;
		}

		return FALSE;
	}

	/* create gst buffer with jpeg_result */
	main_buffer = gst_buffer_new_wrapped_full(0, jpeg_result, jpeg_size, 0, jpeg_size, NULL, free);
	if (main_buffer == NULL) {
		GST_ERROR_OBJECT(camerasrc, "gst_buffer_new_wrapped_full failed");

		free(jpeg_result);
		jpeg_result = NULL;

		return FALSE;
	}

	/* create caps info */
	main_caps = gst_caps_new_simple("image/jpeg",
	                                "width", G_TYPE_INT, camerasrc->width,
	                                "height", G_TYPE_INT, camerasrc->height,
	                                NULL);
	if (main_caps == NULL) {
		GST_ERROR_OBJECT(camerasrc, "gst_caps_new_simple failed");

		free(jpeg_result);
		jpeg_result = NULL;

		gst_buffer_unref(main_buffer);
		main_buffer = NULL;

		return FALSE;
	}

	/* create sample */
	main_sample = gst_sample_new(main_buffer, main_caps, NULL, NULL);

	/* unref gst buffer and caps because their ref count will be increased in gst_sample_new */
	gst_buffer_unref(main_buffer);
	main_buffer = NULL;

	gst_caps_unref(main_caps);
	main_caps = NULL;

	if (main_sample == NULL) {
		GST_ERROR_OBJECT(camerasrc, "gst_sample_new failed");
		return FALSE;
	}

	/* signal emit */
	g_signal_emit(G_OBJECT(camerasrc),
	              gst_camerasrc_signals[SIGNAL_STILL_CAPTURE],
	              0,
	              main_sample,
	              NULL,
	              NULL);

	GST_ERROR_OBJECT(camerasrc, "capture signal emit done");

	return TRUE;
}


static gboolean gst_camerasrc_emit_capture_signal(GstCameraSrc *camerasrc,
                                                  camerasrc_buffer_t *main,
                                                  camerasrc_buffer_t *thumb,
                                                  camerasrc_buffer_t *scrnl)
{
	/* GstBuffers for application */
	GstBuffer *buf_cap_signal_main = NULL;
	GstBuffer *buf_cap_signal_thumb = NULL;
	GstBuffer *buf_cap_signal_scrnl = NULL;
	GstCaps *buf_caps_main = NULL;
	GstCaps *buf_caps_thumb = NULL;
	GstCaps *buf_caps_scrnl = NULL;
	GstSample *buf_sample_main = NULL;
	GstSample *buf_sample_thumb = NULL;
	GstSample *buf_sample_scrnl = NULL;

	if (camerasrc == NULL) {
		GST_ERROR("camerasrc is NULL");
		return FALSE;
	}

	if (main == NULL || thumb == NULL || scrnl == NULL) {
		GST_ERROR_OBJECT(camerasrc, "[main:%p, thumb:%p, scrnl:%p] something is NULL", main, thumb, scrnl);
		return FALSE;
	}

	/* make GstBuffers with captured data */
	buf_cap_signal_main = gst_buffer_new_wrapped_full(0, main->planes[0].start, main->planes[0].length, 0, main->planes[0].length, NULL, NULL);
	GST_INFO("main %p, %d", main->planes[0].start, main->planes[0].length);
	buf_caps_main = gst_caps_new_simple("image/jpeg",
	                                    "width", G_TYPE_INT, camerasrc->cap_width,
	                                    "height", G_TYPE_INT, camerasrc->cap_height,
	                                    NULL);
	buf_sample_main = gst_sample_new(buf_cap_signal_main, buf_caps_main, NULL, NULL);

	/* thumbnail */
	if (thumb->planes[0].start) {
		buf_cap_signal_thumb = gst_buffer_new_wrapped_full(0, thumb->planes[0].start, thumb->planes[0].length, 0, thumb->planes[0].length, NULL, NULL);
		buf_caps_thumb = gst_caps_new_simple("image/jpeg",
		                                     "width", G_TYPE_INT, thumb->width,
		                                     "height", G_TYPE_INT, thumb->height,
		                                     NULL);
		buf_sample_thumb = gst_sample_new(buf_cap_signal_thumb, buf_caps_thumb, NULL, NULL);
	} else {
		buf_cap_signal_thumb = NULL;
	}

	/* screennail (postview) */
	if (scrnl->planes[0].start) {
		buf_cap_signal_scrnl = gst_buffer_new_wrapped_full(0, scrnl->planes[0].start, scrnl->planes[0].length, 0, scrnl->planes[0].length, NULL, NULL);
		buf_caps_scrnl = gst_caps_new_simple("video/x-raw",
		                                     "format", G_TYPE_STRING, _DEFAULT_RAW_DATA_FOURCC,
		                                     "width", G_TYPE_INT, camerasrc->width,
		                                     "height", G_TYPE_INT, camerasrc->height,
		                                     NULL);
		buf_sample_scrnl = gst_sample_new(buf_cap_signal_scrnl, buf_caps_scrnl, NULL, NULL);
	} else {
		buf_cap_signal_scrnl = NULL;
	}

	/* emit capture signal */
	GST_INFO_OBJECT(camerasrc, "Call Capture SIGNAL");

	g_signal_emit(G_OBJECT (camerasrc),
	              gst_camerasrc_signals[SIGNAL_STILL_CAPTURE],
	              0,
	              buf_sample_main,
	              buf_sample_thumb,
	              buf_sample_scrnl);

	GST_INFO_OBJECT(camerasrc, "Return Capture SIGNAL");

	return TRUE;
}


static GstMemory *gst_camerasrc_get_zero_copy_data(GstCameraSrc *camerasrc, guint32 index, guint32 fourcc)
{
	GstMemory *meta = NULL;
	camerasrc_frame_data_t data;
	MMVideoBuffer *mm_buf = NULL;

	/*GST_LOG_OBJECT (camerasrc, "index[%d],pix_format[%x]", index, camerasrc->fourcc);*/

	mm_buf = (MMVideoBuffer *)malloc(sizeof(MMVideoBuffer));
	if (mm_buf == NULL) {
		GST_ERROR_OBJECT(camerasrc, "failed to alloc MMVideoBuffer");
		return NULL;
	}

	memset(mm_buf, 0x0, sizeof(MMVideoBuffer));

	mm_buf->type = MM_VIDEO_BUFFER_TYPE_TBM_BO;

	data.index = index;
	camerasrc_get_frame_data(camerasrc->v4l2_handle, &data);

	mm_buf->width[0] = camerasrc->width;
	mm_buf->height[0] = camerasrc->height;
	mm_buf->size[0] = data.buffer.planes[0].length;
	mm_buf->data[0] = data.buffer.planes[0].start;
	mm_buf->handle.bo[0] = data.buffer.planes[0].bo;

	if (fourcc == GST_MAKE_FOURCC('S','T','1','2')) {
		mm_buf->width[1] = camerasrc->width;
		mm_buf->height[1] = camerasrc->height >> 1;
		mm_buf->size[1] = data.buffer.planes[1].length;
		mm_buf->data[1] = data.buffer.planes[1].start;
		mm_buf->handle.bo[1] = data.buffer.planes[1].bo;

		mm_buf->stride_width[0] = GST_ROUND_UP_16(mm_buf->width[0]);
		mm_buf->stride_height[0] = GST_ROUND_UP_16(mm_buf->height[0]);
		mm_buf->stride_width[1] = GST_ROUND_UP_16(mm_buf->width[1]);
		mm_buf->stride_height[1] = GST_ROUND_UP_16(mm_buf->height[1]);
	} else {
		if (fourcc == GST_MAKE_FOURCC('S','N','1','2') ||
		    fourcc == GST_MAKE_FOURCC('S','N','2','1')) {
			mm_buf->width[1] = camerasrc->width;
			mm_buf->height[1] = camerasrc->height >> 1;
			mm_buf->size[1] = data.buffer.planes[1].length;
			mm_buf->data[1] = data.buffer.planes[1].start;
			mm_buf->handle.bo[1] = data.buffer.planes[1].bo;
		} else {
			GST_WARNING_OBJECT (camerasrc, "Unknown pixel format.");
		}

		mm_buf->stride_width[0] = mm_buf->width[0];
		mm_buf->stride_height[0] = mm_buf->height[0];
		mm_buf->stride_width[1] = mm_buf->width[1];
		mm_buf->stride_height[1] = mm_buf->height[1];
		mm_buf->stride_width[2] = mm_buf->width[2];
		mm_buf->stride_height[2] = mm_buf->height[2];
	}

	GST_LOG_OBJECT(camerasrc, "index[%d],bo Plane0[%p],Plane1[%p],Plane2[%p]",
	                          index, mm_buf->handle.bo[0], mm_buf->handle.bo[1], mm_buf->handle.bo[2]);

	meta = gst_memory_new_wrapped(GST_MEMORY_FLAG_READONLY,
	                              mm_buf,
	                              sizeof(MMVideoBuffer),
	                              0,
	                              sizeof(MMVideoBuffer),
	                              mm_buf,
	                              free);

	return meta;
}


static gboolean gst_camerasrc_get_timeinfo(GstCameraSrc *camerasrc, GstBuffer  *buffer)
{
	int fps_nu = 0;
	int fps_de = 0;
	GstClock *clock = NULL;
	GstClockTime timestamp = GST_CLOCK_TIME_NONE;
	GstClockTime duration = GST_CLOCK_TIME_NONE;

	if (!camerasrc || !buffer) {
		GST_WARNING_OBJECT (camerasrc, "Invalid pointer [hadle:%p, buffer:%p]", camerasrc, buffer);
		return FALSE;
	}

	/* timestamps, LOCK to get clock and base time. */
	clock = GST_ELEMENT_CLOCK(camerasrc);
	if (clock) {
		/* the time now is the time of the clock minus the base time */
		gst_object_ref(clock);
		timestamp = gst_clock_get_time(clock) - GST_ELEMENT(camerasrc)->base_time;
		gst_object_unref(clock);

		/* if we have a framerate adjust timestamp for frame latency */
		if (camerasrc->fps_auto) {
			/* auto fps mode */
			duration = GST_CLOCK_TIME_NONE;
		} else {
			if (camerasrc->fps <= 0) {
				/*if fps is zero, auto fps mode*/
				fps_nu = 0;
				fps_de = 1;
			} else {
				fps_nu = 1;
				fps_de = camerasrc->fps;
			}

			if (fps_nu > 0 && fps_de > 0) {
				duration = gst_util_uint64_scale_int(GST_SECOND, fps_nu, fps_de);
			}
		}

		/* set default duration if duration is NONE : 30 fps */
		if (duration == GST_CLOCK_TIME_NONE) {
			duration = gst_util_uint64_scale_int(GST_SECOND, 1, 30);
		}
	} else {
		/* no clock, can't set timestamps */
		timestamp = GST_CLOCK_TIME_NONE;
	}

	GST_BUFFER_TIMESTAMP(buffer) = timestamp;
	GST_BUFFER_DURATION(buffer) = duration;
/*
	GST_INFO_OBJECT(camerasrc, "[%"GST_TIME_FORMAT" dur %" GST_TIME_FORMAT "]",
	                           GST_TIME_ARGS(GST_BUFFER_TIMESTAMP(buffer)),
	                           GST_TIME_ARGS(GST_BUFFER_DURATION(buffer)));
*/
	return TRUE;
}


/* Gstreamer general functions */
static gboolean gst_camerasrc_src_start(GstBaseSrc *src)
{
	int ret = TRUE;
	GstCameraSrc *camerasrc = GST_CAMERA_SRC (src);

	GST_DEBUG_OBJECT(camerasrc, "ENTERED");

	camerasrc->firsttime = TRUE;
	/* 'gst_camerasrc_set_caps' will call gst_camerasrc_start(). So skip to call it. */
	/*ret = gst_camerasrc_start(camerasrc);*/

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return ret;
}


static gboolean gst_camerasrc_src_stop(GstBaseSrc *src)
{
	int ret = 0;
	GstCameraSrc *camerasrc = GST_CAMERA_SRC(src);

	GST_DEBUG_OBJECT (camerasrc, "ENTERED");

	ret = gst_camerasrc_stop(camerasrc);

	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return ret;
}


static GstFlowReturn gst_camerasrc_src_create(GstPushSrc *src, GstBuffer **buffer)
{
	GstCameraSrc *camerasrc = GST_CAMERA_SRC (src);
	GstFlowReturn ret;

	GST_LOG_OBJECT(camerasrc, "ENTERED");

	ret = gst_camerasrc_read(camerasrc, buffer);

	GST_LOG_OBJECT (camerasrc, "LEAVED");

	return ret;
}


static void gst_camerasrc_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec)
{
	int tmp = 0;
	GstCameraSrc *camerasrc = NULL;

	g_return_if_fail(GST_IS_CAMERA_SRC(object));
	camerasrc = GST_CAMERA_SRC(object);

	switch (prop_id) {
	case ARG_CAMERA_HIGH_SPEED_FPS:
		camerasrc->high_speed_fps = g_value_get_int(value);
		GST_INFO_OBJECT(camerasrc, "Set HIGH SPEED FPS: %d", camerasrc->high_speed_fps);
		break;
	case ARG_CAMERA_AUTO_FPS:
		camerasrc->fps_auto = g_value_get_boolean(value);
		GST_INFO_OBJECT(camerasrc, "Set AUTO_FPS: %d", camerasrc->fps_auto);
		break;
	case ARG_CAMERA_ID:
		camerasrc->camera_id = g_value_get_int(value);
		break;
	case ARG_CAMERA_CAPTURE_FOURCC:
		camerasrc->cap_fourcc = g_value_get_uint(value);
		break;
	case ARG_CAMERA_CAPTURE_WIDTH:
		camerasrc->cap_width = g_value_get_int(value);
		GST_INFO_OBJECT(camerasrc, "Set capture width: %d", camerasrc->cap_width);
		break;
	case ARG_CAMERA_CAPTURE_HEIGHT:
		camerasrc->cap_height = g_value_get_int(value);
		GST_INFO_OBJECT(camerasrc, "Set capture height: %d", camerasrc->cap_height);
		break;
	case ARG_CAMERA_CAPTURE_INTERVAL:
		camerasrc->cap_interval = g_value_get_int(value);
		GST_INFO_OBJECT(camerasrc, "Set capture interval: %d", camerasrc->cap_interval);
		break;
	case ARG_CAMERA_CAPTURE_COUNT:
		tmp = g_value_get_int(value);
		camerasrc->cap_count = tmp;
		g_mutex_lock(&camerasrc->jpg_mutex);
		camerasrc->cap_count_reverse = tmp;
		g_mutex_unlock(&camerasrc->jpg_mutex);
		GST_INFO_OBJECT(camerasrc, "Set capture count: %d", camerasrc->cap_count_reverse);
		break;
	case ARG_CAMERA_CAPTURE_JPG_QUALITY:
	{
		camerasrc->cap_jpg_quality = g_value_get_int(value);
		GST_INFO_OBJECT(camerasrc, "Set jpeg quality : %d", camerasrc->cap_jpg_quality);
		break;
	}
	case ARG_VFLIP:
		camerasrc->vflip = g_value_get_boolean(value);
		GST_INFO_OBJECT(camerasrc, "Set VFLIP : %d", camerasrc->vflip);
		break;
	case ARG_HFLIP:
		camerasrc->hflip = g_value_get_boolean(value);
		GST_INFO_OBJECT(camerasrc, "Set HFLIP : %d", camerasrc->hflip);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		break;
	}

	return;
}


static void gst_camerasrc_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec)
{
	GstCameraSrc *camerasrc;

	g_return_if_fail(GST_IS_CAMERA_SRC(object));
	camerasrc = GST_CAMERA_SRC(object);

	switch (prop_id) {
	case ARG_CAMERA_HIGH_SPEED_FPS:
		g_value_set_int(value, camerasrc->high_speed_fps);
		break;
	case ARG_CAMERA_AUTO_FPS:
		g_value_set_boolean(value, camerasrc->fps_auto);
		break;
	case ARG_CAMERA_ID:
		g_value_set_int(value, camerasrc->camera_id);
		break;
	case ARG_CAMERA_CAPTURE_FOURCC:
		g_value_set_uint(value, camerasrc->cap_fourcc);
		break;
	case ARG_CAMERA_CAPTURE_WIDTH:
		g_value_set_int(value, camerasrc->cap_width);
		break;
	case ARG_CAMERA_CAPTURE_HEIGHT:
		g_value_set_int(value, camerasrc->cap_height);
		break;
	case ARG_CAMERA_CAPTURE_INTERVAL:
		g_value_set_int(value, camerasrc->cap_interval);
		break;
	case ARG_CAMERA_CAPTURE_COUNT:
		g_value_set_int(value, camerasrc->cap_count);
		break;
	case ARG_CAMERA_CAPTURE_JPG_QUALITY:
		g_value_set_int(value, camerasrc->cap_jpg_quality);
		GST_INFO("GET jpeg compress ratio : %d", camerasrc->cap_jpg_quality);
		break;
	case ARG_CAMERA_CAPTURE_PROVIDE_EXIF:
		g_value_set_boolean(value, camerasrc->cap_provide_exif);
		GST_INFO("Is Exif provided? : %d", camerasrc->cap_provide_exif);
		break;
	case ARG_VFLIP:
		g_value_set_boolean(value, camerasrc->vflip);
		break;
	case ARG_HFLIP:
		g_value_set_boolean(value, camerasrc->hflip);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
		break;
	}

	return;
}


static GstStateChangeReturn gst_camerasrc_change_state(GstElement *element, GstStateChange transition)
{
	GstStateChangeReturn ret = GST_STATE_CHANGE_SUCCESS;
	GstCameraSrc *camerasrc;
	camerasrc = GST_CAMERA_SRC (element);

	switch (transition) {
	case GST_STATE_CHANGE_NULL_TO_READY:
		GST_INFO_OBJECT(camerasrc, "GST CAMERA SRC: NULL -> READY");
		GST_INFO("    gst_camerasrc_create");
		if (!gst_camerasrc_create(camerasrc)){
			goto statechange_failed;
		}
		break;
	case GST_STATE_CHANGE_READY_TO_PAUSED:
		GST_INFO_OBJECT(camerasrc, "GST CAMERA SRC: READY -> PAUSED");
		ret = GST_STATE_CHANGE_NO_PREROLL;
		break;
	case GST_STATE_CHANGE_PAUSED_TO_PLAYING:
		GST_INFO_OBJECT(camerasrc, "GST CAMERA SRC: PAUSED -> PLAYING");
		break;
	default:
		break;
	}

	ret = GST_ELEMENT_CLASS(gst_camerasrc_parent_class)->change_state(element, transition);
	if (ret == GST_STATE_CHANGE_FAILURE){
		return ret;
	}

	switch (transition) {
	case GST_STATE_CHANGE_PLAYING_TO_PAUSED:
		GST_INFO_OBJECT(camerasrc, "GST CAMERA SRC: PLAYING -> PAUSED");
		ret = GST_STATE_CHANGE_NO_PREROLL;
		break;
	case GST_STATE_CHANGE_PAUSED_TO_READY:
		GST_INFO_OBJECT(camerasrc, "GST CAMERA SRC: PAUSED -> READY");
		break;
	case GST_STATE_CHANGE_READY_TO_NULL:
		GST_INFO_OBJECT(camerasrc, "GST CAMERA SRC: READY -> NULL");
		GST_INFO("    gst_camerasrc_destroy");
		if (!gst_camerasrc_destroy(camerasrc)){
			goto statechange_failed;
		}
		break;
	default:
		break;
	}

	return ret;

 statechange_failed:
	/* subclass must post a meaningfull error message */
	GST_ERROR_OBJECT(camerasrc, "state change failed");

	return GST_STATE_CHANGE_FAILURE;
}


static void gst_camerasrc_finalize(GObject *object)
{
	GstCameraSrc *camerasrc = GST_CAMERA_SRC(object);

	GST_INFO("ENTERED");

	g_cond_clear(&camerasrc->cond);
	g_cond_clear(&camerasrc->buffer_cond);
	g_cond_clear(&camerasrc->capture_cond);
	g_cond_clear(&camerasrc->restart_cond);
	g_mutex_clear(&camerasrc->jpg_mutex);
	g_mutex_clear(&camerasrc->mutex);
	g_mutex_clear(&camerasrc->buffer_lock);
	g_mutex_clear(&camerasrc->capture_mutex);
	g_mutex_clear(&camerasrc->restart_mutex);
	SAFE_FREE_GQUEUE(camerasrc->capture_cmd_list);
	SAFE_FREE_GQUEUE(camerasrc->capture_buffer_list);
	SAFE_FREE_GQUEUE(camerasrc->restart_cmd_list);

	if (G_OBJECT_CLASS (gst_camerasrc_parent_class)->finalize)
		G_OBJECT_CLASS(gst_camerasrc_parent_class)->finalize(object);

	GST_INFO("LEAVED");

	return;
}


void gst_camerasrc_set_capture_command(GstCameraSrc *camerasrc, GstCameraControlCaptureCommand cmd)
{
	gboolean is_zsl = FALSE;

	if (camerasrc == NULL) {
		GST_ERROR_OBJECT(camerasrc, "camerasrc is NULL");
		return;
	}

	GST_INFO_OBJECT(camerasrc, "ENTERED");

	g_mutex_lock(&camerasrc->mutex);

	/* Check condition of zero shutter lag routine */
	if (camerasrc->cap_fourcc == GST_MAKE_FOURCC('J','P','E','G')) {
		/* Set zero shutter lag mode */
		is_zsl = TRUE;

		if (is_zsl) {
			GST_INFO_OBJECT(camerasrc, "Set ZSL mode");

			g_mutex_lock(&camerasrc->jpg_mutex);

			if (cmd == GST_CAMERA_CONTROL_CAPTURE_COMMAND_START) {
				/* capture start */
				camerasrc->create_jpeg = TRUE;
				camerasrc->cap_count_current = 0;
				camerasrc->cap_next_time = gst_get_current_time();
			} else {
				/* capture stop */
				camerasrc->create_jpeg = FALSE;
				camerasrc->cap_count_reverse = 0;
				camerasrc->cap_next_time = 0;
			}

			g_mutex_unlock(&camerasrc->jpg_mutex);
		}
	}

	/* Do not push command when ZSL mode */
	if (!is_zsl) {
		g_queue_push_tail(camerasrc->capture_cmd_list, (gpointer)cmd);
		GST_INFO_OBJECT(camerasrc, "ACTION: Push capture command [%d] finished.", cmd);
	}

	if (cmd == GST_CAMERA_CONTROL_CAPTURE_COMMAND_STOP) {
		g_cond_signal(&camerasrc->cond);
		GST_INFO_OBJECT(camerasrc, "Send signal for CAPTURE STOP");
	}

	g_mutex_unlock(&camerasrc->mutex);

	return;
}


static gboolean
gst_camerasrc_negotiate (GstBaseSrc * basesrc)
{
	GstCaps *thiscaps;
	GstCaps *caps = NULL;
	GstCaps *peercaps = NULL;
	gboolean result = FALSE;
	GstStructure *s;
	GstCameraSrc *camerasrc = GST_CAMERA_SRC(basesrc);

	GST_INFO_OBJECT(camerasrc, "ENTERED");
	/* first see what is possible on our source pad */
	thiscaps = gst_pad_query_caps (GST_BASE_SRC_PAD (basesrc), NULL);
	GST_DEBUG_OBJECT (basesrc, "caps of src: %" GST_PTR_FORMAT, thiscaps);

	/* nothing or anything is allowed, we're done */
	if (thiscaps == NULL || gst_caps_is_any (thiscaps))
		goto no_nego_needed;

	/* get the peer caps */
	peercaps = gst_pad_peer_query_caps (GST_BASE_SRC_PAD (basesrc), NULL);
	GST_DEBUG_OBJECT (basesrc, "caps of peer: %" GST_PTR_FORMAT, peercaps);
	//LOG_CAPS (basesrc, peercaps);
	if (peercaps && !gst_caps_is_any (peercaps)) {
		GstCaps *icaps = NULL;
		int i;

		/* Prefer the first caps we are compatible with that the peer proposed */
		for (i = 0; i < gst_caps_get_size (peercaps); i++) {
			/* get intersection */
			GstCaps *ipcaps = gst_caps_copy_nth (peercaps, i);

			GST_DEBUG_OBJECT (basesrc, "peer: %" GST_PTR_FORMAT, ipcaps);
			icaps = gst_caps_intersect (thiscaps, ipcaps);
			gst_caps_unref (ipcaps);

			/*s = gst_caps_get_structure (icaps, 0);
			gst_structure_get_fourcc(s, "format", &camerasrc->fourcc);
			if ((camerasrc->fourcc == GST_MAKE_FOURCC('S','N','1','2')) ||
				(camerasrc->fourcc == GST_MAKE_FOURCC('S','T','1','2'))) {
				break;
			}*/
			if (!gst_caps_is_empty (icaps))
				break;

			gst_caps_unref (icaps);
			icaps = NULL;
		}

		GST_DEBUG_OBJECT (basesrc, "intersect: %" GST_PTR_FORMAT, icaps);
		if (icaps) {
			/* If there are multiple intersections pick the one with the smallest
			* resolution strictly bigger then the first peer caps */
			if (gst_caps_get_size (icaps) > 1) {
				s = gst_caps_get_structure (peercaps, 0);
				int best = 0;
				int twidth, theight;
				int width = G_MAXINT, height = G_MAXINT;

				if (gst_structure_get_int (s, "width", &twidth)
					&& gst_structure_get_int (s, "height", &theight)) {

					/* Walk the structure backwards to get the first entry of the
					* smallest resolution bigger (or equal to) the preferred resolution)
					*/
					for (i = gst_caps_get_size (icaps) - 1; i >= 0; i--) {
						GstStructure *is = gst_caps_get_structure (icaps, i);
						int w, h;

						if (gst_structure_get_int (is, "width", &w)
							&& gst_structure_get_int (is, "height", &h)) {
							if (w >= twidth && w <= width && h >= theight && h <= height) {
								width = w;
								height = h;
								best = i;
							}
						}
					}
				}

				caps = gst_caps_copy_nth (icaps, best);
				gst_caps_unref (icaps);
			} else {
				caps = icaps;
			}
		}
		gst_caps_unref (thiscaps);
		gst_caps_unref (peercaps);
	} else {
		/* no peer or peer have ANY caps, work with our own caps then */
		caps = thiscaps;
	}
	if (caps) {
        caps = gst_caps_fixate(caps);

		/* now fixate */
		if (!gst_caps_is_empty (caps)) {
			GST_DEBUG_OBJECT (basesrc, "fixated to: %" GST_PTR_FORMAT, caps);

			if (gst_caps_is_any (caps)) {
				/* hmm, still anything, so element can do anything and
				* nego is not needed */
				result = TRUE;
			} else if (gst_caps_is_fixed (caps)) {
				/* yay, fixed caps, use those then */
			    result = gst_camerasrc_set_caps(basesrc, caps);
			}
		}
		gst_caps_unref (caps);
	}
	return result;

no_nego_needed:
	{
		GST_DEBUG_OBJECT (basesrc, "no negotiation needed");
		if (thiscaps)
			gst_caps_unref (thiscaps);
		return TRUE;
	}
}


static GstCaps *gst_camerasrc_get_caps(GstBaseSrc *src, GstCaps *filter)
{
	GstCameraSrc *camerasrc = GST_CAMERA_SRC(src);
	GstCaps *ret = NULL;

	GST_DEBUG_OBJECT(camerasrc, "ENTERED");

	if (camerasrc->mode == VIDEO_IN_MODE_UNKNOWN) {
		GST_INFO_OBJECT(camerasrc, "Unknown mode. Just return template caps.");
		GST_DEBUG_OBJECT(camerasrc, "LEAVED");

		ret = gst_pad_get_pad_template_caps(GST_BASE_SRC_PAD(camerasrc));
		return filter ? gst_caps_intersect(ret, filter) : gst_caps_copy(ret);
	}

	/*FIXME: Using "VIDIOC_ENUM_FMT".*/
	ret = gst_caps_copy(gst_pad_get_pad_template_caps(GST_BASE_SRC_PAD(camerasrc)));

	if (filter != NULL) {
	    gst_caps_take(&ret, gst_caps_intersect(ret, filter));
	}

	GST_INFO_OBJECT(camerasrc, "probed caps: %p", ret);
	GST_DEBUG_OBJECT(camerasrc, "LEAVED");

	return ret;
}


static gboolean _gst_camerasrc_get_raw_pixel_info(int fourcc, int *pix_format, int *colorspace)
{
	switch (fourcc) {
	case GST_MAKE_FOURCC('I','4','2','0'):	/* V4L2_PIX_FMT_YUV420 */
	case GST_MAKE_FOURCC('S','4','2','0'):
		*pix_format = CAMERASRC_PIX_YUV420P;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('Y','V','1','2'):
		*pix_format = CAMERASRC_PIX_YV12;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('Y','U','Y','V'):	/* V4L2_PIX_FMT_YUYV */
	case GST_MAKE_FOURCC('Y','U','Y','2'):	/* V4L2_PIX_FMT_YUYV */
	case GST_MAKE_FOURCC('S','U','Y','V'):
	case GST_MAKE_FOURCC('S','U','Y','2'):
		*pix_format = CAMERASRC_PIX_YUY2;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('U','Y','V','Y'):	/* V4L2_PIX_FMT_UYVY */
	case GST_MAKE_FOURCC('S','Y','V','Y'):	/* V4L2_PIX_FMT_UYVY */
		*pix_format = CAMERASRC_PIX_UYVY;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('4','2','2','P'):	/* V4L2_PIX_FMT_YUV422P */
	case GST_MAKE_FOURCC('Y','4','2','B'):	/* V4L2_PIX_FMT_YUV422P */
		*pix_format = CAMERASRC_PIX_YUV422P;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('N','V','1','2'):	/* V4L2_PIX_FMT_NV12 */
		*pix_format = CAMERASRC_PIX_NV12;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('S','N','1','2'):	/* V4L2_PIX_FMT_NV12 non-linear */
		*pix_format = CAMERASRC_PIX_SN12;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('J','P','E','G'):
	case GST_MAKE_FOURCC('j','p','e','g'):
		*pix_format = CAMERASRC_PIX_RGGB8;
		*colorspace = CAMERASRC_COL_JPEG;
		break;
	case GST_MAKE_FOURCC('S','N','2','1'):
		*pix_format = CAMERASRC_PIX_SN21;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('N','V','2','1'):
		*pix_format = CAMERASRC_PIX_NV21;
		*colorspace = CAMERASRC_COL_RAW;
		break;
	case GST_MAKE_FOURCC('Y','4','1','P'):	/* V4L2_PIX_FMT_Y41P */
	case GST_MAKE_FOURCC('Y','4','1','B'):	/* V4L2_PIX_FMT_YUV411P */
	default:
		/* ERROR */
		*pix_format = CAMERASRC_PIX_NONE;
		*colorspace = CAMERASRC_COL_NONE;
		break;
	}

	return TRUE;
}


static gboolean _gst_camerasrc_get_frame_size(int fourcc, int width, int height, unsigned int *outsize)
{
	switch (fourcc) {
	case GST_MAKE_FOURCC('I','4','2','0'):	/* V4L2_PIX_FMT_YUV420 */
	case GST_MAKE_FOURCC('I','Y','U','V'):
	case GST_MAKE_FOURCC('Y','U','1','2'):
	case GST_MAKE_FOURCC('Y','V','1','2'):
	case GST_MAKE_FOURCC('S','4','2','0'):	/* V4L2_PIX_FMT_NV12 tiled non-linear */
		*outsize = GST_ROUND_UP_4 (width) * GST_ROUND_UP_2 (height);
		*outsize += 2 * ((GST_ROUND_UP_8 (width) / 2) * (GST_ROUND_UP_2 (height) / 2));
		break;
	case GST_MAKE_FOURCC('Y','U','Y','V'):	/* V4L2_PIX_FMT_YUYV */
	case GST_MAKE_FOURCC('Y','U','Y','2'):	/* V4L2_PIX_FMT_YUYV */
	case GST_MAKE_FOURCC('S','U','Y','V'):
	case GST_MAKE_FOURCC('S','U','Y','2'):
	case GST_MAKE_FOURCC('U','Y','V','Y'):	/* V4L2_PIX_FMT_UYVY */
	case GST_MAKE_FOURCC('S','Y','V','Y'):	/* V4L2_PIX_FMT_UYVY */
	case GST_MAKE_FOURCC('4','2','2','P'):	/* V4L2_PIX_FMT_YUV422P */
	case GST_MAKE_FOURCC('Y','4','2','B'):	/* V4L2_PIX_FMT_YUV422P */
	case GST_MAKE_FOURCC('Y','4','1','P'):	/* V4L2_PIX_FMT_Y41P */
		*outsize = (GST_ROUND_UP_2 (width) * 2) * height;
		break;
	case GST_MAKE_FOURCC('Y','4','1','B'):	/* V4L2_PIX_FMT_YUV411P */
		*outsize = GST_ROUND_UP_4 (width) * height;
		*outsize += 2 * ((GST_ROUND_UP_8 (width) / 4) * height);
		break;
	case GST_MAKE_FOURCC('N','V','1','2'):	/* V4L2_PIX_FMT_NV12 */
	case GST_MAKE_FOURCC('N','V','2','1'):	/* V4L2_PIX_FMT_NV21 */
	case GST_MAKE_FOURCC('S','N','1','2'):	/* V4L2_PIX_FMT_NV12 non-linear */
	case GST_MAKE_FOURCC('S','N','2','1'):	/* V4L2_PIX_FMT_NV21 non-linear */
		*outsize = GST_ROUND_UP_4 (width) * GST_ROUND_UP_2 (height);
		*outsize += (GST_ROUND_UP_4 (width) * height) / 2;
		break;
	case GST_MAKE_FOURCC('J','P','E','G'):
	case GST_MAKE_FOURCC('j','p','e','g'):
		/* jpeg size can't be calculated here. */
		*outsize = 0;
		break;
	default:
		/* unkown format!! */
		*outsize = 0;
		break;
	}

	return TRUE;
}

static gboolean _gst_camerasrc_get_normal_buffer(GstCameraSrc *camerasrc, int fourcc,
                                                 unsigned char *base_buffer, int width, int height,
                                                 GstMemory **new_buffer)
{
	unsigned char *copy_data = NULL;
	unsigned int new_buffer_length = 0;
	guint length_Y = 0;

	if (camerasrc == NULL || base_buffer == NULL || new_buffer == NULL) {
		GST_WARNING_OBJECT(camerasrc, "something is NULL(%p,%p,%p)",
		                              camerasrc, base_buffer, new_buffer);
		return FALSE;
	}

	switch (fourcc) {
	case GST_MAKE_FOURCC('I','4','2','0'):
	case GST_MAKE_FOURCC('S','4','2','0'):
	case GST_MAKE_FOURCC('Y','V','1','2'):
	{
		guint length_Cb = 0;
		guint length_Cr = 0;
		guint offset_Cb = 0;
		guint offset_Cr = 0;

		length_Y = width * height;
		length_Cb = length_Cr = (width * height) >> 2;

		copy_data = (unsigned char*)malloc(length_Y + length_Cb + length_Cr);
		if (copy_data == NULL) {
			GST_ERROR_OBJECT(camerasrc, "New buffer data ALLOC FAILED");
			gst_camerasrc_error_handler(camerasrc, CAMERASRC_ERR_ALLOCATION);
			return FALSE;
		}

		offset_Cb = CAMERASRC_ALIGN(length_Y, ALIGN_SIZE_I420);
		offset_Cr = offset_Cb + CAMERASRC_ALIGN(length_Cb, ALIGN_SIZE_I420);

		memcpy(copy_data, base_buffer, length_Y);
		memcpy(copy_data + length_Y, base_buffer + offset_Cb, length_Cb);
		memcpy(copy_data + length_Y + length_Cb, base_buffer + offset_Cr, length_Cr);

		new_buffer_length = length_Y + length_Cb + length_Cr;
		*new_buffer = gst_memory_new_wrapped(0, copy_data, new_buffer_length, 0, new_buffer_length, copy_data, free);

		GST_DEBUG_OBJECT(camerasrc, "Total(0x%x), %c%c%c%c length Y(0x%x),Cb(0x%x),Cr(0x%x), offset Cb(0x%x),Cr(0x%x)",
		                            new_buffer_length, fourcc, fourcc>>8, fourcc>>16, fourcc>>24,
		                            length_Y, length_Cb, length_Cr, offset_Cb, offset_Cr);
		break;
	}
	case GST_MAKE_FOURCC('N','V','1','2'):
	case GST_MAKE_FOURCC('S','N','1','2'):
	case GST_MAKE_FOURCC('N','V','2','1'):
	case GST_MAKE_FOURCC('S','N','2','1'):
	{
		guint length_CbCr = 0;
		guint offset_CbCr = 0;

		length_Y = width * height;
		length_CbCr = (width * height) >> 1;

		copy_data = (unsigned char*)malloc(length_Y + length_CbCr);
		if (copy_data == NULL) {
			GST_ERROR_OBJECT(camerasrc, "New buffer data ALLOC FAILED");
			gst_camerasrc_error_handler(camerasrc, CAMERASRC_ERR_ALLOCATION);
			return FALSE;
		}

		offset_CbCr = CAMERASRC_ALIGN(length_Y, ALIGN_SIZE_NV12);

		memcpy(copy_data, base_buffer, length_Y);
		memcpy(copy_data + length_Y, base_buffer + offset_CbCr, length_CbCr);

		new_buffer_length = length_Y + length_CbCr;
		*new_buffer = gst_memory_new_wrapped(0, copy_data, new_buffer_length, 0, new_buffer_length, copy_data, free);

		GST_DEBUG_OBJECT(camerasrc, "Total(0x%x), %c%c%c%c length Y(0x%x),CbCr(0x%x), offset CbCr(0x%x)",
		                            new_buffer_length, fourcc, fourcc>>8, fourcc>>16, fourcc>>24,
		                            length_Y, length_CbCr, offset_CbCr);
		break;
	}
	default:
	{
		char *pfourcc = (char*)&fourcc;
		GST_WARNING_OBJECT(camerasrc, "Unknown fourcc(%c%c%c%c)",
		                              pfourcc[0], pfourcc[1], pfourcc[2], pfourcc[3]);
		*new_buffer = NULL;
		return FALSE;
	}
	}

	GST_DEBUG_OBJECT(camerasrc, "Done.");

	return TRUE;
}


/* this function should be called after lock buffer_lock and preview_lock */
static void _gst_camerasrc_restart_preview(GstCameraSrc *camerasrc)
{
	if (!camerasrc) {
		GST_ERROR("handle is NULL");
		return;
	}

	GST_INFO("START");

	/* stop stream */
	GST_INFO("_gst_camerasrc_restart_preview : gst_camerasrc_stop");
	gst_camerasrc_stop(camerasrc);

	/* start stream */
	GST_INFO("_gst_camerasrc_restart_preview : gst_camerasrc_start");
	gst_camerasrc_start(camerasrc);

	GST_INFO("DONE");

	return;
}


static gboolean gst_camerasrc_get_caps_info(GstCameraSrc *camerasrc, GstCaps *caps, guint *size)
{
	gint fps_n = 0;
	gint fps_d = 0;
	gint w = 0;
	gint h = 0;
	gint rot = 0;
	gchar *caps_string = NULL;
	const gchar *mimetype;
	GstVideoInfo caps_info;
	const gchar *caps_format_name = NULL;
	guint32 caps_fourcc = 0;
	const GValue *framerate;
	GstStructure *structure = NULL;

	GST_INFO_OBJECT(camerasrc, "ENTERED Collect data for given caps.(caps:%p)", caps);

	structure = gst_caps_get_structure(caps, 0);

	if (!gst_structure_get_int(structure, "width", &w)) {
		goto _caps_info_failed;
	}

	if (!gst_structure_get_int(structure, "height", &h)) {
		goto _caps_info_failed;
	}

	if (!gst_structure_get_int(structure, "rotate", &rot)) {
		GST_WARNING_OBJECT(camerasrc, "Failed to get rotate info in caps. set default 0.");
		camerasrc->use_rotate_caps = FALSE;
	} else {
		GST_INFO_OBJECT(camerasrc, "Succeed to get rotate[%d] info in caps", rot);
		camerasrc->use_rotate_caps = TRUE;
	}

	/* set default size if there is no capsfilter */
	if (w == 1) {
		w = _DEFAULT_WIDTH * 2;
	}

	if (h == 1) {
		h = _DEFAULT_HEIGHT * 2;
	}

	camerasrc->width = w;
	camerasrc->height = h;
	camerasrc->rotate = rot;

	framerate = gst_structure_get_value(structure, "framerate");
	if (!framerate) {
		GST_INFO("Set FPS as default(30/1)");

		/* set default fps if framerate is not existed in caps */
		fps_n = _DEFAULT_FPS;
		fps_d = 1;
	} else {
		fps_n = gst_value_get_fraction_numerator(framerate);
		fps_d = gst_value_get_fraction_denominator(framerate);

		/* numerator and denominator should be bigger than zero */
		if (fps_n <= 0) {
			GST_WARNING("numerator of FPS is %d. make it default(30).", fps_n);
			fps_n = _DEFAULT_FPS;
		}

		if (fps_d <= 0) {
			GST_WARNING("denominator of FPS is %d. make it 1.", fps_d);
			fps_d = 1;
		}
	}

	camerasrc->fps = (int)((float)fps_n / (float)fps_d);

	mimetype = gst_structure_get_name (structure);

	*size = 0;

	if (!strcmp(mimetype, "video/x-raw")) {
		gst_video_info_init(&caps_info);
		if (gst_video_info_from_caps(&caps_info, caps) &&
		    GST_VIDEO_INFO_IS_RGB(&caps_info)) {
			switch (GST_VIDEO_FORMAT_INFO_BITS(caps_info.finfo)) {
			case 8:  /* V4L2_PIX_FMT_RGB332 */
				camerasrc->pix_format = CAMERASRC_PIX_RGGB8;
				camerasrc->colorspace = CAMERASRC_COL_RAW;
				break;
			case 15: /* V4L2_PIX_FMT_RGB555 : V4L2_PIX_FMT_RGB555X */
				camerasrc->pix_format = CAMERASRC_PIX_NONE;
				camerasrc->colorspace = CAMERASRC_COL_NONE;
				break;
			case 16: /* V4L2_PIX_FMT_RGB565 : V4L2_PIX_FMT_RGB565X */
				camerasrc->pix_format = CAMERASRC_PIX_NONE;
				camerasrc->colorspace = CAMERASRC_COL_NONE;
				break;
			case 24: /* V4L2_PIX_FMT_BGR24 : V4L2_PIX_FMT_RGB24 */
				camerasrc->pix_format = CAMERASRC_PIX_NONE;
				camerasrc->colorspace = CAMERASRC_COL_NONE;
				break;
			case 32: /* V4L2_PIX_FMT_BGR32 : V4L2_PIX_FMT_RGB32 */
				camerasrc->pix_format = CAMERASRC_PIX_NONE;
				camerasrc->colorspace = CAMERASRC_COL_NONE;
				break;
			}
		} else {
			caps_format_name = gst_structure_get_string(structure, "format");
			if (caps_format_name == NULL) {
				GST_INFO_OBJECT(camerasrc, "Getting format name is NULL.");
				goto _caps_info_failed;
			}

			strncpy(camerasrc->format_name, caps_format_name, sizeof(camerasrc->format_name));
			caps_fourcc = MAKE_FOURCC_FROM_STRING(camerasrc->format_name);
			_gst_camerasrc_get_frame_size(caps_fourcc, w, h, size);
			_gst_camerasrc_get_raw_pixel_info(caps_fourcc, &(camerasrc->pix_format), &(camerasrc->colorspace));
		}
	} else if (strcmp(mimetype, "video/x-dv") == 0) { /* V4L2_PIX_FMT_DV */
		camerasrc->pix_format = CAMERASRC_PIX_NONE;
		camerasrc->colorspace = CAMERASRC_COL_NONE;
	} else if (strcmp(mimetype, "image/jpeg") == 0) { /* V4L2_PIX_FMT_JPEG */
		camerasrc->pix_format = CAMERASRC_PIX_RGGB8; /* default */
		camerasrc->colorspace = CAMERASRC_COL_JPEG;
	}

	GST_INFO_OBJECT(camerasrc, "pixformat %d, colorspace %d, size %d",
	                           camerasrc->pix_format, camerasrc->colorspace, *size);

	caps_string = gst_caps_to_string(caps);
	if (caps_string) {
		GST_INFO_OBJECT(camerasrc, "caps : [%s]", caps_string);
		g_free(caps_string);
		caps_string = NULL;
	}

	return TRUE;

_caps_info_failed:
	GST_INFO_OBJECT(camerasrc, "Failed to get caps info.");
	GST_DEBUG_OBJECT(camerasrc, "LEAVED");
	return FALSE;
}


static void _gst_camerasrc_post_message_int(GstCameraSrc *camerasrc, const char *msg_name, const char *field_name, int value)
{
	GstMessage *m = NULL;
	GstStructure *s = NULL;

	if (!camerasrc || !msg_name || !field_name) {
		GST_ERROR("pointer is NULL %p, %p, %p", camerasrc, msg_name, field_name);
		return;
	}

	GST_INFO("post message [%s] %s %d", msg_name, field_name, value);

	s = gst_structure_new(msg_name, field_name, G_TYPE_INT, value, NULL);
	if (s == NULL) {
		GST_ERROR("gst_structure_new failed");
		gst_camerasrc_error_handler(camerasrc, CAMERASRC_ERR_ALLOCATION);
		return;
	}

	m = gst_message_new_element(GST_OBJECT(camerasrc), s);
	if (m == NULL) {
		GST_ERROR("gst_message_new_element failed");
		gst_camerasrc_error_handler(camerasrc, CAMERASRC_ERR_ALLOCATION);
		return;
	}

	gst_element_post_message(GST_ELEMENT(camerasrc), m);

	return;
}


static gboolean gst_camerasrc_set_caps(GstBaseSrc *src, GstCaps *caps)
{
	guint size;
	GstCameraSrc *camerasrc = NULL;
	gboolean res = FALSE;

	camerasrc = GST_CAMERA_SRC(src);

	GST_INFO_OBJECT(camerasrc, "ENTERED");

	if (camerasrc->mode == VIDEO_IN_MODE_PREVIEW ||
	    camerasrc->mode == VIDEO_IN_MODE_VIDEO) {
		GST_INFO_OBJECT(camerasrc, "Proceed set_caps");
		GST_INFO("            gst_camerasrc_stop");
		if (!gst_camerasrc_stop(camerasrc)) {
			GST_INFO_OBJECT(camerasrc, "Cam sensor stop failed.");
		}
	} else if (camerasrc->mode == VIDEO_IN_MODE_CAPTURE) {
		GST_ERROR_OBJECT(camerasrc, "A mode of avsystem camera is capture. Not to proceed set_caps.");
		GST_DEBUG_OBJECT(camerasrc, "LEAVED");
		return FALSE;
	} else {
		GST_INFO_OBJECT(camerasrc, "A mode of avsystem camera is unknown[%d]. Proceed set_caps.", camerasrc->mode);
	}

	/* we want our own v4l2 type of fourcc codes */
	if (!gst_camerasrc_get_caps_info(camerasrc, caps, &size)) {
		GST_INFO_OBJECT(camerasrc, "can't get capture information from caps %p", caps);
		return FALSE;
	}

	GST_INFO("            gst_camerasrc_start");
	if (!gst_camerasrc_start(camerasrc)) {
		GST_INFO_OBJECT (camerasrc,  "Cam sensor start failed.");
	}

	res = gst_pad_push_event (GST_BASE_SRC_PAD (src), gst_event_new_caps (caps));

	GST_INFO_OBJECT (camerasrc, "LEAVED");

	return res;
}


static void gst_camerasrc_class_init(GstCameraSrcClass *klass)
{
	GST_DEBUG_CATEGORY_INIT(camerasrc_debug, "camerasrc", 0, "camerasrc element");

	GObjectClass *gobject_class;
	GstElementClass *element_class;
	GstBaseSrcClass *basesrc_class;
	GstPushSrcClass *pushsrc_class;

	GST_DEBUG("ENTERED");

	gobject_class = G_OBJECT_CLASS(klass);
	element_class = GST_ELEMENT_CLASS(klass);
	basesrc_class = GST_BASE_SRC_CLASS(klass);
	pushsrc_class = GST_PUSH_SRC_CLASS(klass);

	gobject_class->set_property = gst_camerasrc_set_property;
	gobject_class->get_property = gst_camerasrc_get_property;
	gobject_class->finalize = gst_camerasrc_finalize;
	element_class->change_state = gst_camerasrc_change_state;
	gst_element_class_add_pad_template(element_class, gst_static_pad_template_get (&src_factory));
	gst_element_class_set_static_metadata(element_class,
	                                      "Camera Source GStreamer Plug-in",
	                                      "Source/Video",
	                                      "camera src for videosrc based GStreamer Plug-in",
	                                      "Jeongmo Yang <jm80.yang@samsung.com>");
	basesrc_class->start = gst_camerasrc_src_start;
	basesrc_class->stop = gst_camerasrc_src_stop;
	basesrc_class->get_caps = gst_camerasrc_get_caps;
	basesrc_class->set_caps = gst_camerasrc_set_caps;
	basesrc_class->negotiate = gst_camerasrc_negotiate;
	pushsrc_class->create = gst_camerasrc_src_create;

	g_object_class_install_property(gobject_class, ARG_CAMERA_HIGH_SPEED_FPS,
	                                g_param_spec_int("high-speed-fps", "Fps for high speed recording",
	                                                 "If this value is 0, the element doesn't activate high speed recording.",
	                                                 0, G_MAXINT, _DEFAULT_HIGH_SPEED_FPS,
	                                                 G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_CAMERA_AUTO_FPS,
	                                g_param_spec_boolean("fps-auto", "FPS Auto",
	                                                     "Field for auto fps setting",
	                                                     _DEFAULT_FPS_AUTO,
	                                                     G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_CAMERA_ID,
	                                g_param_spec_int("camera-id", "index number of camera to activate",
	                                                 "index number of camera to activate",
	                                                 _FD_MIN, _FD_MAX, 0,
	                                                 G_PARAM_READWRITE));

	/*Capture*/
	g_object_class_install_property(gobject_class, ARG_CAMERA_CAPTURE_FOURCC,
	                                g_param_spec_uint("capture-fourcc", "Capture format",
	                                                  "Fourcc value for capture format",
	                                                  0, G_MAXUINT, 0,
	                                                  G_PARAM_READWRITE));

	g_object_class_install_property(gobject_class, ARG_CAMERA_CAPTURE_WIDTH,
	                                g_param_spec_int("capture-width", "Capture width",
	                                                 "Width for camera size to capture",
	                                                 0, G_MAXINT, _DEFAULT_CAP_WIDTH,
	                                                 G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_CAMERA_CAPTURE_HEIGHT,
	                                g_param_spec_int("capture-height", "Capture height",
	                                                 "Height for camera size to capture",
	                                                 0, G_MAXINT, _DEFAULT_CAP_HEIGHT,
	                                                 G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_CAMERA_CAPTURE_INTERVAL,
	                                g_param_spec_int("capture-interval", "Capture interval",
	                                                 "Interval time to capture (millisecond)",
	                                                 0, G_MAXINT, _DEFAULT_CAP_INTERVAL,
	                                                 G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_CAMERA_CAPTURE_COUNT,
	                                g_param_spec_int("capture-count", "Capture count",
	                                                 "Capture conut for multishot",
	                                                 1, G_MAXINT, _DEFAULT_CAP_COUNT,
	                                                 G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_CAMERA_CAPTURE_JPG_QUALITY,
	                                g_param_spec_int("capture-jpg-quality", "JPEG Capture compress ratio",
	                                                 "Quality of capture image compress ratio",
	                                                 1, 100, _DEFAULT_CAP_JPG_QUALITY,
	                                                 G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_CAMERA_CAPTURE_PROVIDE_EXIF,
	                                g_param_spec_boolean("provide-exif", "Whether EXIF is provided",
	                                                     "Does capture provide EXIF?",
	                                                     _DEFAULT_CAP_PROVIDE_EXIF,
	                                                     G_PARAM_READABLE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_VFLIP,
	                                g_param_spec_boolean("vflip", "Flip vertically",
	                                                     "Flip camera input vertically",
	                                                     0,
	                                                     G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	g_object_class_install_property(gobject_class, ARG_HFLIP,
	                                g_param_spec_boolean("hflip", "Flip horizontally",
	                                                     "Flip camera input horizontally",
	                                                     0,
	                                                     G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

	/**
	* GstCameraSrc::still-capture:
	* @camerasrc: the camerasrc instance
	* @buffer: the buffer that will be pushed - Main
	* @buffer: the buffer that will be pushed - Thumbnail
	* @buffer: the buffer that will be pushed - Screennail
	*
	* This signal gets emitted before sending the buffer.
	*/
	gst_camerasrc_signals[SIGNAL_STILL_CAPTURE] =
		g_signal_new("still-capture",
		             G_TYPE_FROM_CLASS(klass),
		             G_SIGNAL_RUN_LAST,
		             G_STRUCT_OFFSET(GstCameraSrcClass, still_capture),
		             NULL,
		             NULL,
		             g_cclosure_user_marshal_VOID__OBJECT_OBJECT_OBJECT,
		             G_TYPE_NONE,
		             3, /* Number of parameter */
		             GST_TYPE_SAMPLE,  /* Main image buffer */
		             GST_TYPE_SAMPLE,  /* Thumbnail image buffer */
		             GST_TYPE_SAMPLE); /* Screennail image buffer */

	GST_DEBUG("LEAVED");

	return;
}


static void gst_camerasrc_init(GstCameraSrc *camerasrc)
{
	GST_INFO("ENTERED");

	camerasrc->v4l2_handle = NULL;
	camerasrc->mode = VIDEO_IN_MODE_UNKNOWN;
	camerasrc->firsttime = TRUE;
	camerasrc->main_buf_sz = 0;
	camerasrc->cap_count_current = -1;
	camerasrc->cap_count_reverse = _DEFAULT_CAP_COUNT;
	camerasrc->cap_next_time = 0UL;
	camerasrc->capture_cmd_list = g_queue_new();
	camerasrc->restart_cmd_list = g_queue_new();

	/*camera*/
	camerasrc->width = _DEFAULT_WIDTH;
	camerasrc->height = _DEFAULT_HEIGHT;
	camerasrc->fps = _DEFAULT_FPS;
	camerasrc->rotate = 0;
	camerasrc->use_rotate_caps = FALSE;
	camerasrc->high_speed_fps = _DEFAULT_HIGH_SPEED_FPS;
	camerasrc->fps_auto = _DEFAULT_FPS_AUTO;
	camerasrc->pix_format = _DEFAULT_PIX_FORMAT;
	camerasrc->colorspace = _DEFAULT_COLORSPACE;
	strcpy(camerasrc->format_name, _DEFAULT_PIX_FORMAT_NAME);
	camerasrc->num_live_buffers = 0;
	camerasrc->buffer_count = _DEFAULT_BUFFER_COUNT;
	camerasrc->buffer_running = FALSE;
	camerasrc->bfirst = TRUE;
	camerasrc->pad_alloc_list = g_queue_new ();
	camerasrc->current_buffer_data_index = 0;
	camerasrc->vflip = FALSE;
	camerasrc->hflip = FALSE;
	camerasrc->camera_id = _DEFAULT_CAMERA_ID;

	/*capture*/
	camerasrc->cap_fourcc = MAKE_FOURCC_FROM_STRING(_DEFAULT_CAPTURE_FORMAT_NAME);
	camerasrc->cap_width = _DEFAULT_CAP_WIDTH;
	camerasrc->cap_height = _DEFAULT_CAP_HEIGHT;
	camerasrc->cap_interval = _DEFAULT_CAP_INTERVAL;
	camerasrc->cap_count = _DEFAULT_CAP_COUNT;
	camerasrc->cap_jpg_quality = _DEFAULT_CAP_JPG_QUALITY;
	camerasrc->cap_provide_exif = _DEFAULT_CAP_PROVIDE_EXIF;
	camerasrc->create_jpeg = FALSE;

	camerasrc->first_invokation = TRUE;
	camerasrc->flash_activated = FALSE;

	camerasrc->capture_buffer_list = g_queue_new ();
	camerasrc->capture_thread = NULL;
	camerasrc->quit_capture_thread = FALSE;
	camerasrc->buffer_info = NULL;
	camerasrc->flush_cache = TRUE;

	g_mutex_init(&camerasrc->mutex);
	g_mutex_init(&camerasrc->buffer_lock);
	g_mutex_init(&camerasrc->pad_alloc_mutex);
	g_mutex_init(&camerasrc->jpg_mutex);
	g_mutex_init(&camerasrc->capture_mutex);
	g_mutex_init(&camerasrc->restart_mutex);
	g_cond_init(&camerasrc->cond);
	g_cond_init(&camerasrc->buffer_cond);
	g_cond_init(&camerasrc->capture_cond);
	g_cond_init(&camerasrc->restart_cond);

#ifdef _SPEED_UP_RAW_CAPTURE
	camerasrc->cap_stream_diff = FALSE;
#endif /* _SPEED_UP_RAW_CAPTURE */

	/* we operate in time */
	gst_base_src_set_format(GST_BASE_SRC(camerasrc), GST_FORMAT_TIME);
	gst_base_src_set_live(GST_BASE_SRC(camerasrc), TRUE);
	gst_base_src_set_do_timestamp(GST_BASE_SRC(camerasrc), TRUE);

	GST_INFO("LEAVED");

	return;
}


static unsigned long gst_get_current_time(void)
{
	struct timeval lc_time;

	gettimeofday(&lc_time, NULL);

	return ((unsigned long)(lc_time.tv_sec * 1000L) + (unsigned long)(lc_time.tv_usec / 1000L));
}


#if _ENABLE_CAMERASRC_DEBUG
#include <stdio.h>
static int __util_write_file(char *filename, void *data, int size)
{
	FILE *fp = NULL;

	fp = fopen(filename, "wb");
	if (!fp) {
		return FALSE;
	}

	fwrite(data, 1, size, fp);
	fclose(fp);

	return TRUE;
}
#endif


static gboolean plugin_init(GstPlugin *plugin)
{
	gboolean error;

	error = gst_element_register(plugin, "camerasrc", GST_RANK_PRIMARY + 100, GST_TYPE_CAMERA_SRC);

	return error;
}


GST_PLUGIN_DEFINE(GST_VERSION_MAJOR,
                  GST_VERSION_MINOR,
                  camerasrc,
                  "Camera source plug-in",
                  plugin_init,
                  PACKAGE_VERSION,
                  "LGPL",
                  "Samsung Electronics Co",
                  "http://www.samsung.com")

/* GstURIHandler interface */
static GstURIType
gst_camerasrc_uri_get_type (GType type)
{
	return GST_URI_SRC;
}

static const gchar * const*
gst_camerasrc_uri_get_protocols (GType type)
{
	static const gchar *protocols[] = { "camera", NULL };
	return protocols;
}

static gchar *
gst_camerasrc_uri_get_uri (GstURIHandler * handler)
{
	return strdup("camera://0");
}

static gboolean
gst_camerasrc_uri_set_uri (GstURIHandler * handler, const gchar * uri, GError **error)
{
	GstCameraSrc *camerasrc = GST_CAMERA_SRC (handler);
	const gchar *device = "0";
	if (strcmp (uri, "camera://") != 0) {
		device = uri + 9;
	}
	g_object_set (camerasrc, "camera-id", atoi(device), NULL);

	return TRUE;
}


static void
gst_camerasrc_uri_handler_init (gpointer g_iface, gpointer iface_data)
{
	GstURIHandlerInterface *iface = (GstURIHandlerInterface *) g_iface;

	iface->get_type = gst_camerasrc_uri_get_type;
	iface->get_protocols = gst_camerasrc_uri_get_protocols;
	iface->get_uri = gst_camerasrc_uri_get_uri;
	iface->set_uri = gst_camerasrc_uri_set_uri;
}
/* EOF */
