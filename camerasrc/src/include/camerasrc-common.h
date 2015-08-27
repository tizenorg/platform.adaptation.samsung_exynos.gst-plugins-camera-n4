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

#ifndef __CAMERASRC_COMMON_H__
#define __CAMERASRC_COMMON_H__

#include <stdio.h>
#include <malloc.h>
#include <pthread.h>
#include <errno.h>      /*EXXX*/
#include <sys/ioctl.h>  /*ioctl*/
#include <string.h>     /*memcpy*/

#include <sys/types.h>  /*open*/
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>     /*mmap*/
#include <sys/mman.h>   /*alloc series, free..*/
#include <sys/time.h>   /*gettimeofday*/
#include <math.h>       /*log2*/
#include <gst/gst.h>

#undef __ASM_ARM_TYPES_H
#undef __ASSEMBLY_
#undef _I386_TYPES_H

#include <asm/types.h>
#include <linux/videodev2.h>    /* V4L2 APIs */
#include <linux/media.h>

#include "camerasrc.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Memory utility definitions
 */
#if !defined (PAGE_SHIFT)
    #define PAGE_SHIFT sysconf(_SC_PAGESIZE)
#endif
#if !defined (PAGE_SIZE)
    #define PAGE_SIZE (1UL << PAGE_SHIFT)
#endif
#if !defined (PAGE_MASK)
    #define PAGE_MASK (~(PAGE_SIZE-1))
#endif

#define PAGE_ALIGN(addr)    (((addr)+PAGE_SIZE-1)&PAGE_MASK)
#define CLEAR(x)            memset (&(x), 0, sizeof (x))

#define CAMERASRC_MAX_WIDTH                     2560
#define CAMERASRC_MAX_HEIGHT                    1920
#define CAMERASRC_CID_NOT_SUPPORT               -1
#define CAMERASRC_USRPTR_MAX_BUFFER_NUM         12
#define CAMERASRC_ERRMSG_MAX_LEN                128
#define CAMERASRC_PRIMARY_BASIC_INFO_PATH       "/tmp/.camprimarybasicinfo"
#define CAMERASRC_PRIMARY_MISC_INFO_PATH        "/tmp/.camprimarymiscinfo"
#define CAMERASRC_PRIMARY_EXTRA_INFO_PATH        "/tmp/.camprimaryextrainfo"
#define CAMERASRC_SECONDARY_BASIC_INFO_PATH     "/tmp/.camsecondarybasicinfo"
#define CAMERASRC_SECONDARY_MISC_INFO_PATH      "/tmp/.camsecondarymiscinfo"
#define CAMERASRC_SECONDARY_EXTRA_INFO_PATH      "/tmp/.camsecondaryextrainfo"
#define CAMERASRC_MAX_IMAGE_BUFFER_PLANES	3

//#define USE_OPEN_CHK                            /*< Using open check with temporary file */
#define USE_SENSOR_MODE                         0
/*#define USE_SKIP_FRAME*/                        /*< Skip frame toggle */
/*#define USE_IOCTL_DEBUG*/                       /*< For debugging ioctl name, argument, address, etc */
/*#define USE_FRAME_COPY_BOUNDARY_CHECK*/         /*< Copy boundary checks occurs seg fault when overrun */
/*#define USE_SKIP_FRAME_AT_RAW_FRAME*/           /*< In pumping raw frame, initial 2-3 frames are darker. so skip it */
/*#define USE_CAMERASRC_FRAME_DUMP*/              /*< Debug system annoying me. Use printf!!!! */
/*#define USE_USERPTR_DEBUG*/
/*#define ENABLE_Q_ERROR*/

#ifndef GST_CAT_DEFAULT
GST_DEBUG_CATEGORY_EXTERN(camerasrc_debug);
#define GST_CAT_DEFAULT camerasrc_debug
#endif /* GST_CAT_DEFAULT */


#define camsrc_log(msg, args...)           GST_LOG(msg, ##args)
#define camsrc_debug(msg, args...)         GST_DEBUG(msg, ##args)
#define camsrc_info(msg, args...)          GST_INFO(msg, ##args)
#define camsrc_warning(msg, args...)       GST_WARNING(msg, ##args)
#define camsrc_error(msg, args...)         GST_ERROR(msg, ##args)
#define camsrc_critical(msg, args...)      GST_ERROR(msg, ##args)
#define camsrc_assert(condition) { \
	if (!(condition)) { \
		GST_ERROR("failed [%s]", #condition); \
	} \
}

#define SET_CTRL_VAL(cid, in_value) {\
	int err = CAMERASRC_ERR_UNKNOWN;\
	struct v4l2_control control;\
	control.id = cid;\
	control.value = in_value;\
	camsrc_log("[VIDIOC_S_CTRL] >> [%x] request with value %d", cid, in_value); \
	err = _camerasrc_ioctl(handle, handle->fd_sensor, VIDIOC_S_CTRL, &control);\
	if(err != CAMERASRC_SUCCESS) {\
		return err;\
	}\
}

#define GET_CTRL_VAL(cid, ret_value) {\
	int err = CAMERASRC_ERR_UNKNOWN;\
	struct v4l2_control control;\
	control.id = cid;\
	err = _camerasrc_ioctl(handle, handle->fd_sensor, VIDIOC_G_CTRL, &control);\
	if(err != CAMERASRC_SUCCESS) {\
		return err;\
	}\
	ret_value = control.value;\
	camsrc_log("[VIDIOC_G_CTRL] << [%x] request with value %d", cid, ret_value); \
}

#define SET_CTRL_VAL_ERR(cid, in_value, err) {\
	struct v4l2_control control;\
	control.id = cid;\
	control.value = in_value;\
	camsrc_log("[VIDIOC_S_CTRL] >> [%x] request with value %d", cid, in_value); \
	_camerasrc_ioctl_with_err(handle, VIDIOC_S_CTRL, &control, &err);\
}

#define GET_CTRL_VAL_ERR(cid, ret_value, err) {\
	struct v4l2_control control;\
	control.id = cid;\
	_camerasrc_ioctl_with_err(handle, VIDIOC_G_CTRL, &control, &err);\
	ret_value = control.value;\
	camsrc_log("[VIDIOC_G_CTRL] << [%x] request with value %d", cid, ret_value); \
}

/*
 * Values for internal
 */
enum camerasrc_op_mode_t {
    CAMERASRC_OP_PREVIEW = 0,
    CAMERASRC_OP_CAPTURE,
    CAMERASRC_OP_VIDEO,
    CAMERASRC_OP_REGISTER_VALUE,
    CAMERASRC_OP_NUM,
};

/*
 * Values for internal
 */
enum camerasrc_ctrl_property_t{
    CAMERASRC_CTRL_SUPPORT = 0,
    CAMERASRC_CTRL_MAX_VALUE,
    CAMERASRC_CTRL_MIN_VALUE,
    CAMERASRC_CTRL_CID_VALUE,
    CAMERASRC_CTRL_CURRENT_VALUE,
    CAMERASRC_CTRL_PROPERTY_NUM,
};

/*
 * Values for internal
 */
enum camerasrc_quality_t{
    CAMERASRC_QUALITY_NORMAL = 0,
    CAMERASRC_QUALITY_HIGH,
    CAMERASRC_QUALITY_NUM,
};

enum camerasrc_dev_recog_t{
    CAMERASRC_DEV_RECOG_ID = 0,
    CAMERASRC_DEV_RECOG_INDEX,
    CAMERASRC_DEV_RECOG_NUM,
};

/**
 * Phase, camerasrc consist of two phase, running and non-running.
 */
typedef enum {
    CAMERASRC_PHASE_RUNNING = 0,
    CAMERASRC_PHASE_NON_RUNNING,
    CAMERASRC_PHASE_NUM,
} _camerasrc_phase_t;

typedef enum {
    CAMERASRC_MISC_STILL_SIGNAL = 0,
    CAMERASRC_MISC_SKIP_FRAME,
    CAMERASRC_MISC_FUNC_NUM,
} _camerasrc_misc_func_t;

typedef enum{
    _CAMERASRC_CMD_AF_CONTROL = 0,
    _CAMERASRC_CMD_AF_AREA,
    _CAMERASRC_CMD_STROBE_MODE,
    _CAMERASRC_CMD_SHUTTER_SPEED,
    _CAMERASRC_CMD_EXPOSURE_VALUE,
    _CAMERASRC_CMD_FRAME_DATA,
    _CAMERASRC_CMD_EXIF_INFO,
    _CAMERASRC_CMD_CTRL,
    _CAMERASRC_CMD_ROTATION,
    _CAMERASRC_CMD_VFLIP,
    _CAMERASRC_CMD_HFLIP,
    _CAMERASRC_CMD_NUM,
}_camsrc_cmd_t;

typedef struct{
    int cid;
    int value;
} _camerasrc_ctrl_t;

enum {
    _CAMERASRC_AF_START = 0,
    _CAMERASRC_AF_STOP,
    _CAMERASRC_AF_RELEASE,
    _CAMERASRC_AF_INIT,
    _CAMERASRC_AF_DESTROY,
    _CAMERASRC_AF_RESULT,
};

// U T I L I T Y   D E F I N I T I O N
/**
 * Utility definitions
 */
#define CAMERASRC_SET_STATE(handle, state) { \
	handle->prev_state = handle->cur_state; \
	handle->cur_state = state; \
	camsrc_info("Set state [%d] -> [%d]", handle->prev_state, handle->cur_state); \
}
#define CAMERASRC_SET_PHASE(handle, phase)      handle->cur_phase = phase;
#define CAMERASRC_STATE(handle)                 (handle->cur_state)
#define CAMERASRC_PREV_STREAM_STATE(handle)     -1
#define CAMERASRC_PHASE(handle)                 (handle->cur_phase)
#define CAMERASRC_HANDLE(handle)                ((camerasrc_handle_t*) handle)
#define CAMERASRC_CURRENT_DEV_ID(handle)        (handle->dev_id)

#define YUV422_SIZE(handle) ((handle->format.img_size.dim.height * handle->format.img_size.dim.width) << 1)
#define YUV420_SIZE(handle) ((handle->format.img_size.dim.height * handle->format.img_size.dim.width * 3) >> 1)
#define RGB565_SIZE(handle) ((handle->format.img_size.dim.height * handle->format.img_size.dim.width) << 1)

#define ISO_APPROXIMATE_VALUE(iso_in, iso_approximated) { \
	if(iso_in > 8.909 && iso_in <= 11.22) iso_approximated = 10; \
	else if(iso_in > 11.22 && iso_in <= 14.14) iso_approximated = 12; \
	else if(iso_in > 14.14 && iso_in <= 17.82) iso_approximated = 16; \
	else if(iso_in > 17.82 && iso_in <= 22.45) iso_approximated = 20; \
	else if(iso_in > 22.45 && iso_in <= 28.28) iso_approximated = 25; \
	else if(iso_in > 28.28 && iso_in <= 35.64) iso_approximated = 32; \
	else if(iso_in > 35.64 && iso_in <= 44.90) iso_approximated = 40; \
	else if(iso_in > 44.90 && iso_in <= 56.57) iso_approximated = 50; \
	else if(iso_in > 56.57 && iso_in <= 71.27) iso_approximated = 64; \
	else if(iso_in > 71.27 && iso_in <= 89.09) iso_approximated = 80; \
	else if(iso_in > 89.09 && iso_in <= 112.2) iso_approximated = 100; \
	else if(iso_in > 112.2 && iso_in <= 141.4) iso_approximated = 125; \
	else if(iso_in > 141.4 && iso_in <= 178.2) iso_approximated = 160; \
	else if(iso_in > 178.2 && iso_in <= 224.5) iso_approximated = 200; \
	else if(iso_in > 224.5 && iso_in <= 282.8) iso_approximated = 250; \
	else if(iso_in > 282.8 && iso_in <= 356.4) iso_approximated = 320; \
	else if(iso_in > 356.4 && iso_in <= 449.0) iso_approximated = 400; \
	else if(iso_in > 449.0 && iso_in <= 565.7) iso_approximated = 500; \
	else if(iso_in > 565.7 && iso_in <= 712.7) iso_approximated = 640; \
	else if(iso_in > 712.7 && iso_in <= 890.9) iso_approximated = 800; \
	else if(iso_in > 890.9 && iso_in <= 1122) iso_approximated = 1000; \
	else if(iso_in > 1122 && iso_in <= 1414) iso_approximated = 1250; \
	else if(iso_in > 1414 && iso_in <= 1782) iso_approximated = 1600; \
	else if(iso_in > 1782 && iso_in <= 2245) iso_approximated = 2000; \
	else if(iso_in > 2245 && iso_in <= 2828) iso_approximated = 2500; \
	else if(iso_in > 2828 && iso_in <= 3564) iso_approximated = 3200; \
	else if(iso_in > 3564 && iso_in <= 4490) iso_approximated = 4000; \
	else if(iso_in > 4490 && iso_in <= 5657) iso_approximated = 5000; \
	else if(iso_in > 5657 && iso_in <= 7127) iso_approximated = 6400; \
	else if(iso_in > 7127 && iso_in <= 8909) iso_approximated = 8000; \
	else { \
		camsrc_warning("Invalid parameter(Maybe kernel failure).. give default value, 100");\
		iso_approximated = 100;\
	}\
}

#define PHOTOMETRY_MODE_TO_METERING_MODE(photometry_mode, metering_mode) { \
	if(photometry_mode == V4L2_EXPOSURE_METERING_MATRIX) metering_mode = 1; \
	else if (photometry_mode == V4L2_EXPOSURE_METERING_CENTER_WEIGHTED) metering_mode = 2; \
	else if (photometry_mode == V4L2_EXPOSURE_METERING_SPOT) metering_mode = 3; \
	else metering_mode = 1; \
}

#define CAMERASRC_EXIF_SHUTTERSPEED_VALUE_IN_APEX(NUM, DEN) (int)(-(log2((double)((double)NUM/(double)DEN))))
#define CAMERASRC_EXIF_APERTURE_VALUE_IN_APEX(NUM, DEN)     (int)(2 * (log2((double)((double)NUM/(double)DEN))) + 0.5)

/* for media device */
#define LENGTH_DEV_NAME                 32

typedef struct _media_entity_t media_entity_t;
typedef struct _media_pad_t    media_pad_t;
typedef struct _media_link_t   media_link_t;
typedef struct _media_device_t media_device_t;

struct _media_entity_t {
	media_device_t *media;
	struct media_entity_desc info;
	media_pad_t *pads;
	media_link_t *links;
	unsigned int max_links;
	unsigned int num_links;
	char devname[LENGTH_DEV_NAME];
	int fd;
	__u32 padding[6];
};

struct _media_pad_t {
	media_entity_t *entity;
	__u32 index;
	__u32 flags;
	__u32 padding[3];
};

struct _media_link_t {
	media_pad_t *source;
	media_pad_t *sink;
	media_link_t *twin;
	__u32 flags;
	__u32 padding[3];
};

struct _media_device_t {
	int fd;
	media_entity_t *entities;
	unsigned int entities_count;
	void (*debug_handler)(void *, ...);
	void *debug_priv;
	__u32 padding[6];
};


typedef void *(*camerasrc_signal_func_t) (camsrc_handle_t handle);
typedef int (*camerasrc_skip_frame_func_t) (camsrc_handle_t handle, long int timeout, int skip_frame);

typedef struct _camerasrc_handle_t {
	/* device information */
	int cur_dev_id;
	int errnum;

	/* fd list */
	int fd_companion;
	int fd_sensor;
	int fd_isp;
	int fd_3aa1;
	int fd_3aa1c;
	int fd_scc;
	int fd_scp;

	int streamon_sensor;
	int streamon_3aa1_output;
	int streamon_3aa1_capture;
	int streamon_isp;
	int streamon_scc;
	int streamon_scp;
	int streamon_sensor_sub;

	int reqbuf_sensor;
	int reqbuf_3aa1_output;
	int reqbuf_3aa1_capture;
	int reqbuf_isp;
	int reqbuf_scc;
	int reqbuf_scp;

	int request_count;

	/* state information */
	int prev_stream_state;
	int prev_state;
	int cur_state;
	int cur_phase;

	/* image format information */
	int is_highquality;
	camerasrc_format_t format_sensor;
	camerasrc_format_t format_3aa1;
	camerasrc_format_t format_isp;
	camerasrc_format_t format_scc;
	camerasrc_format_t format_scp;

	/* buffer information */
	guint buffer_idx;
	guint queued_buffer_count;
	int first_frame;
	struct v4l2_buffer queued_buf_list[CAMERASRC_USRPTR_MAX_BUFFER_NUM];
	camerasrc_buffer_t *buffer_sensor;
	camerasrc_buffer_t *buffer_isp;
	camerasrc_buffer_t *buffer_scc;
	camerasrc_buffer_t *buffer_scp;
	camerasrc_buffer_t scrnl_buf;  /* screennail buffer of captured JPEG image */

	/* autofocusing information */
	camerasrc_af_mode_t cur_af_mode;
	camerasrc_af_scan_range_t cur_af_range;
	camerasrc_callback_t af_cb;
	pthread_t focusing_thread;
	pthread_cond_t af_wait_cond;
	camerasrc_auto_focus_status_t af_status;
	camerasrc_auto_focus_cmd_t af_cmd;
	int af_dev_val;
	void *af_usr_data;

	/* Jpg Still information */
	camerasrc_exif_info_t current_exif;     /* EXIF info of current captured image */

	/* fps */
	camerasrc_frac_t timeperframe;

	/* flip */
	int vflip;
	int hflip;

	/* thread safe mechanism */
	pthread_mutex_t mutex;
	pthread_mutex_t af_mutex;
	pthread_cond_t cond;

	/* TBM */
	tbm_bufmgr bufmgr;
} camerasrc_handle_t;

typedef struct {
    int (*_ioctl) (camerasrc_handle_t *handle, int fd, int request, void *arg);
    int (*_ioctl_once) (camerasrc_handle_t *handle, int request, void *arg);
    void *(*_run_autofocusing) (camerasrc_handle_t *handle);
    int (*_set_cmd) (camerasrc_handle_t *handle, _camsrc_cmd_t cmd, void *value);
    int (*_get_cmd) (camerasrc_handle_t *handle, _camsrc_cmd_t cmd, void *value);
} CAMERASRC_DEV_DEPENDENT_MISC_FUNC;

#ifdef __cplusplus
}
#endif

#endif /*__CAMERASRC_COMMON_H__*/
