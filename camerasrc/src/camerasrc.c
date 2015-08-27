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

#include <stdint.h>
#include <unistd.h>
#include <poll.h>
#include <exynos_drm.h>
#include <libdrm/drm.h>
#include <linux/v4l2-subdev.h>

#include "camerasrc-common.h"
#include "camerasrc-internal.h"
#include "fimc-is-metadata.h"


#define S_FMT_COUNT                     2
#define BUF_LEN                         64

/*
 * LOCAL DEFINITIONS
 */
#ifndef EXPORT_API
#define EXPORT_API __attribute__((__visibility__("default")))
#endif

#define LOCK(p) {\
    if(0 != pthread_mutex_lock(&(p->mutex))) {\
        camsrc_error("Mutex locking error");\
        camsrc_assert(0);\
    }\
}

#define UNLOCK(p) {\
    if(0 != pthread_mutex_unlock(&(p->mutex))) {\
        camsrc_error("Mutex unlocking error");\
        camsrc_assert(0);\
    }\
}

/* Extended CID */
#ifndef V4L2_CID_FIMC_IS_BASE
#define V4L2_CID_FIMC_IS_BASE			(V4L2_CTRL_CLASS_CAMERA | 0x1000)
#endif /* V4L2_CID_FIMC_IS_BASE */
#ifndef V4L2_CID_IS_SET_SETFILE
#define V4L2_CID_IS_SET_SETFILE			(V4L2_CID_FIMC_IS_BASE + 51)
#endif /* V4L2_CID_IS_SET_SETFILE */
#ifndef V4L2_CID_IS_COLOR_RANGE
#define V4L2_CID_IS_COLOR_RANGE			(V4L2_CID_FIMC_IS_BASE + 54)
#endif /* V4L2_CID_IS_COLOR_RANGE */
#ifndef V4L2_CID_IS_MIN_TARGET_FPS
#define V4L2_CID_IS_MIN_TARGET_FPS		(V4L2_CID_FIMC_IS_BASE + 55)
#endif /* V4L2_CID_IS_MIN_TARGET_FPS */
#ifndef V4L2_CID_IS_MAX_TARGET_FPS
#define V4L2_CID_IS_MAX_TARGET_FPS		(V4L2_CID_FIMC_IS_BASE + 56)
#endif /* V4L2_CID_IS_MAX_TARGET_FPS */
#ifndef V4L2_CID_IS_S_STREAM
#define V4L2_CID_IS_S_STREAM			(V4L2_CID_FIMC_IS_BASE + 14)
#endif /* V4L2_CID_IS_S_STREAM */


/* FIMC IS NUM */
#define FIMC_IS_VIDEO_3A1_NUM	14
#define FIMC_IS_VIDEO_3A1P_NUM	16
#define FIMC_IS_VIDEO_ISP_NUM	30
#define FIMC_IS_VIDEO_SCC_NUM	34
#define FIMC_IS_VIDEO_SCP_NUM	37

/* struct camera2_internal_udm */
#define FIMC_IS_ISP_VS2_INIT	0x003F8CE4
#define FIMC_IS_ISP_VS2_STEP	0x8350
#define FIMC_IS_ISP_VS2_RANGE	40

/* Magic Number */
#define METADATA_MAGIC_NUMBER	0x23456789

/* GROUP ID */
#define GRP_SENSOR	0x01
#define GRP_3AA		0x02
#define GRP_3AAP	0x03
#define GRP_ISP		0x04
#define GRP_DIS		0x05
#define GRP_SCC		0x06
#define GRP_SCP		0x07


extern const CAMERASRC_DEV_DEPENDENT_MISC_FUNC *dev_misc_func;

/** proto type of internal function **/
static int buf_4k_align(unsigned int buf_size);
static int __camerasrc_open_device(camerasrc_handle_t *p, camerasrc_dev_id_t camera_id);
static void __camerasrc_close_device(camerasrc_handle_t *p);
int _camerasrc_ioctl(camerasrc_handle_t *handle, int fd, int request, void *arg);
int _camerasrc_ioctl_once(camerasrc_handle_t *handle, int request, void *arg);
void *_camerasrc_run_autofocusing(camerasrc_handle_t *handle);
int _camerasrc_ioctl_s_ctrl(camerasrc_handle_t *p, int fd, guint cid, int value);
int _camerasrc_ioctl_g_ctrl(camerasrc_handle_t *p, int fd, guint cid, int *value);
int _camerasrc_ioctl_s_parm(camerasrc_handle_t *p, int fd, int type, int numerator, int denominator);
int _camerasrc_ioctl_s_fmt(camerasrc_handle_t *p, int fd, int type, camerasrc_format_t *format);
int _camerasrc_ioctl_reqbufs(camerasrc_handle_t *p, int fd, guint count, int type, int memory, guint *ret_count);
int _camerasrc_ioctl_qbuf(camerasrc_handle_t *p, int fd, int type, int memory, int idx, camerasrc_buffer_t *buffer);
int _camerasrc_ioctl_dqbuf(camerasrc_handle_t *p, int fd, int type, int memory, int *idx, int num_planes);
int _camerasrc_ioctl_stream(camerasrc_handle_t *p, int fd, int type, int on);


/****  A U T O F O C U S I N G   F U N C T I O N S  ****/
int _camerasrc_set_autofocusing_area(camerasrc_handle_t *handle, camerasrc_rect_t *rect);
int _camerasrc_get_autofocusing_area(camerasrc_handle_t *handle, camerasrc_rect_t *rect);
int _camerasrc_start_autofocusing(camerasrc_handle_t *handle);
int _camerasrc_stop_autofocusing(camerasrc_handle_t *handle);
int _camerasrc_release_autofocusing(camerasrc_handle_t *handle);
int _camerasrc_destroy_autofocusing(camerasrc_handle_t *handle);
int _camerasrc_init_autofocusing_mode(camerasrc_handle_t *handle);
int _camerasrc_get_autofocusing_result(camerasrc_handle_t *handle);

int _camerasrc_get_frame_data(camerasrc_handle_t *handle, camerasrc_frame_data_t *data);
int _camerasrc_get_exif_info(camerasrc_handle_t *handle, camerasrc_buffer_t *exif_string);

static int _camerasrc_set_shot_meta(camerasrc_handle_t *p, unsigned int grpid, int fcount,
                                    unsigned long long timestamp, struct camera2_shot_ext *shot_ext);


/* media definitions */
#define VIDEO_DEVICE_NODE_PRIMARY       "/dev/video1"
#define VIDEO_DEVICE_NODE_SECONDARY     "/dev/video101"
#define VIDEO_DEVICE_NODE_COMPANION     "/dev/video109"
#define VIDEO_DEVICE_NODE_ISP           "/dev/video130"
#define VIDEO_DEVICE_NODE_3AA1          "/dev/video114"
#define VIDEO_DEVICE_NODE_3AA1C         "/dev/video115"
#define VIDEO_DEVICE_NODE_SCC           "/dev/video134"
#define VIDEO_DEVICE_NODE_SCP           "/dev/video137"

#define SENSOR_MARGIN_WIDTH             16
#define SENSOR_MARGIN_HEIGHT            16
#define FRONT_SENSOR_SIZE_WIDTH         2560
#define FRONT_SENSOR_SIZE_HEIGHT        1440

#define CAMERASRC_SET_CMD(cmd, value) _camerasrc_set_cmd(handle, cmd, (void*)value);
#define CAMERASRC_GET_CMD(cmd, value) _camerasrc_get_cmd(handle, cmd, (void*)value);

static int buf_4k_align(unsigned int buf_size)
{
	unsigned int ret;

	if (buf_size & (PAGE_SIZE - 1)) {
		camsrc_info("buf_size(0x%08x) is not 4K aligned", buf_size);
		ret = buf_size + PAGE_SIZE;
		ret &= PAGE_MASK;
	} else {
		camsrc_info("buf_size(0x%08x) is 4K aligned", buf_size);
		ret = buf_size;
	}

	return ret;

}


static int _camerasrc_set_cmd(camsrc_handle_t handle, _camsrc_cmd_t cmd, void *value)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (dev_misc_func->_set_cmd == NULL) {
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}

	err = dev_misc_func->_set_cmd(p, cmd, value);

	return err;
}

static int _camerasrc_get_cmd(camsrc_handle_t handle, _camsrc_cmd_t cmd, void *value)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if(dev_misc_func->_get_cmd == NULL)
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;

	err = dev_misc_func->_get_cmd(p, cmd, value);

	return err;
}


int camerasrc_extract_exif_info_from_capture_data(camsrc_handle_t handle, camerasrc_capture_data_info *capture_data)
{
	camerasrc_handle_t *p = CAMERASRC_HANDLE(handle);

	if (!p || !capture_data) {
		camsrc_error("pointer is NULL");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	if (TRUE) {
		camsrc_error("set default.");

		p->current_exif.shutter_speed.numerator = 1;
		p->current_exif.shutter_speed.denominator = EXIF_DEFAULT_SHUTTER_SPEED_DENOMINATOR;
		p->current_exif.exposure_time.numerator = EXIF_DEFAULT_EXPOSURE_TIME_NUMERATOR;
		p->current_exif.exposure_time.denominator = 1;
		p->current_exif.brightness.numerator = 1;
		p->current_exif.brightness.denominator = EXIF_DEFAULT_BRIGHTNESS_DENOMINATOR;
		p->current_exif.iso_speed_rating = EXIF_DEFAULT_ISO_SPEED_RATING;

		return CAMERASRC_ERR_NULL_POINTER;
	}

	/* flash */
	if (capture_data->flash_activated) {
		p->current_exif.flash = TRUE;
	} else {
		p->current_exif.flash = FALSE;
	}

	p->current_exif.shutter_speed.denominator = EXIF_DEFAULT_SHUTTER_SPEED_DENOMINATOR;
	p->current_exif.exposure_time.numerator = EXIF_DEFAULT_EXPOSURE_TIME_NUMERATOR;
	p->current_exif.brightness.denominator = EXIF_DEFAULT_BRIGHTNESS_DENOMINATOR;

	if (p->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY) {
		/* TODO */
		/* shutter speed */
		/* exposure time */
		/* brightness */
		/* iso */
	} else {
		/* TODO */
		/* shutter speed */
		/* exposure time */
		/* brightness */
		/* iso */
	}

	camsrc_info("flash %d, shutter_speed %d/%d, exposure_time %d/%d, brightness %d/%d, iso_speed_rating %d",
	            p->current_exif.flash,
	            p->current_exif.shutter_speed.numerator, p->current_exif.shutter_speed.denominator,
	            p->current_exif.exposure_time.numerator, p->current_exif.exposure_time.denominator,
	            p->current_exif.brightness.numerator, p->current_exif.brightness.denominator,
	            p->current_exif.iso_speed_rating);

	return CAMERASRC_SUCCESS;
}


int camerasrc_get_screennail_buffer(camsrc_handle_t handle, camerasrc_buffer_t *scrnl_buf)
{
	camerasrc_handle_t *p = NULL;

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	if (scrnl_buf == NULL) {
		camsrc_error("scrnl_buf is null");
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	p = CAMERASRC_HANDLE(handle);

	scrnl_buf->planes[0].start = p->scrnl_buf.planes[0].start;
	scrnl_buf->planes[0].length = p->scrnl_buf.planes[0].length;

	camsrc_info("screennail ptr[%p],length[%d]", scrnl_buf->planes[0].start, scrnl_buf->planes[0].length);

	return CAMERASRC_SUCCESS;
}


int camerasrc_set_shutter_speed(camsrc_handle_t handle, camerasrc_frac_t frac)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_SHUTTER_SPEED, &frac);
}


int camerasrc_get_shutter_speed(camsrc_handle_t handle, camerasrc_frac_t *frac)
{
	return CAMERASRC_GET_CMD(_CAMERASRC_CMD_SHUTTER_SPEED, frac);
}


int camerasrc_set_exposure_value(camsrc_handle_t handle, camerasrc_frac_t frac)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_EXPOSURE_VALUE, &frac);
}


int camerasrc_get_exposure_value(camsrc_handle_t handle, camerasrc_frac_t *frac)
{
	return CAMERASRC_GET_CMD(_CAMERASRC_CMD_EXPOSURE_VALUE, frac);
}


int camerasrc_set_strobe_mode(camsrc_handle_t handle, camerasrc_strobe_mode_t mode)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_STROBE_MODE, &mode);
}


int camerasrc_get_strobe_mode(camsrc_handle_t handle, camerasrc_strobe_mode_t *mode)
{
	return CAMERASRC_GET_CMD(_CAMERASRC_CMD_STROBE_MODE, mode);
}


int camerasrc_set_control(camsrc_handle_t handle, camerasrc_ctrl_t ctrl_id, int value)
{
	_camerasrc_ctrl_t ctrl;

	CLEAR(ctrl);

	ctrl.cid = ctrl_id;
	ctrl.value = value;

	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_CTRL, &ctrl);
}


int camerasrc_get_control(camsrc_handle_t handle, camerasrc_ctrl_t ctrl_id, int *value)
{
	int err = CAMERASRC_ERR_UNKNOWN;
	_camerasrc_ctrl_t ctrl;

	CLEAR(ctrl);

	ctrl.cid = ctrl_id;

	err = CAMERASRC_GET_CMD(_CAMERASRC_CMD_CTRL, &ctrl);
	if (err != CAMERASRC_SUCCESS) {
		return err;
	}

	*value = ctrl.value;

	return err;
}


int _camerasrc_ioctl(camerasrc_handle_t *handle, int fd, int request, void *arg)
{
	if (dev_misc_func->_ioctl == NULL) {
#if USE_NOT_SUPPORT_ERR
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
#else
		return CAMERASRC_SUCCESS;
#endif
	}

	return dev_misc_func->_ioctl(handle, fd, request, arg);
}


int _camerasrc_ioctl_once(camerasrc_handle_t *handle, int request, void *arg)
{
	if (dev_misc_func->_ioctl_once == NULL) {
#if USE_NOT_SUPPORT_ERR
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
#else
		return CAMERASRC_SUCCESS;
#endif
	}

	return dev_misc_func->_ioctl_once(handle, request, arg);
}


void *_camerasrc_run_autofocusing(camerasrc_handle_t *handle)
{
	if (dev_misc_func->_run_autofocusing != NULL) {
		dev_misc_func->_run_autofocusing(handle);
	}

	return NULL;
}


int _camerasrc_ioctl_s_ctrl(camerasrc_handle_t *p, int fd, guint cid, int value)
{
	struct v4l2_control ctrl;

	if (p == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	ctrl.id = cid;
	ctrl.value = value;

	return _camerasrc_ioctl(p, fd, VIDIOC_S_CTRL, &ctrl);
}


int _camerasrc_ioctl_g_ctrl(camerasrc_handle_t *p, int fd, guint cid, int *value)
{
	int ret = CAMERASRC_SUCCESS;
	struct v4l2_control ctrl;

	if (p == NULL || value == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	ctrl.id = cid;

	ret = _camerasrc_ioctl(p, fd, VIDIOC_G_CTRL, &ctrl);
	if (ret == CAMERASRC_SUCCESS) {
		*value = ctrl.value;
	} else {
		camsrc_error("VIDIOC_G_CTRL failed. %x", ret);
	}

	return ret;
}


int _camerasrc_ioctl_s_parm(camerasrc_handle_t *p, int fd, int type, int numerator, int denominator)
{
	struct v4l2_streamparm vstreamparm;

	if (p == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	CLEAR(vstreamparm);

	camsrc_info("ENTER : type %d", type);

	vstreamparm.type = type;
	vstreamparm.parm.capture.timeperframe.numerator = numerator;
	vstreamparm.parm.capture.timeperframe.denominator = denominator;

	camsrc_info("[FPS] timeperframe.numerator = %d", numerator);
	camsrc_info("[FPS] timeperframe.denominator = %d", denominator);

	return _camerasrc_ioctl(p, fd, VIDIOC_S_PARM, &vstreamparm);
}


int _camerasrc_ioctl_s_fmt(camerasrc_handle_t *p, int fd, int type, camerasrc_format_t *format)
{
	int i = 0;
	int width = 0;
	int height = 0;
	struct v4l2_format vformat;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	if (p == NULL || fd < 0 || format ==NULL) {
		camsrc_error("INVALID parameter %p %d %p", p, fd, format);
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	memset(&vformat, 0x0, sizeof(struct v4l2_format));

	width = format->img_size.width;
	height = format->img_size.height;

	camsrc_info("ENTER - type %d, format %d, %dx%d",
	            type, format->pix_format, width, height);

	vformat.type = type;
	vformat.fmt.pix_mp.width = width;
	vformat.fmt.pix_mp.height = height;
	vformat.fmt.pix_mp.field = V4L2_FIELD_ANY;
	vformat.fmt.pix_mp.colorspace = V4L2_COLORSPACE_JPEG;

	switch (format->pix_format) {
	case CAMERASRC_PIX_NV12:
		vformat.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
		vformat.fmt.pix_mp.num_planes = 2;
		//vformat.fmt.pix_mp.plane_fmt[0].bytesperline = (width * 3) >> 1;
		//vformat.fmt.pix_mp.plane_fmt[0].sizeimage = (width * height * 3) >> 1;
		break;
	case CAMERASRC_PIX_SN12:
		vformat.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12M;
		vformat.fmt.pix_mp.num_planes = 3;
		//vformat.fmt.pix_mp.plane_fmt[0].bytesperline = width;
		//vformat.fmt.pix_mp.plane_fmt[0].sizeimage = width * height;
		//vformat.fmt.pix_mp.plane_fmt[1].bytesperline = width >> 1;
		//vformat.fmt.pix_mp.plane_fmt[1].sizeimage = (width * height) >> 1;
		break;
	case CAMERASRC_PIX_NV21:
		vformat.fmt.pix.pixelformat = V4L2_PIX_FMT_NV21;
		vformat.fmt.pix_mp.num_planes = 2;
		//vformat.fmt.pix.bytesperline = (width * 3) >> 1;
		//vformat.fmt.pix.sizeimage = (width * height * 3) >> 1;
		break;
	case CAMERASRC_PIX_SN21:
		vformat.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV21M;
		vformat.fmt.pix_mp.num_planes = 3;
		//vformat.fmt.pix_mp.plane_fmt[0].bytesperline = width;
		//vformat.fmt.pix_mp.plane_fmt[0].sizeimage = width * height;
		//vformat.fmt.pix_mp.plane_fmt[1].bytesperline = width >> 1;
		//vformat.fmt.pix_mp.plane_fmt[1].sizeimage = (width * height) >> 1;
		break;
	case CAMERASRC_PIX_SBGGR12:
		vformat.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR12;
		vformat.fmt.pix_mp.num_planes = 2;
		//vformat.fmt.pix.bytesperline = ((width + 9) / 10) * 10 * 8 / 5;
		//vformat.fmt.pix.sizeimage = vformat.fmt.pix.bytesperline * height;
		break;
	default:
		camsrc_warning("invalid output format = %d, set NV12", format->pix_format);
		vformat.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
		vformat.fmt.pix_mp.num_planes = 2;
		//vformat.fmt.pix_mp.plane_fmt[0].bytesperline = (width * 3) >> 1;
		//vformat.fmt.pix_mp.plane_fmt[0].sizeimage = (width * height * 3) >> 1;
		break;
	}

	format->num_planes = vformat.fmt.pix_mp.num_planes;

	camsrc_info("== %d ======================================", i);
	camsrc_info("| Request output type = %d", vformat.type);
	camsrc_info("| Request output width = %d", vformat.fmt.pix_mp.width);
	camsrc_info("| Request output height = %d", vformat.fmt.pix_mp.height);
	camsrc_info("| Request output field = %d", vformat.fmt.pix_mp.field);
	camsrc_info("| Request output pixel format = %c%c%c%c",
	            vformat.fmt.pix_mp.pixelformat, vformat.fmt.pix_mp.pixelformat >> 8,
	            vformat.fmt.pix_mp.pixelformat >> 16, vformat.fmt.pix_mp.pixelformat >> 24);
	camsrc_info("| Request output pixel format = %d", vformat.fmt.pix_mp.pixelformat);
	camsrc_info("| Request output num planes = %d", vformat.fmt.pix_mp.num_planes);
	camsrc_info("===========================================");

	GST_INFO("            VIDIOC_S_FMT");
	if (CAMERASRC_SUCCESS != _camerasrc_ioctl(p, fd, VIDIOC_S_FMT, &vformat)) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("[%d] VIDIOC_S_FMT failed : %s", i, err_msg);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_info("DONE");

	return CAMERASRC_SUCCESS;
}


int _camerasrc_ioctl_reqbufs(camerasrc_handle_t *p, int fd, guint count, int type, int memory, guint *ret_count)
{
	int ret = CAMERASRC_SUCCESS;
	struct v4l2_requestbuffers vreq_bufs;

	if (p == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	CLEAR(vreq_bufs);

	vreq_bufs.count = count;
	vreq_bufs.type = type;
	vreq_bufs.memory = memory;

	camsrc_info("VIDIOC_REQBUFS : count %d, type %d, memory %d", count, type, memory);

	ret = _camerasrc_ioctl(p, fd, VIDIOC_REQBUFS, &vreq_bufs);

	if (ret_count) {
		*ret_count = vreq_bufs.count;
	}

	camsrc_info("VIDIOC_REQBUFS result : count %d", vreq_bufs.count);

	return ret;
}


int _camerasrc_ioctl_qbuf(camerasrc_handle_t *p, int fd, int type, int memory, int idx, camerasrc_buffer_t *buffer)
{
	int ret = CAMERASRC_SUCCESS;
	int i = 0;
	struct v4l2_plane planes[MAX_PLANE_NUM];
	struct v4l2_buffer lvbuf;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	if (p == NULL || buffer == NULL || fd < 0) {
		camsrc_error("INVALID parameter %p %p %d", p, buffer, fd);
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	camsrc_debug("ENTER - fd %d, type %d, memory %d, idx %d, buffer %p, planes %d",
	            fd, type, memory, idx, buffer, buffer->num_planes);

	CLEAR(lvbuf);
	CLEAR(planes);

	lvbuf.type = type;
	lvbuf.memory = memory;
	lvbuf.index = idx;
	lvbuf.m.planes = planes;
	lvbuf.length = buffer->num_planes;

	for (i = 0 ; i < MAX_PLANE_NUM ; i++) {
		if (buffer->planes[i].bo) {
			lvbuf.m.planes[i].m.fd = buffer->planes[i].fd;
			lvbuf.m.planes[i].length = buffer->planes[i].length;

			camsrc_debug("[index %d] [plane %d] fd %d, length %d",
			            idx, i, lvbuf.m.planes[i].m.fd, lvbuf.m.planes[i].length);
		} else {
			break;
		}
	}

	ret = _camerasrc_ioctl(p, fd, VIDIOC_QBUF, &lvbuf);
	if (ret != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("VIDIOC_QBUF failed : %s, fd %d, type %d, memory %d, idx %d, buffer %p",
		             err_msg, fd, type, memory, idx, buffer);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_debug("DONE");

	return CAMERASRC_SUCCESS;
}


int _camerasrc_ioctl_dqbuf(camerasrc_handle_t *p, int fd, int type, int memory, int *idx, int num_planes)
{
	int ret = CAMERASRC_SUCCESS;
	struct v4l2_plane planes[MAX_PLANE_NUM];
	struct v4l2_buffer lvbuf;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	if (p == NULL || idx == NULL || fd < 0) {
		camsrc_error("INVALID parameter %p %p %d", p, idx, fd);
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	camsrc_debug("ENTER - fd %d, type %d, memory %d, num planes %d", fd, type, memory, num_planes);

	CLEAR(lvbuf);

	lvbuf.type = type;
	lvbuf.memory = memory;
	lvbuf.m.planes = planes;
	lvbuf.length = num_planes;

	ret = _camerasrc_ioctl(p, fd, VIDIOC_DQBUF, &lvbuf);
	if (ret != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("VIDIOC_DQBUF failed : %s, fd %d, type %d, memory %d",
		             err_msg, fd, type, memory);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	*idx = lvbuf.index;

	camsrc_debug("DONE - index %d", *idx);

	return CAMERASRC_SUCCESS;
}


int _camerasrc_ioctl_stream(camerasrc_handle_t *p, int fd, int type, int on)
{
	int ret = CAMERASRC_SUCCESS;
	enum v4l2_buf_type vtype;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	if (p == NULL || fd < 0) {
		camsrc_error("INVALID parameter %p %d", p, fd);
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	camsrc_info("STREAM %d, type %d", on, type);

	vtype = type;

	ret = _camerasrc_ioctl(p, fd, on ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &vtype);
	if (ret != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		if (on) {
			camsrc_error("VIDIOC_STREAMON failed[%x] : %s", p->errnum, err_msg);
		} else {
			camsrc_error("VIDIOC_STREAMOFF failed[%x] : %s", p->errnum, err_msg);
		}
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_info("DONE");

	return CAMERASRC_SUCCESS;
}


/****  A U T O F O C U S I N G   F U N C T I O N S  ****/
int _camerasrc_set_autofocusing_area(camerasrc_handle_t *handle, camerasrc_rect_t *rect)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_AF_AREA, rect);
}

int _camerasrc_get_autofocusing_area(camerasrc_handle_t *handle, camerasrc_rect_t *rect)
{
	return CAMERASRC_GET_CMD(_CAMERASRC_CMD_AF_AREA, rect);
}

int _camerasrc_start_autofocusing(camerasrc_handle_t *handle)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_AF_CONTROL, _CAMERASRC_AF_START);
}

int _camerasrc_stop_autofocusing(camerasrc_handle_t *handle)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_AF_CONTROL, _CAMERASRC_AF_STOP);
}

int _camerasrc_release_autofocusing(camerasrc_handle_t *handle)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_AF_CONTROL, _CAMERASRC_AF_RELEASE);
}

int _camerasrc_destroy_autofocusing(camerasrc_handle_t *handle)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_AF_CONTROL, _CAMERASRC_AF_DESTROY);
}

int _camerasrc_init_autofocusing_mode(camerasrc_handle_t *handle)
{
	return CAMERASRC_SET_CMD(_CAMERASRC_CMD_AF_CONTROL, _CAMERASRC_AF_INIT);
}

int _camerasrc_get_autofocusing_result(camerasrc_handle_t *handle)
{
	return CAMERASRC_GET_CMD(_CAMERASRC_CMD_AF_CONTROL, _CAMERASRC_AF_RESULT);
}


int _camerasrc_get_frame_data(camerasrc_handle_t *handle, camerasrc_frame_data_t *data)
{
	return CAMERASRC_GET_CMD(_CAMERASRC_CMD_FRAME_DATA, data);
}

int _camerasrc_get_exif_info(camerasrc_handle_t *handle, camerasrc_buffer_t *exif_string)
{
	return CAMERASRC_GET_CMD(_CAMERASRC_CMD_EXIF_INFO, exif_string);
}


static int __camerasrc_open_device(camerasrc_handle_t *p, camerasrc_dev_id_t camera_id)
{
	int ret = CAMERASRC_ERR_DEVICE_NOT_FOUND;
	int ret_ioctl = 0;
	int input = 0x0;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	camsrc_warning("start - camera id %d", camera_id);

	if (p == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	p->cur_dev_id = camera_id;

	if (camera_id == CAMERASRC_DEV_ID_PRIMARY) {
		/* TODO : secondary camera is only enabled now */
	} else {
		/* open device nodes and VIDIOC_S_INPUT */
		/* SENSOR */
		p->fd_companion = open(VIDEO_DEVICE_NODE_COMPANION, O_RDWR, 0);
		if (p->fd_companion < 0) {
			camsrc_error("open failed [%s] : COMPANION", VIDEO_DEVICE_NODE_COMPANION);
			goto OPEN_FAILED;
		}

		p->fd_sensor = open(VIDEO_DEVICE_NODE_SECONDARY, O_RDWR, 0);
		if (p->fd_sensor < 0) {
			camsrc_error("open failed [%s] : SENSOR", VIDEO_DEVICE_NODE_SECONDARY);
			goto OPEN_FAILED;
		}

		input = 0x10;
		camsrc_info("[SENSOR] VIDIOC_S_INPUT %d", input);
		ret_ioctl = ioctl(p->fd_sensor, VIDIOC_S_INPUT, &input);
		if (ret_ioctl < 0) {
			camsrc_error("[SENSOR] VIDIOC_S_INPUT failed. SENSOR - input 0x%x", input);
			goto OPEN_FAILED;
		}

		/* ISP */
		p->fd_isp = open(VIDEO_DEVICE_NODE_ISP, O_RDWR, 0);
		if (p->fd_isp < 0) {
			camsrc_error("open failed [%s] : ISP", VIDEO_DEVICE_NODE_ISP);
			goto OPEN_FAILED;
		}

		/* 3AA1 */
		p->fd_3aa1 = open(VIDEO_DEVICE_NODE_3AA1, O_RDWR, 0);
		if (p->fd_3aa1 < 0) {
			camsrc_error("open failed [%s] : 3AA1", VIDEO_DEVICE_NODE_3AA1);
			goto OPEN_FAILED;
		}

		input = 0x11010;
		camsrc_info("[3AA1 - 1] VIDIOC_S_INPUT %d", input);
		ret_ioctl = ioctl(p->fd_3aa1, VIDIOC_S_INPUT, &input);
		if (ret_ioctl < 0) {
			camsrc_error("[3AA1] VIDIOC_S_INPUT failed. 3AA1 - input 0x%x", input);
			goto OPEN_FAILED;
		}

		camsrc_info("[3AA1 - 2] VIDIOC_S_INPUT %d", input);
		ret_ioctl = ioctl(p->fd_3aa1, VIDIOC_S_INPUT, &input);
		if (ret_ioctl < 0) {
			camsrc_error("[3AA1] VIDIOC_S_INPUT failed. 3AA1 - input 0x%x", input);
			goto OPEN_FAILED;
		}

		/* 3AA1C */
		p->fd_3aa1c = open(VIDEO_DEVICE_NODE_3AA1C, O_RDWR, 0);
		if (p->fd_3aa1c < 0) {
			camsrc_error("open failed [%s] : 3AA1C", VIDEO_DEVICE_NODE_3AA1C);
			goto OPEN_FAILED;
		}

		input = 0x10010;
		camsrc_info("[3AA1C] VIDIOC_S_INPUT %d", input);
		ret_ioctl = ioctl(p->fd_3aa1c, VIDIOC_S_INPUT, &input);
		if (ret_ioctl < 0) {
			camsrc_error("[3AA1C] VIDIOC_S_INPUT failed. 3AA1C - input 0x%x", input);
			goto OPEN_FAILED;
		}

		/* SCC */
		p->fd_scc = open(VIDEO_DEVICE_NODE_SCC, O_RDWR, 0);
		if (p->fd_scc < 0) {
			camsrc_error("open failed [%s] : SCC", VIDEO_DEVICE_NODE_SCC);
			goto OPEN_FAILED;
		}

		input = 0x2210;
		camsrc_info("[SCC] VIDIOC_S_INPUT %d", input);
		ret_ioctl = ioctl(p->fd_scc, VIDIOC_S_INPUT, &input);
		if (ret_ioctl < 0) {
			camsrc_error("[SCC] VIDIOC_S_INPUT failed. SCC - input 0x%x", input);
			goto OPEN_FAILED;
		}

		/* SCP */
		p->fd_scp = open(VIDEO_DEVICE_NODE_SCP, O_RDWR, 0);
		if (p->fd_scp < 0) {
			camsrc_error("open failed [%s] : SCP", VIDEO_DEVICE_NODE_SCP);
			goto OPEN_FAILED;
		}

		input = 0x2510;
		camsrc_info("[SCP] VIDIOC_S_INPUT %d", input);
		ret_ioctl = ioctl(p->fd_scp, VIDIOC_S_INPUT, &input);
		if (ret_ioctl < 0) {
			camsrc_error("[SCP] VIDIOC_S_INPUT failed. SCP - input 0x%x", input);
			goto OPEN_FAILED;
		}
	}

	camsrc_warning("done");

	return CAMERASRC_SUCCESS;

OPEN_FAILED:
	/* open failed */
	strerror_r(errno, err_msg, CAMERASRC_ERRMSG_MAX_LEN);

	switch (errno) {
	case EBUSY:
		ret = CAMERASRC_ERR_DEVICE_BUSY;
		break;
	case ENOENT:
	case ENODEV:
		ret = CAMERASRC_ERR_DEVICE_NOT_FOUND;
		break;
	default:
		ret = CAMERASRC_ERR_DEVICE_OPEN;
		break;
	}

	camsrc_error("Device open fail [%d][%s][ret:0x%x]", errno, err_msg, ret);

	__camerasrc_close_device(p);

	return ret;
}


static void __camerasrc_close_device(camerasrc_handle_t *p)
{
	if (!p) {
		camsrc_error("handle is NULL");
		return;
	}

	camsrc_info("start");

	/* Disable link media */
	if (p->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY) {
		/* TODO : secondary camera is only enabled now */
	} else {
		if (p->fd_companion > -1) {
			camsrc_info("close fd_companion : %d", p->fd_companion);
			close(p->fd_companion);
			p->fd_companion = -1;
		}

		if (p->fd_sensor > -1) {
			camsrc_info("close fd_sensor : %d", p->fd_sensor);
			close(p->fd_sensor);
			p->fd_sensor = -1;
		}
		if (p->fd_isp > -1) {
			camsrc_info("close fd_isp : %d", p->fd_isp);
			close(p->fd_isp);
			p->fd_isp = -1;
		}
		if (p->fd_3aa1 > -1) {
			camsrc_info("close fd_3aa1 : %d", p->fd_3aa1);
			close(p->fd_3aa1);
			p->fd_3aa1 = -1;
		}
		if (p->fd_3aa1c > -1) {
			camsrc_info("close fd_3aa1c : %d", p->fd_3aa1c);
			close(p->fd_3aa1c);
			p->fd_3aa1c = -1;
		}
		if (p->fd_scc > -1) {
			camsrc_info("close fd_scc : %d", p->fd_scc);
			close(p->fd_scc);
			p->fd_scc = -1;
		}
		if (p->fd_scp > -1) {
			camsrc_info("close fd_scp : %d", p->fd_scp);
			close(p->fd_scp);
			p->fd_scp = -1;
		}
	}

	camsrc_info("done");

	return;
}


static int _camerasrc_initialize_handle(camerasrc_handle_t *handle)
{
/*
	int i = 0;
*/
	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	/* Initialize handle variables */
	handle->buffer_sensor = NULL;
	handle->buffer_isp = NULL;
	handle->buffer_scc = NULL;
	handle->buffer_scp = NULL;
	handle->fd_sensor = -1;
	handle->fd_isp = -1;
	handle->fd_3aa1 = -1;
	handle->fd_3aa1c = -1;
	handle->fd_scc = -1;
	handle->fd_scp = -1;
	handle->cur_dev_id = CAMERASRC_DEV_ID_UNKNOWN;
	handle->buffer_idx = 0;
	handle->af_status = CAMERASRC_AUTO_FOCUS_STATUS_RELEASED;
	handle->af_cmd = CAMERASRC_AUTO_FOCUS_CMD_NULL;
	handle->cur_af_mode = CAMERASRC_AF_MODE_AUTO;
	handle->cur_af_range = CAMERASRC_AF_RANGE_NORMAL;
	handle->af_usr_data = NULL;
	handle->af_cb = NULL;
	handle->first_frame = 1;
	handle->request_count = 0;

	CLEAR(handle->queued_buf_list);
	CLEAR(handle->format_sensor);
	CLEAR(handle->format_3aa1);
	CLEAR(handle->format_isp);
	CLEAR(handle->format_scc);
	CLEAR(handle->format_scp);

	return CAMERASRC_SUCCESS;
}


static int _camerasrc_wait_frame_available(camerasrc_handle_t *handle, int timeout)
{
	camerasrc_handle_t *p = handle;
	struct pollfd poll_fd;
	int r = 0;

	if (p == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	memset(&poll_fd, 0x0, sizeof(struct pollfd));

	poll_fd.fd = p->fd_scp;

	/* select waiting */
	r = poll(&poll_fd, 1, timeout);
	if (-1 == r) {
		if (EINTR == errno) {
			return CAMERASRC_SUCCESS;
		}
		camsrc_error("poll() failed.");

		return CAMERASRC_ERR_INTERNAL;
	}

	if (r == 0) {
		camsrc_error("poll() timeout");
		return CAMERASRC_ERR_DEVICE_WAIT_TIMEOUT;
	}

	return CAMERASRC_SUCCESS;
}


static int _camerasrc_queue_buffer(camerasrc_handle_t *handle, int buf_index)
{
	camerasrc_handle_t *p = handle;
	int ret;

	if (p == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	ret = _camerasrc_ioctl_qbuf(p,
	                            p->fd_scp,
	                            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                            V4L2_MEMORY_DMABUF,
	                            buf_index,
	                            &p->buffer_scp[buf_index]);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("_camerasrc_ioctl_qbuf failed");
		return CAMERASRC_ERR_IO_CONTROL;
	}

	return CAMERASRC_SUCCESS;
}


static int _camerasrc_dequeue_buffer(camerasrc_handle_t *handle, int *buf_index, camerasrc_buffer_t *buffer, camerasrc_buffer_t *thm_buffer)
{
	camerasrc_handle_t *p = handle;
	int ret = CAMERASRC_ERR_UNKNOWN;
	int buf_index_sensor = 0;
	int buf_index_3aa1cap = 0;
	int buf_index_3aa1out = 0;
	int buf_index_isp = 0;
	int buf_index_scc = 0;
	int buf_index_scp = 0;
	unsigned long long timestamp;
	struct camera2_shot_ext *shot_sensor = NULL;
	struct camera2_shot_ext *shot_isp = NULL;
	struct camera2_shot_ext *shot_scc = NULL;

	if (p->buffer_scp == NULL) {
		camsrc_error("buffer is not allocated");
		return CAMERASRC_ERR_ALLOCATION;
	}

	/* SENSOR - DQBUF */
	ret = _camerasrc_ioctl_dqbuf(p,
	                             p->fd_sensor,
	                             V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                             V4L2_MEMORY_DMABUF,
	                             &buf_index_sensor,
	                             p->buffer_sensor[0].num_planes);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[SENSOR] _camerasrc_ioctl_dqbuf failed");
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_debug("[SENSOR] DQBUF index %d", buf_index_sensor);

	shot_sensor = (struct camera2_shot_ext *)p->buffer_sensor[buf_index_sensor].planes[1].start;
	timestamp = shot_sensor->shot.dm.sensor.timeStamp;
	handle->request_count++;

	/* SENSOR set shot meta */
	_camerasrc_set_shot_meta(p, GRP_3AA, handle->request_count, timestamp, shot_sensor);

	/* 3AA1 CAPTURE - QBUF */
	ret = _camerasrc_ioctl_qbuf(p,
	                            p->fd_3aa1,
	                            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                            V4L2_MEMORY_DMABUF,
	                            buf_index_sensor,
	                            &p->buffer_isp[buf_index_sensor]);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[3AA1 - CAPTURE] _camerasrc_ioctl_qbuf index %d, failed[%x]", buf_index_sensor, ret);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	/* 3AA1 OUTPUT - QBUF */
	ret = _camerasrc_ioctl_qbuf(p,
	                            p->fd_3aa1,
	                            V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	                            V4L2_MEMORY_DMABUF,
	                            buf_index_sensor,
	                            &p->buffer_sensor[buf_index_sensor]);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[3AA1 - OUTPUT] _camerasrc_ioctl_qbuf index %d, failed[%x]", buf_index_sensor, ret);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	/* 3AA1 CAPTURE - DQBUF */
	ret = _camerasrc_ioctl_dqbuf(p,
	                             p->fd_3aa1,
	                             V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                             V4L2_MEMORY_DMABUF,
	                             &buf_index_3aa1cap,
	                             p->buffer_sensor[0].num_planes);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[3AA1 CAPTURE] _camerasrc_ioctl_dqbuf failed");
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_debug("[3AA1 - CAPTURE] DQBUF index %d", buf_index_3aa1cap);

	/* 3AA1 OUTPUT - DQBUF */
	ret = _camerasrc_ioctl_dqbuf(p,
	                             p->fd_3aa1,
	                             V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	                             V4L2_MEMORY_DMABUF,
	                             &buf_index_3aa1out,
	                             p->buffer_sensor[0].num_planes);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[3AA1 OUTPUT] _camerasrc_ioctl_dqbuf failed");
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_debug("[3AA1 - OUTPUT] DQBUF index %d", buf_index_3aa1out);

	/* SENSOR set shot meta */
	_camerasrc_set_shot_meta(p, GRP_SENSOR, 0, 0, shot_sensor);

	/* SENSOR - QBUF */
	ret = _camerasrc_ioctl_qbuf(p,
	                            p->fd_sensor,
	                            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                            V4L2_MEMORY_DMABUF,
	                            buf_index_sensor,
	                            &p->buffer_sensor[buf_index_sensor]);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[SENSOR] _camerasrc_ioctl_qbuf index %d, failed[%x]", buf_index_sensor, ret);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	shot_isp = (struct camera2_shot_ext *)p->buffer_isp[buf_index_sensor].planes[1].start;
	_camerasrc_set_shot_meta(p, GRP_ISP, handle->request_count, timestamp, shot_isp);

	/* ISP - QBUF */
	ret = _camerasrc_ioctl_qbuf(p,
	                            p->fd_isp,
	                            V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	                            V4L2_MEMORY_DMABUF,
	                            buf_index_sensor,
	                            &p->buffer_isp[buf_index_sensor]);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[ISP] _camerasrc_ioctl_qbuf index %d, failed[%x]", buf_index_sensor, ret);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	/* ISP - DQBUF */
	ret = _camerasrc_ioctl_dqbuf(p,
	                             p->fd_isp,
	                             V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	                             V4L2_MEMORY_DMABUF,
	                             &buf_index_isp,
	                             p->buffer_isp[0].num_planes);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[3AA1 OUTPUT] _camerasrc_ioctl_dqbuf failed");
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_debug("[ISP] DQBUF index %d", buf_index_isp);

	//_camerasrc_wait_frame_available(p, 2000);

	/* SCC - DQBUF */
	ret = _camerasrc_ioctl_dqbuf(p,
	                             p->fd_scc,
	                             V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                             V4L2_MEMORY_DMABUF,
	                             &buf_index_scc,
	                             p->buffer_scc[0].num_planes);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[SCC] _camerasrc_ioctl_dqbuf failed");
		return CAMERASRC_ERR_IO_CONTROL;
	}

	shot_scc = (struct camera2_shot_ext *)p->buffer_scc[buf_index_scc].planes[1].start;
	shot_scc->shot.dm.request.frameCount = handle->request_count;

	/* SCC - QBUF */
	ret = _camerasrc_ioctl_qbuf(p,
	                            p->fd_scc,
	                            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                            V4L2_MEMORY_DMABUF,
	                            buf_index_scc,
	                            &p->buffer_scc[buf_index_scc]);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[SCC] _camerasrc_ioctl_qbuf index %d, failed[%x]", buf_index_scc, ret);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	/* SCP */
	ret = _camerasrc_ioctl_dqbuf(p,
	                             p->fd_scp,
	                             V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	                             V4L2_MEMORY_DMABUF,
	                             &buf_index_scp,
	                             p->buffer_scp[0].num_planes);
	if (ret != CAMERASRC_SUCCESS) {
		camsrc_error("[SCC] _camerasrc_ioctl_dqbuf failed");
		return CAMERASRC_ERR_IO_CONTROL;
	}

	camsrc_debug("SCP DQBUF index : %d", buf_index_scp);

	memcpy(buffer, &p->buffer_scp[buf_index_scp], sizeof(camerasrc_buffer_t));

	*buf_index = buf_index_scp;

	return CAMERASRC_SUCCESS;
}

static int _camerasrc_dump_format(camsrc_handle_t handle);

/**** M A I N    O P E R A T I O N ****/
int camerasrc_create(camsrc_handle_t *phandle)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	camsrc_info("enter");

	if (phandle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = (camerasrc_handle_t *)malloc(sizeof(camerasrc_handle_t));
	if(p == NULL) {
		camsrc_error("malloc fail");
		return CAMERASRC_ERR_ALLOCATION;
	}

	memset(p, '\0', sizeof(camerasrc_handle_t));

	/* STATE TO MEANINGFUL STATE */
	p->cur_state = CAMERASRC_STATE_NONE;
	CAMERASRC_SET_STATE(p, CAMERASRC_STATE_CREATED);
	CAMERASRC_SET_PHASE(p, CAMERASRC_PHASE_NON_RUNNING);
	camsrc_info("Transit to non-running phase");

	/* INIT VARIABLES */
	err = _camerasrc_initialize_handle(p);
	if(err != CAMERASRC_SUCCESS) {
		camsrc_error("Invalid handle");
		return CAMERASRC_ERR_INVALID_HANDLE;
	}

	err = pthread_mutex_init(&(p->mutex), NULL);
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("Mutex creating status : %s", err_msg);
	}

	err = pthread_mutex_init(&(p->af_mutex), NULL);
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF Mutex creating status : %s", err_msg);
	}

	err = pthread_cond_init(&(p->af_wait_cond), NULL);
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF Condition creating status : %s", err_msg);
	}

	camsrc_info("Thread creation start..");
	err = pthread_create(&p->focusing_thread, NULL, (void*)_camerasrc_run_autofocusing, p);
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF Thread creating status : %s", err_msg);
	}

	camsrc_info("Thread creation end..");

	/* set default focus mode */
	p->cur_af_mode = CAMERASRC_AF_MODE_CONTINUOUS;
	p->cur_af_range = CAMERASRC_AF_RANGE_NORMAL;

	*phandle = (camsrc_handle_t)p;
	err = CAMERASRC_SUCCESS;
	return err;
}


int camerasrc_destroy(camsrc_handle_t handle)
{
	camerasrc_handle_t* p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if(CAMERASRC_STATE(p) != CAMERASRC_STATE_UNREALIZED) {
		camsrc_warning("Invalid state transition");
	}

	/* Remove AF thread */
	_camerasrc_destroy_autofocusing(p);

	camsrc_info("Thread join wait start..");
	err = pthread_join(p->focusing_thread, NULL);
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF Thread join status : %s", err_msg);
	}

	camsrc_info("Thread join wait end..");

	err = pthread_cond_destroy(&p->af_wait_cond);
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF Condition destroying error : %s", err_msg);
	}

	err = pthread_mutex_destroy(&(p->af_mutex));
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF Mutex destroying error : %s", err_msg);
	}

	err = pthread_mutex_destroy(&(p->mutex));
	if(err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("Mutex destroying error : %s", err_msg);
	}

	free((void*)p);
	handle = NULL;
	err = CAMERASRC_SUCCESS;
	return err;
}


int camerasrc_get_state(camsrc_handle_t handle, camerasrc_state_t* state)
{
	camerasrc_handle_t* p = NULL;

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	*state = CAMERASRC_STATE(p);
	return CAMERASRC_SUCCESS;
}


int camerasrc_realize(camsrc_handle_t handle, camerasrc_dev_id_t camera_id, camerasrc_sensor_mode_t mode)
{
	camerasrc_handle_t* p = NULL;
	int ret = CAMERASRC_SUCCESS;

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	/* STATE OR PHASE CHECKING */
	if(CAMERASRC_STATE(p) == CAMERASRC_STATE_REALIZED) {
		return CAMERASRC_SUCCESS;
	}
	if(CAMERASRC_STATE(p) != CAMERASRC_STATE_CREATED &&
	   CAMERASRC_STATE(p) != CAMERASRC_STATE_UNREALIZED) {
		camsrc_warning("Invalid state transition");
	}

	/* Open device */
	ret = __camerasrc_open_device(p, camera_id);

	if (ret == CAMERASRC_SUCCESS) {
		CAMERASRC_SET_STATE(p, CAMERASRC_STATE_REALIZED);
	}

	return ret;
}


int camerasrc_unrealize(camsrc_handle_t handle)
{
	camerasrc_handle_t* p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if(CAMERASRC_STATE(p) == CAMERASRC_STATE_UNREALIZED ||
			CAMERASRC_STATE(p) == CAMERASRC_STATE_DESTROYED) {
		return CAMERASRC_SUCCESS;
	}
	if(CAMERASRC_STATE(p) != CAMERASRC_STATE_READY) {
		camsrc_warning("Invalid state transition");
	}

	__camerasrc_close_device(p);

	CAMERASRC_SET_STATE(p, CAMERASRC_STATE_UNREALIZED);
	CAMERASRC_SET_PHASE(p, CAMERASRC_PHASE_NON_RUNNING);
	camsrc_info("Transit to non-running phase");
	err = CAMERASRC_SUCCESS;
	return err;
}


int camerasrc_start(camsrc_handle_t handle)
{
	camerasrc_handle_t* p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	/* STATE OR PHASE CHECKING */
	if(CAMERASRC_STATE(p) == CAMERASRC_STATE_READY) {
		return CAMERASRC_SUCCESS;
	}
	if(CAMERASRC_STATE(p) != CAMERASRC_STATE_REALIZED) {
		camsrc_warning("Invalid state transition");
	}

	CAMERASRC_SET_STATE(p, CAMERASRC_STATE_READY);
	CAMERASRC_SET_PHASE(p, CAMERASRC_PHASE_RUNNING);

	camsrc_info("Transit to running phase");
	err = CAMERASRC_SUCCESS;
	return err;
}


int camerasrc_get_num_buffer(camsrc_handle_t handle, unsigned int *num_buffer)
{
	camerasrc_handle_t *p = NULL;

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	camerasrc_state_t state = CAMERASRC_STATE_NONE;
	camerasrc_get_state(handle, &state);

	switch(state) {
	case CAMERASRC_STATE_NONE:
	case CAMERASRC_STATE_CREATED:
	case CAMERASRC_STATE_REALIZED:
	case CAMERASRC_STATE_READY:
	case CAMERASRC_STATE_UNREALIZED:
	case CAMERASRC_STATE_DESTROYED:
		camsrc_warning("Current buffer number not initialized.");
		*num_buffer = 0;
		break;
	case CAMERASRC_STATE_PREVIEW:
	case CAMERASRC_STATE_STILL:
	case CAMERASRC_STATE_VIDEO:
	case CAMERASRC_STATE_AF_IN_PROGRESS:
		*num_buffer = p->format_scp.num_buffers;
		break;
	default:
		camsrc_error("Unknown state");
		*num_buffer = -1;
		return CAMERASRC_ERR_INVALID_STATE;
	}

	return CAMERASRC_SUCCESS;
}


int camerasrc_create_buffer(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;
	int ret = CAMERASRC_SUCCESS;
	int count = 0;
	int plane_index = 0;
	int plane_num = 0;
	int buffer_size[MAX_PLANE_NUM] = {0,};
	int preview_width = 0;
	int preview_height = 0;
	int sensor_width = 0;
	int sensor_height = 0;
	int aligned_width = 0;
	int bytes_per_line = 0;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (p->buffer_sensor || p->buffer_isp || p->buffer_scc || p->buffer_scp) {
		camerasrc_destroy_buffer(handle);
	}

	preview_width = p->format_scp.img_size.width;
	preview_height = p->format_scp.img_size.height;

	memset(&p->format_sensor, 0x0, sizeof(camerasrc_format_t));
	memset(&p->format_isp, 0x0, sizeof(camerasrc_format_t));
	memset(&p->format_scc, 0x0, sizeof(camerasrc_format_t));

	p->format_sensor.num_buffers = p->format_scp.num_buffers - 6;
	p->format_3aa1.num_buffers = p->format_scp.num_buffers - 6;
	p->format_isp.num_buffers = p->format_scp.num_buffers - 6;
	p->format_scc.num_buffers = p->format_scp.num_buffers - 6;

	/* set sensor resolution */
	sensor_width = FRONT_SENSOR_SIZE_WIDTH + SENSOR_MARGIN_WIDTH;
	sensor_height = FRONT_SENSOR_SIZE_HEIGHT + SENSOR_MARGIN_HEIGHT;

	/**
	 * buffer allocation
	 */
	/* sensor */
	p->buffer_sensor = (camerasrc_buffer_t *)malloc(sizeof(camerasrc_buffer_t) * p->format_sensor.num_buffers);
	if (p->buffer_sensor == NULL) {
		camsrc_error("failed to buffer_sensor");
		return CAMERASRC_ERR_ALLOCATION;
	}

	memset(p->buffer_sensor, 0x0, sizeof(camerasrc_buffer_t) * p->format_sensor.num_buffers);

	aligned_width = ((sensor_width + 9) / 10) * 10;
	bytes_per_line = (aligned_width * 8) / 5;

	buffer_size[0] = bytes_per_line * sensor_height;
	buffer_size[1] = SPARE_SIZE;
	buffer_size[2] = 0;
	buffer_size[3] = 0;

	for (count = 0 ; count < p->format_sensor.num_buffers ; count++) {
		p->buffer_sensor[count].num_planes = BAYER_PLANE_COUNT;
		for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
			if (buffer_size[plane_index] > 0) {
				ret = camerasrc_tbm_alloc_buffer(handle, buffer_size[plane_index],
				                                 &(p->buffer_sensor[count].planes[plane_index].bo),
				                                 &(p->buffer_sensor[count].planes[plane_index].fd),
				                                 &(p->buffer_sensor[count].planes[plane_index].start));
				if (ret == FALSE) {
					camsrc_error("[sensor] camerasrc_tbm_alloc_buffer failed[%d:%d] : size %d",
					             count, plane_index, buffer_size[plane_index]);
					goto CREATE_BUFFER_FAILED;
				}

				p->buffer_sensor[count].planes[plane_index].length = buffer_size[plane_index];

				camsrc_info("sensor [%d][%d] BUF: bo[%p] fd[%d] addr[%p] size[%d]",
				            count, plane_index,
				            p->buffer_sensor[count].planes[plane_index].bo,
				            p->buffer_sensor[count].planes[plane_index].fd,
				            p->buffer_sensor[count].planes[plane_index].start,
				            p->buffer_sensor[count].planes[plane_index].length);
			}
		}
	}

	/* ISP */
	p->buffer_isp = (camerasrc_buffer_t *)malloc(sizeof(camerasrc_buffer_t) * p->format_isp.num_buffers);
	if (p->buffer_isp == NULL) {
		camsrc_error("failed to buffer_isp");
		goto CREATE_BUFFER_FAILED;
	}

	memset(p->buffer_isp, 0x0, sizeof(camerasrc_buffer_t) * p->format_isp.num_buffers);

	for (count = 0 ; count < p->format_isp.num_buffers ; count++) {
		p->buffer_isp[count].num_planes = ISP_PLANE_COUNT;
		for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
			if (buffer_size[plane_index] > 0) {
				ret = camerasrc_tbm_alloc_buffer(handle, buffer_size[plane_index],
				                                 &(p->buffer_isp[count].planes[plane_index].bo),
				                                 &(p->buffer_isp[count].planes[plane_index].fd),
				                                 &(p->buffer_isp[count].planes[plane_index].start));
				if (ret == FALSE) {
					camsrc_error("[isp] camerasrc_tbm_alloc_buffer failed[%d:%d] : size %d",
					             count, plane_index, buffer_size[plane_index]);
					goto CREATE_BUFFER_FAILED;
				}

				p->buffer_isp[count].planes[plane_index].length = buffer_size[plane_index];

				camsrc_info("isp [%d][%d] BUF: bo[%p] fd[%d] addr[%p] size[%d]",
				            count, plane_index,
				            p->buffer_isp[count].planes[plane_index].bo,
				            p->buffer_isp[count].planes[plane_index].fd,
				            p->buffer_isp[count].planes[plane_index].start,
				            p->buffer_isp[count].planes[plane_index].length);
			}
		}
	}

	/* SCC */
	p->buffer_scc = (camerasrc_buffer_t *)malloc(sizeof(camerasrc_buffer_t) * p->format_scc.num_buffers);
	if (p->buffer_scc == NULL) {
		camsrc_error("failed to buffer_scc");
		goto CREATE_BUFFER_FAILED;
	}

	memset(p->buffer_scc, 0x0, sizeof(camerasrc_buffer_t) * p->format_scc.num_buffers);

	for (count = 0 ; count < p->format_scc.num_buffers ; count++) {
		p->buffer_scc[count].num_planes = SCC_PLANE_COUNT;
		for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
			if (buffer_size[plane_index] > 0) {
				ret = camerasrc_tbm_alloc_buffer(handle, buffer_size[plane_index],
				                                 &(p->buffer_scc[count].planes[plane_index].bo),
				                                 &(p->buffer_scc[count].planes[plane_index].fd),
				                                 &(p->buffer_scc[count].planes[plane_index].start));
				if (ret == FALSE) {
					camsrc_error("[scc] camerasrc_tbm_alloc_buffer failed[%d:%d] : size %d",
					             count, plane_index, buffer_size[plane_index]);
					goto CREATE_BUFFER_FAILED;
				}

				p->buffer_scc[count].planes[plane_index].length = buffer_size[plane_index];

				camsrc_info("scc [%d][%d] BUF: bo[%p] fd[%d] addr[%p] size[%d]",
				            count, plane_index,
				            p->buffer_scc[count].planes[plane_index].bo,
				            p->buffer_scc[count].planes[plane_index].fd,
				            p->buffer_scc[count].planes[plane_index].start,
				            p->buffer_scc[count].planes[plane_index].length);
			}
		}
	}

	/* SCP */
	p->buffer_scp = (camerasrc_buffer_t *)malloc(sizeof(camerasrc_buffer_t) * p->format_scp.num_buffers);
	if (p->buffer_scp == NULL) {
		camsrc_error("failed to buffer_scp");
		goto CREATE_BUFFER_FAILED;
	}

	memset(p->buffer_scp, 0x0, sizeof(camerasrc_buffer_t) * p->format_scp.num_buffers);

	switch (p->format_scp.pix_format) {
	case CAMERASRC_PIX_YUY2:
		buffer_size[0] = buf_4k_align((preview_width * preview_height) << 1);
		buffer_size[1] = buf_4k_align(SPARE_SIZE);
		buffer_size[2] = 0;
		buffer_size[3] = 0;
		plane_num = 2;
		break;
	case CAMERASRC_PIX_YUV420P:
		buffer_size[0] = buf_4k_align(preview_width * preview_height);
		buffer_size[1] = buf_4k_align((preview_width * preview_height) >> 2);
		buffer_size[2] = buffer_size[1];
		buffer_size[3] = buf_4k_align(SPARE_SIZE);
		plane_num = 4;
		break;
	case CAMERASRC_PIX_NV12:
		buffer_size[0] = buf_4k_align((preview_width * preview_height * 3) >> 1);
		buffer_size[1] = buf_4k_align(SPARE_SIZE);
		buffer_size[2] = 0;
		buffer_size[3] = 0;
		plane_num = 2;
		break;
	case CAMERASRC_PIX_SN12:
		buffer_size[0] = buf_4k_align(preview_width * preview_height);
		buffer_size[1] = buf_4k_align((preview_width * preview_height) >> 1);
		buffer_size[2] = buf_4k_align(SPARE_SIZE);
		buffer_size[3] = 0;
		plane_num = 3;
		break;
	default:
		camsrc_warning("invalid output format = %d, set size as NV12M(SN12)", p->format_scp.pix_format);
		buffer_size[0] = buf_4k_align(preview_width * preview_height);
		buffer_size[1] = buf_4k_align((preview_width * preview_height) >> 1);
		buffer_size[2] = buf_4k_align(SPARE_SIZE);
		buffer_size[3] = 0;
		plane_num = 3;
		break;
	}

	for (count = 0 ; count < p->format_scp.num_buffers ; count++) {
		p->buffer_scp[count].num_planes = plane_num;
		for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
			if (buffer_size[plane_index] > 0) {
				ret = camerasrc_tbm_alloc_buffer(handle, buffer_size[plane_index],
				                                 &(p->buffer_scp[count].planes[plane_index].bo),
				                                 &(p->buffer_scp[count].planes[plane_index].fd),
				                                 &(p->buffer_scp[count].planes[plane_index].start));
				if (ret == FALSE) {
					camsrc_error("[scp] camerasrc_tbm_alloc_buffer failed[%d:%d] : size %d",
					             count, plane_index, buffer_size[plane_index]);
					goto CREATE_BUFFER_FAILED;
				}

				p->buffer_scp[count].planes[plane_index].length = buffer_size[plane_index];

				camsrc_info("scp [%d][%d] BUF: bo[%p] fd[%d] addr[%p] size[%d]",
				            count, plane_index,
				            p->buffer_scp[count].planes[plane_index].bo,
				            p->buffer_scp[count].planes[plane_index].fd,
				            p->buffer_scp[count].planes[plane_index].start,
				            p->buffer_scp[count].planes[plane_index].length);
			}
		}
	}

	return CAMERASRC_SUCCESS;

CREATE_BUFFER_FAILED:
	camerasrc_destroy_buffer(p);

	return CAMERASRC_ERR_ALLOCATION;
}


int camerasrc_destroy_buffer(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;
	int count = 0;
	int plane_index = 0;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	/* buffer - sensor */
	if (p->buffer_sensor) {
		for (count = 0 ; count < p->format_sensor.num_buffers ; count++) {
			for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
				if (p->buffer_sensor[count].planes[plane_index].bo) {
					camsrc_info("[sensor] unref bo %p", p->buffer_sensor[count].planes[plane_index].bo);
					tbm_bo_unref(p->buffer_sensor[count].planes[plane_index].bo);
					p->buffer_sensor[count].planes[plane_index].bo = NULL;
					p->buffer_sensor[count].planes[plane_index].fd = 0;
					p->buffer_sensor[count].planes[plane_index].start = NULL;
					p->buffer_sensor[count].planes[plane_index].length = 0;
				} else {
					camsrc_info("[sensor] skip release buffer[%d] plane[%d]", count, plane_index);
				}
			}
		}

		free(p->buffer_sensor);
		p->buffer_sensor = NULL;
	} else {
		camsrc_warning("buffer_sensor is already NULL");
	}

	/* buffer - isp */
	if (p->buffer_isp) {
		for (count = 0 ; count < p->format_isp.num_buffers ; count++) {
			for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
				if (p->buffer_isp[count].planes[plane_index].bo) {
					camsrc_info("[isp] unref bo %p", p->buffer_isp[count].planes[plane_index].bo);
					tbm_bo_unref(p->buffer_isp[count].planes[plane_index].bo);
					p->buffer_isp[count].planes[plane_index].bo = NULL;
					p->buffer_isp[count].planes[plane_index].fd = 0;
					p->buffer_isp[count].planes[plane_index].start = NULL;
					p->buffer_isp[count].planes[plane_index].length = 0;
				} else {
					camsrc_info("[isp] skip release buffer[%d] plane[%d]", count, plane_index);
				}
			}
		}

		free(p->buffer_isp);
		p->buffer_isp = NULL;
	} else {
		camsrc_warning("buffer_isp is already NULL");
	}

	/* buffer - scc */
	if (p->buffer_scc) {
		for (count = 0 ; count < p->format_scc.num_buffers ; count++) {
			for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
				if (p->buffer_scc[count].planes[plane_index].bo) {
					camsrc_info("[scc] unref bo %p", p->buffer_scc[count].planes[plane_index].bo);
					tbm_bo_unref(p->buffer_scc[count].planes[plane_index].bo);
					p->buffer_scc[count].planes[plane_index].bo = NULL;
					p->buffer_scc[count].planes[plane_index].fd = 0;
					p->buffer_scc[count].planes[plane_index].start = NULL;
					p->buffer_scc[count].planes[plane_index].length = 0;
				} else {
					camsrc_info("[scc] skip release buffer[%d] plane[%d]", count, plane_index);
				}
			}
		}

		free(p->buffer_scc);
		p->buffer_scc = NULL;
	} else {
		camsrc_warning("buffer_scc is already NULL");
	}

	/* buffer - scp */
	if (p->buffer_scp) {
		for (count = 0 ; count < p->format_scp.num_buffers ; count++) {
			for (plane_index = 0 ; plane_index < MAX_PLANE_NUM ; plane_index++) {
				if (p->buffer_scp[count].planes[plane_index].bo) {
					camsrc_info("[scp] unref bo %p", p->buffer_scp[count].planes[plane_index].bo);
					tbm_bo_unref(p->buffer_scp[count].planes[plane_index].bo);
					p->buffer_scp[count].planes[plane_index].bo = NULL;
					p->buffer_scp[count].planes[plane_index].fd = 0;
					p->buffer_scp[count].planes[plane_index].start = NULL;
					p->buffer_scp[count].planes[plane_index].length = 0;
				} else {
					camsrc_info("[scp] skip release buffer[%d] plane[%d]", count, plane_index);
				}
			}
		}

		free(p->buffer_scp);
		p->buffer_scp = NULL;
	} else {
		camsrc_warning("buffer_scp is already NULL");
	}

	camsrc_info("done");

	return CAMERASRC_SUCCESS;
}


int camerasrc_start_preview_stream(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;
	int i = 0;
	int err = CAMERASRC_ERR_UNKNOWN;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	/* STATE OR PHASE CHECKING */
	if(CAMERASRC_STATE(p) == CAMERASRC_STATE_PREVIEW) {
		camsrc_info("already PREVIEW state. skip this func...");
		return CAMERASRC_SUCCESS;
	}
	if(CAMERASRC_STATE(p) != CAMERASRC_STATE_READY) {
		camsrc_warning("Invalid state transition");
	}

	/* check buffer */
	if (p->buffer_sensor == NULL || p->buffer_isp == NULL || p->buffer_scc == NULL || p->buffer_scp == NULL) {
		camsrc_error("buffer is NULL %p %p %p %p", p->buffer_sensor, p->buffer_isp, p->buffer_scc, p->buffer_scp);
		return CAMERASRC_ERR_INVALID_STATE;
	}

	/* P R E V I E W   F O R M A T   S E T T I N G */
	if (!(p->format_scp.pix_format == CAMERASRC_PIX_YUV422P ||
	      p->format_scp.pix_format == CAMERASRC_PIX_YUV420P || p->format_scp.pix_format == CAMERASRC_PIX_YV12 ||
	      p->format_scp.pix_format == CAMERASRC_PIX_SN12 || p->format_scp.pix_format == CAMERASRC_PIX_NV12 ||
	      p->format_scp.pix_format == CAMERASRC_PIX_SN21 || p->format_scp.pix_format == CAMERASRC_PIX_NV21 ||
	      p->format_scp.pix_format == CAMERASRC_PIX_UYVY || p->format_scp.pix_format == CAMERASRC_PIX_YUY2)) {
		camsrc_error("Invalid output format %d", p->format_scp.pix_format);
		return CAMERASRC_ERR_INVALID_FORMAT;
	}

	if (p->format_scp.colorspace != CAMERASRC_COL_RAW) {
		camsrc_error("Invalid store method.");
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	err = _camerasrc_dump_format(handle);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Format dump error");
		return err;
	}

	if (p->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY) {
		/* TODO */
	} else {
		int type = 0;
		int input = 0;

		/* sensor */
		err = _camerasrc_ioctl_s_ctrl(p, p->fd_sensor, V4L2_CID_IS_MIN_TARGET_FPS, 15);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR] V4L2_CID_IS_MIN_TARGET_FPS failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[SENSOR] V4L2_CID_IS_MIN_TARGET_FPS 15 done");

		err = _camerasrc_ioctl_s_ctrl(p, p->fd_sensor, V4L2_CID_IS_MAX_TARGET_FPS, 30);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR] V4L2_CID_IS_MAX_TARGET_FPS failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[SENSOR] V4L2_CID_IS_MAX_TARGET_FPS 30 done");

		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		p->format_sensor.img_size.width = FRONT_SENSOR_SIZE_WIDTH + SENSOR_MARGIN_WIDTH;
		p->format_sensor.img_size.height = FRONT_SENSOR_SIZE_HEIGHT + SENSOR_MARGIN_HEIGHT;
		p->format_sensor.pix_format = CAMERASRC_PIX_SBGGR12;

		err = _camerasrc_ioctl_s_fmt(p, p->fd_sensor, type, &p->format_sensor);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR - CAPTURE] _camerasrc_ioctl_s_fmt failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[SENSOR] S_FMT done");

		err = _camerasrc_ioctl_reqbufs(p, p->fd_sensor, p->format_sensor.num_buffers,
		                               type, V4L2_MEMORY_DMABUF, &p->format_sensor.num_buffers);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_sensor = TRUE;

		camsrc_info("[SENSOR] REQBUF done. count : %d", p->format_sensor.num_buffers);

		GST_INFO("            preview: _camerasrc_ioctl_s_parm");
		err = _camerasrc_ioctl_s_parm(p, p->fd_sensor, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, 1, 30);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("_camerasrc_ioctl_s_parm failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[SENSOR] S_PARM done");

		/* 3AA1 - OUTPUT */
		type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		p->format_3aa1.img_size.width = p->format_sensor.img_size.width;
		p->format_3aa1.img_size.height = p->format_sensor.img_size.height;
		p->format_3aa1.pix_format = CAMERASRC_PIX_SBGGR12;

		err = _camerasrc_ioctl_s_fmt(p, p->fd_3aa1, type, &p->format_3aa1);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - OUTPUT] _camerasrc_ioctl_s_fmt failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[3AA1 - OUTPUT] S_FMT done");

		err = _camerasrc_ioctl_reqbufs(p, p->fd_3aa1, p->format_3aa1.num_buffers,
		                               type, V4L2_MEMORY_DMABUF, &p->format_3aa1.num_buffers);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - OUTPUT] _camerasrc_ioctl_reqbufs failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_3aa1_output = TRUE;

		camsrc_info("[3AA1 - OUTPUT] REQBUF done. count : %d", p->format_3aa1.num_buffers);

		/* 3AA1 - CAPTURE */
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		p->format_3aa1.img_size.width = FRONT_SENSOR_SIZE_WIDTH;
		p->format_3aa1.img_size.height = FRONT_SENSOR_SIZE_HEIGHT;

		err = _camerasrc_ioctl_s_fmt(p, p->fd_3aa1, type, &p->format_3aa1);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - CAPTURE] _camerasrc_ioctl_s_fmt failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[3AA1 - CAPTURE] S_FMT done");

		err = _camerasrc_ioctl_reqbufs(p, p->fd_3aa1, p->format_3aa1.num_buffers,
		                               type, V4L2_MEMORY_DMABUF, &p->format_3aa1.num_buffers);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_3aa1_capture = TRUE;

		camsrc_info("[3AA1 - CAPTURE] REQBUF done. count : %d", p->format_3aa1.num_buffers);

		/* ISP */
		input = 0x11010;
		camsrc_info("[ISP] VIDIOC_S_INPUT %d", input);
		err = ioctl(p->fd_isp, VIDIOC_S_INPUT, &input);
		if (err < 0) {
			camsrc_error("[ISP] VIDIOC_S_INPUT failed. input 0x%x", input);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[ISP] VIDIOC_S_INPUT done");

		type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		p->format_isp.img_size.width = p->format_3aa1.img_size.width;
		p->format_isp.img_size.height = p->format_3aa1.img_size.height;
		p->format_isp.pix_format = CAMERASRC_PIX_SBGGR12;

		err = _camerasrc_ioctl_s_fmt(p, p->fd_isp, type, &p->format_isp);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[ISP - OUTPUT] _camerasrc_ioctl_s_fmt failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[ISP] S_FMT done");

		err = _camerasrc_ioctl_reqbufs(p, p->fd_isp, p->format_isp.num_buffers,
		                               type, V4L2_MEMORY_DMABUF, &p->format_isp.num_buffers);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[ISP - OUTPUT] _camerasrc_ioctl_reqbufs failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_isp = TRUE;

		camsrc_info("[ISP] REQBUF done. count : %d", p->format_isp.num_buffers);

		err = _camerasrc_ioctl_s_ctrl(p, p->fd_isp, V4L2_CID_IS_SET_SETFILE, 6);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[ISP] V4L2_CID_IS_SET_SETFILE failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[ISP] IS_SET_SETFILE 6 done");

		/* SCC */
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		p->format_scc.img_size.width = p->format_3aa1.img_size.width;
		p->format_scc.img_size.height = p->format_3aa1.img_size.height;
		p->format_scc.pix_format = CAMERASRC_PIX_NV12;

		err = _camerasrc_ioctl_s_fmt(p, p->fd_scc, type, &p->format_scc);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCC - CAPTURE] _camerasrc_ioctl_s_fmt failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[SCC] S_FMT done");

		err = _camerasrc_ioctl_reqbufs(p, p->fd_scc, p->format_scc.num_buffers,
		                               type, V4L2_MEMORY_DMABUF, &p->format_scc.num_buffers);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCC - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_scc = TRUE;

		camsrc_info("[SCC] REQBUF done. count : %d", p->format_scc.num_buffers);

		/* SCP */
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		/* FIXME */
		p->format_scp.pix_format = CAMERASRC_PIX_SN21;

		err = _camerasrc_ioctl_s_fmt(p, p->fd_scp, type, &p->format_scp);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCP - CAPTURE] _camerasrc_ioctl_s_fmt failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[SCP] S_FMT done");

		err = _camerasrc_ioctl_reqbufs(p, p->fd_scp, p->format_scp.num_buffers,
		                               type, V4L2_MEMORY_DMABUF, &p->format_scp.num_buffers);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCP - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_scp = TRUE;

		camsrc_info("[SCP] REQBUF done. count : %d", p->format_scp.num_buffers);

		err = _camerasrc_ioctl_s_ctrl(p, p->fd_scp, V4L2_CID_IS_COLOR_RANGE, 0);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCP] V4L2_CID_IS_COLOR_RANGE failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		camsrc_info("[SCP] V4L2_CID_IS_COLOR_RANGE done");
	}

	/* QBUF */
	if (p->buffer_sensor) {
		err = _camerasrc_set_shot_meta(p, GRP_SENSOR, 0, 0, (struct camera2_shot_ext *)p->buffer_sensor[0].planes[1].start);
		if (err != CAMERASRC_SUCCESS) {
			camsrc_error("[SENSOR] _camerasrc_set_shot_meta index %d, failed[%x]", i, err);
			return err;
		}

		for (i = 0 ; i < p->format_sensor.num_buffers ; i++) {
			err = _camerasrc_ioctl_qbuf(p,
			                            p->fd_sensor,
			                            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			                            V4L2_MEMORY_DMABUF,
			                            i,
			                            &p->buffer_sensor[i]);
			if (err != CAMERASRC_SUCCESS) {
				camsrc_error("[SENSOR] _camerasrc_ioctl_qbuf index %d, failed[%x]", i, err);
				return CAMERASRC_ERR_IO_CONTROL;
			}
		}

		camsrc_info("[SENSOR] QBUF done - %d", p->format_sensor.num_buffers);
	}

	if (p->buffer_scp) {
		for (i = 0 ; i < p->format_scp.num_buffers ; i++) {
			err = _camerasrc_set_shot_meta(p, GRP_SCP, i, 0, (struct camera2_shot_ext *)p->buffer_scp[i].planes[2].start);
			if (err != CAMERASRC_SUCCESS) {
				camsrc_error("[SCP] _camerasrc_set_shot_meta index %d, failed[%x]", i, err);
				return err;
			}

			err = _camerasrc_ioctl_qbuf(p,
			                            p->fd_scp,
			                            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			                            V4L2_MEMORY_DMABUF,
			                            i,
			                            &p->buffer_scp[i]);
			if (err != CAMERASRC_SUCCESS) {
				camsrc_error("[SCP] _camerasrc_ioctl_qbuf index %d, failed[%x]", i, err);
				return CAMERASRC_ERR_IO_CONTROL;
			}
		}

		camsrc_info("[SCP] QBUF done - %d", p->format_scp.num_buffers);
	}

	/* STREAM ON */
	err = _camerasrc_ioctl_stream(p, p->fd_sensor, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, TRUE);
	if (err != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("[SENSOR] _camerasrc_ioctl_stream failed[%x] : %s", err, err_msg);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	p->streamon_sensor = TRUE;

	camsrc_info("[SENSOR] STREAMON done");

	err = _camerasrc_ioctl_stream(p, p->fd_3aa1, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, TRUE);
	if (err != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("[3AA1 - CAPTURE] _camerasrc_ioctl_stream failed[%x] : %s", err, err_msg);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	p->streamon_3aa1_capture = TRUE;

	camsrc_info("[3AA1 - CAPTURE] STREAMON done");

	err = _camerasrc_ioctl_stream(p, p->fd_3aa1, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, TRUE);
	if (err != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("[3AA1 - OUTPUT] _camerasrc_ioctl_stream failed[%x] : %s", err, err_msg);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	p->streamon_3aa1_output = TRUE;

	camsrc_info("[3AA1 - OUTPUT] STREAMON done");

	err = _camerasrc_ioctl_stream(p, p->fd_scc, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, TRUE);
	if (err != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("[SCC] _camerasrc_ioctl_stream failed[%x] : %s", err, err_msg);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	p->streamon_scc = TRUE;

	camsrc_info("[SCC] STREAMON done");

	err = _camerasrc_ioctl_stream(p, p->fd_scp, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, TRUE);
	if (err != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("[SCP] _camerasrc_ioctl_stream failed[%x] : %s", err, err_msg);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	p->streamon_scp = TRUE;

	camsrc_info("[SCP] STREAMON done");

	err = _camerasrc_ioctl_stream(p, p->fd_isp, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, TRUE);
	if (err != CAMERASRC_SUCCESS) {
		strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("[ISP] _camerasrc_ioctl_stream failed[%x] : %s", err, err_msg);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	p->streamon_isp = TRUE;

	camsrc_info("[ISP] STREAMON done");

	if (p->cur_dev_id == CAMERASRC_DEV_ID_SECONDARY) {
		err = _camerasrc_ioctl_s_ctrl(p, p->fd_sensor, V4L2_CID_IS_S_STREAM, TRUE);
		if (err != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR] V4L2_CID_IS_S_STREAM failed[%x] : %s", err, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_sensor_sub = TRUE;

		camsrc_info("[SENSOR] IS_S_STREAM 1 done");
	}

	if (p->buffer_scc) {
		for (i = 0 ; i < p->format_scc.num_buffers ; i++) {
			err = _camerasrc_set_shot_meta(p, GRP_SCC, i, 0, (struct camera2_shot_ext *)p->buffer_scc[i].planes[1].start);
			if (err != CAMERASRC_SUCCESS) {
				camsrc_error("[SCC] _camerasrc_set_shot_meta index %d, failed[%x]", i, err);
				return err;
			}

			err = _camerasrc_ioctl_qbuf(p,
			                            p->fd_scc,
			                            V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			                            V4L2_MEMORY_DMABUF,
			                            i,
			                            &p->buffer_scc[i]);
			if (err != CAMERASRC_SUCCESS) {
				camsrc_error("[SCC] _camerasrc_ioctl_qbuf index %d, failed[%x]", i, err);
				return CAMERASRC_ERR_IO_CONTROL;
			}
		}

		camsrc_info("[SCC] QBUF done - %d", p->format_scc.num_buffers);
	}

	CAMERASRC_SET_STATE(p, CAMERASRC_STATE_PREVIEW);

	return CAMERASRC_SUCCESS;
}


int camerasrc_stop_stream(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;
	int ret = CAMERASRC_SUCCESS;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_STILL &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_PREVIEW &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_VIDEO &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_AF_IN_PROGRESS) {
		camsrc_warning("Stop stream called [STREAM-NOT-STARTED STATE]");
	}

	camsrc_info("Change to READY state first for preventing to check Q/DQ after stop");
	CAMERASRC_SET_STATE(p, CAMERASRC_STATE_READY);

	LOCK(p);
	p->timeperframe.denominator = 0;
	p->timeperframe.numerator = 0;
	p->first_frame = 1;
	UNLOCK(p);

	/* STREAM OFF */
	if (p->cur_dev_id == CAMERASRC_DEV_ID_SECONDARY &&
	    p->streamon_sensor_sub) {
		ret = _camerasrc_ioctl_s_ctrl(p, p->fd_sensor, V4L2_CID_IS_S_STREAM, 0);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR] V4L2_CID_IS_S_STREAM failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_sensor_sub = FALSE;

		camsrc_info("[SENSOR] V4L2_CID_IS_S_STREAM 0 done");
	}

	if (p->streamon_sensor) {
		ret = _camerasrc_ioctl_stream(p, p->fd_sensor, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, FALSE);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR] _camerasrc_ioctl_stream failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_sensor = FALSE;

		camsrc_info("[SENSOR] STREAMOFF done");
	}

	if (p->streamon_3aa1_output) {
		ret = _camerasrc_ioctl_stream(p, p->fd_3aa1, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, FALSE);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - OUTPUT] _camerasrc_ioctl_stream failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_3aa1_output = FALSE;

		camsrc_info("[3AA1 - OUTPUT] STREAMOFF done");
	}

	if (p->streamon_3aa1_capture) {
		ret = _camerasrc_ioctl_stream(p, p->fd_3aa1, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, FALSE);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - CAPTURE] _camerasrc_ioctl_stream failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_3aa1_capture = FALSE;

		camsrc_info("[3AA1 - CAPTURE] STREAMOFF done");
	}

	if (p->streamon_isp) {
		ret = _camerasrc_ioctl_stream(p, p->fd_isp, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, FALSE);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[ISP] _camerasrc_ioctl_stream failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_isp = FALSE;

		camsrc_info("[ISP] STREAMOFF done");
	}

	if (p->streamon_scc) {
		ret = _camerasrc_ioctl_stream(p, p->fd_scc, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, FALSE);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCC] _camerasrc_ioctl_stream failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_scc = FALSE;

		camsrc_info("[SCC] STREAMOFF done");
	}

	if (p->streamon_scp) {
		ret = _camerasrc_ioctl_stream(p, p->fd_scp, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, FALSE);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCP] _camerasrc_ioctl_stream failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->streamon_scp = FALSE;

		camsrc_info("[SCP] STREAMOFF done");
	}

	/**
	 * REQBUF 0
	 */
	/* SENSOR */
	if (p->reqbuf_sensor) {
		ret = _camerasrc_ioctl_reqbufs(p, p->fd_sensor, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_DMABUF, NULL);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SENSOR - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_sensor = FALSE;

		camsrc_info("[SENSOR] REQBUF 0 done");
	}

	/* 3AA1 - OUTPUT */
	if (p->reqbuf_3aa1_output) {
		ret = _camerasrc_ioctl_reqbufs(p, p->fd_3aa1, 0, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_DMABUF, NULL);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - OUTPUT] _camerasrc_ioctl_reqbufs failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_3aa1_output = FALSE;

		camsrc_info("[3AA1 - OUTPUT] REQBUF 0 done");
	}

	/* 3AA1 - CAPTURE */
	if (p->reqbuf_3aa1_capture) {
		ret = _camerasrc_ioctl_reqbufs(p, p->fd_3aa1, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_DMABUF, NULL);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[3AA1 - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_3aa1_capture = FALSE;

		camsrc_info("[3AA1 - CAPTURE] REQBUF 0 done");
	}

	/* ISP */
	if (p->reqbuf_isp) {
		ret = _camerasrc_ioctl_reqbufs(p, p->fd_isp, 0, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_MEMORY_DMABUF, NULL);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[ISP - OUTPUT] _camerasrc_ioctl_reqbufs failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_isp = FALSE;

		camsrc_info("[ISP] REQBUF 0 done");
	}

	/* SCC */
	if (p->reqbuf_scc) {
		ret = _camerasrc_ioctl_reqbufs(p, p->fd_isp, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_DMABUF, NULL);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCC - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_scc = FALSE;

		camsrc_info("[SCC] REQBUF 0 done");
	}

	/* SCP */
	if (p->reqbuf_scp) {
		ret = _camerasrc_ioctl_reqbufs(p, p->fd_isp, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, V4L2_MEMORY_DMABUF, NULL);
		if (ret != CAMERASRC_SUCCESS) {
			strerror_r(p->errnum, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
			camsrc_error("[SCP - CAPTURE] _camerasrc_ioctl_reqbufs failed[%x] : %s", ret, err_msg);
			return CAMERASRC_ERR_IO_CONTROL;
		}

		p->reqbuf_scp = FALSE;

		camsrc_info("[SCP] REQBUF 0 done");
	}

	camsrc_info("VIDIOC_REQBUFS 0 done");

	return CAMERASRC_SUCCESS;
}


int camerasrc_wait_frame_available(camsrc_handle_t handle, int timeout)
{
	camerasrc_handle_t *p = NULL;

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_STILL &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_PREVIEW &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_VIDEO &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_AF_IN_PROGRESS) {
		camsrc_warning("Invalid state transition" );
	}

	return _camerasrc_wait_frame_available(p, timeout);
}


int camerasrc_queue_buffer(camsrc_handle_t handle, int buf_index)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_STILL &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_PREVIEW &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_VIDEO &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_AF_IN_PROGRESS ) {
		camsrc_warning("Invalid state transition");
	}

	err = _camerasrc_queue_buffer(p, buf_index);

	return err;
}


int camerasrc_dequeue_buffer(camsrc_handle_t handle, int *buf_index, camerasrc_buffer_t *buffer, camerasrc_buffer_t *thm_buffer)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_STILL &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_PREVIEW &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_VIDEO &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_AF_IN_PROGRESS) {
		camsrc_warning("Invalid state transition");
	}

	err = _camerasrc_dequeue_buffer(p, buf_index, buffer, thm_buffer);

	return err;
}


int camerasrc_read_frame(camsrc_handle_t handle, camerasrc_buffer_t *main_img_buffer, camerasrc_buffer_t *thm_img_buffer, int *buffer_index)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("ENTER");

	if (!handle || !main_img_buffer || !thm_img_buffer || !buffer_index) {
		camsrc_error("handle(%p)main(%p)thm(%p)index(%p) is null",
		                        handle, main_img_buffer, thm_img_buffer, buffer_index);
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_STILL &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_PREVIEW &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_VIDEO &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_AF_IN_PROGRESS) {
		camsrc_warning("Invalid state transition");
	}

	GST_INFO("                Stillshot select()");
	err = _camerasrc_wait_frame_available(p, CAMERASRC_TIMEOUT_CRITICAL_VALUE);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Frame waiting error, [%x]", err);
		return err;
	}

	/* Buffer DQ */
	GST_INFO("                Stillshot VIDIOC_DQBUF");
	err = _camerasrc_dequeue_buffer(p, buffer_index, main_img_buffer, thm_img_buffer);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Dequeue frame error, [%x]", err);
		return err;
	}

	camsrc_info("DEQUEUED Index : %d", *buffer_index);

	return CAMERASRC_SUCCESS;
}


int camerasrc_set_focused_callback(camsrc_handle_t handle, camerasrc_callback_t cb, void *usr_data)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_REALIZED &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_READY) {
		camsrc_warning("Invalid state transition");
	}

	LOCK(p);
	p->af_cb = cb;
	if (usr_data != NULL) {
		p->af_usr_data = usr_data;
	}
	UNLOCK(p);

	return CAMERASRC_SUCCESS;
}


int camerasrc_set_autofocusing_area(camsrc_handle_t handle, camerasrc_rect_t *rect)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_PHASE(p) != CAMERASRC_PHASE_RUNNING) {
		camsrc_warning("Invalid state transition");
	}

	err = _camerasrc_set_autofocusing_area(p, rect);

	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Set autofocusing area error");
		return err;
	}

	return CAMERASRC_SUCCESS;
}


int camerasrc_get_autofocusing_area(camsrc_handle_t handle, camerasrc_rect_t *rect)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_PHASE(p) != CAMERASRC_PHASE_RUNNING) {
		camsrc_warning("Invalid state transition");
	}

	err = _camerasrc_get_autofocusing_area(p, rect);

	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Get autofocusing area error");
		return err;
	}

	return CAMERASRC_SUCCESS;
}


int camerasrc_start_autofocusing(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (!(CAMERASRC_STATE(p) == CAMERASRC_STATE_PREVIEW ||
	      CAMERASRC_STATE(p) == CAMERASRC_STATE_VIDEO ||
	      CAMERASRC_STATE(p) == CAMERASRC_STATE_AF_IN_PROGRESS)) {
		camsrc_error("Invalid state [%d]", CAMERASRC_STATE(p));
		return CAMERASRC_ERR_INVALID_STATE;
	}

	/* START AF */
	err = _camerasrc_start_autofocusing(p);

	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Create autofocusing thread error");
		return err;
	}

	camsrc_info("Set state to [AF-IN-PROGRESS]!!");
	CAMERASRC_SET_STATE(p, CAMERASRC_STATE_AF_IN_PROGRESS);

	return CAMERASRC_SUCCESS;
}


int camerasrc_stop_autofocusing(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (!(CAMERASRC_STATE(p) == CAMERASRC_STATE_PREVIEW ||
	      CAMERASRC_STATE(p) == CAMERASRC_STATE_VIDEO ||
	      CAMERASRC_STATE(p) == CAMERASRC_STATE_AF_IN_PROGRESS)) {
		camsrc_error("Invalid state [%d]", CAMERASRC_STATE(p));
		return CAMERASRC_ERR_INVALID_STATE;
	}

	/* STOP AF */
	camsrc_info("AF_STOP called!!");
	err = _camerasrc_stop_autofocusing(p);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Stop autofocusing error");
		return err;
	}

	if (CAMERASRC_STATE(p) > CAMERASRC_STATE_READY) {
		camsrc_info("Set state to [PREVIEW] again!!");
		CAMERASRC_SET_STATE(p, CAMERASRC_STATE_PREVIEW);
	} else {
		camsrc_info("Do not change state");
	}

	return CAMERASRC_SUCCESS;
}


int camerasrc_release_autofocusing(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_PREVIEW &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_VIDEO &&
	    CAMERASRC_STATE(p) != CAMERASRC_STATE_AF_IN_PROGRESS) {
		return CAMERASRC_SUCCESS;
	}

	/* STOP AF */
	camsrc_info("AF_RELEASE called!!");
	err = _camerasrc_release_autofocusing(p);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Stop autofocusing error");
		return err;
	}

	if (CAMERASRC_STATE(p) > CAMERASRC_STATE_READY) {
		camsrc_info("Set state to [PREVIEW] again!!");
		CAMERASRC_SET_STATE(p, CAMERASRC_STATE_PREVIEW);
	} else {
		camsrc_info("Do not change state");
	}

	return CAMERASRC_SUCCESS;
}


int camerasrc_init_autofocusing_mode(camsrc_handle_t handle, camerasrc_af_mode_t af_mode, camerasrc_af_scan_range_t af_range)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	p->cur_af_mode = af_mode;
	p->cur_af_range = af_range;

	if (CAMERASRC_STATE(p) < CAMERASRC_STATE_PREVIEW) {
		camsrc_info("Skip focus mode[%d], range[%d], it will be set when start preview",
		            af_mode, af_range);
		return CAMERASRC_SUCCESS;
	}

	err = _camerasrc_init_autofocusing_mode(p);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Init autofocusing mode error");
		return err;
	}

	return CAMERASRC_SUCCESS;
}


int camerasrc_get_autofocusing_mode(camsrc_handle_t handle, camerasrc_af_mode_t *af_mode, camerasrc_af_scan_range_t *af_range)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (af_mode) {
		*af_mode = p->cur_af_mode;
	}

	if (af_range) {
		*af_range = p->cur_af_range;
	}

	return CAMERASRC_SUCCESS;
}


int camerasrc_get_autofocusing_status(camsrc_handle_t handle, camerasrc_auto_focus_status_t *af_status)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	if (af_status == NULL) {
		camsrc_error("*af_status is NULL");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	*af_status = p->af_status;

	camsrc_info("[camerasrc_get_autofocusing_status] %d", *af_status);

	return CAMERASRC_SUCCESS;
}


/**** O U T P U T    C O N T R O L    O P E R A T I O N ****/

int camerasrc_set_timeperframe(camsrc_handle_t handle, camerasrc_frac_t *frac)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_PHASE(p) != CAMERASRC_PHASE_RUNNING) {
		camsrc_warning("Invalid phase, but can go");
	}

	camsrc_info("Numerator = %d, Denominator = %d", frac->numerator, frac->denominator);

	LOCK(p);
	p->timeperframe.numerator = frac->numerator;
	p->timeperframe.denominator = frac->denominator;
	UNLOCK(p);

	return CAMERASRC_SUCCESS;
}


int camerasrc_set_format(camsrc_handle_t handle, camerasrc_format_t *fmt)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_STATE(p) != CAMERASRC_STATE_READY) {
		camsrc_error("Invalid state");
	}

	err = _camerasrc_dump_format(handle);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Format dump error");
		return err;
	}

	memset(&p->format_scp, 0x0, sizeof(camerasrc_format_t));

	p->format_scp.colorspace = fmt->colorspace;
	p->format_scp.bytesperline = fmt->bytesperline;
	p->format_scp.img_size.width = fmt->img_size.width;
	p->format_scp.img_size.height = fmt->img_size.height;
	p->format_scp.capture_size.width = fmt->capture_size.width;
	p->format_scp.capture_size.height = fmt->capture_size.height;
	p->format_scp.thumb_size.width = fmt->thumb_size.width;
	p->format_scp.thumb_size.height= fmt->thumb_size.height;
	p->format_scp.pix_format = fmt->pix_format;
	p->format_scp.quality = fmt->quality;
	p->format_scp.sizeimage = fmt->sizeimage;
	p->format_scp.rotation = fmt->rotation;
	p->format_scp.num_buffers = CAMERASRC_PREVIEW_BUFFER_NUM;

	switch (fmt->pix_format) {
	case CAMERASRC_PIX_YUV420:
	case CAMERASRC_PIX_SN12:
	case CAMERASRC_PIX_NV12:
	case CAMERASRC_PIX_SN21:
	case CAMERASRC_PIX_NV21:
		p->format_scp.num_planes = 2;
		break;
	case CAMERASRC_PIX_YUY2:
	case CAMERASRC_PIX_UYVY:
	case CAMERASRC_PIX_RGGB8:
	case CAMERASRC_PIX_RGGB10:
	case CAMERASRC_PIX_RGB565:
		p->format_scp.num_planes = 1;
		break;
	case CAMERASRC_PIX_YUV422P:
	case CAMERASRC_PIX_YUV420P:
	case CAMERASRC_PIX_YV12:
		p->format_scp.num_planes = 3;
		break;
	default:
		p->format_scp.num_planes = 3;
		camsrc_error("Invalid output format [%d]", fmt->pix_format);
		break;
	}

	err = _camerasrc_dump_format(handle);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Format dump error");
		return err;
	}

	camsrc_info("leave");

	return err;
}


int camerasrc_get_format(camsrc_handle_t handle, camerasrc_format_t *fmt)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_PHASE(p) != CAMERASRC_PHASE_RUNNING) {
		camsrc_warning("Invalid phase");
	}

	memcpy(fmt, &(p->format_scp), sizeof(camerasrc_format_t));

	camsrc_info("leave");

	return CAMERASRC_SUCCESS;
}


int camerasrc_get_frame_data(camsrc_handle_t handle, camerasrc_frame_data_t *data)
{
	camerasrc_handle_t *p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_PHASE(p) != CAMERASRC_PHASE_RUNNING) {
		camsrc_warning("Invalid state transition");
	}

	err = _camerasrc_get_frame_data(p, data);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("toggle auto exposure failed");
		return err;
	}

	return err;
}


static int _camerasrc_dump_format(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	camsrc_info("---------FORMAT SETTING DUMP--------");
	camsrc_info("- Image size : %d x %d", p->format_scp.img_size.width, p->format_scp.img_size.height);
	camsrc_info("- Capture size : %d x %d", p->format_scp.capture_size.width, p->format_scp.capture_size.height);
	camsrc_info("- Thumbnail size : %d x %d", p->format_scp.thumb_size.width, p->format_scp.thumb_size.height);
	camsrc_info("- Pixel format : %d", p->format_scp.pix_format);
	camsrc_info("- Bytes per line : %d", p->format_scp.bytesperline);
	camsrc_info("- Image size in bytes : %d", p->format_scp.sizeimage);
	camsrc_info("- Colorspace : %d", p->format_scp.colorspace);
	camsrc_info("- Rotation : %d", p->format_scp.rotation);
	camsrc_info("------------------------------------");

	return CAMERASRC_SUCCESS;
}


int camerasrc_get_exif_info(camsrc_handle_t handle, camerasrc_exif_t *exif_struct)
{
	camerasrc_handle_t* p = NULL;
	int err = CAMERASRC_ERR_UNKNOWN;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	if (CAMERASRC_PHASE(p) != CAMERASRC_PHASE_RUNNING) {
		camsrc_warning("Invalid state transition");
	}

	err = _camerasrc_get_exif_info(p, (camerasrc_buffer_t*)exif_struct);
	if (err != CAMERASRC_SUCCESS) {
		camsrc_error("Get exif information string failed");
	}

	return err;
}


int camerasrc_set_vflip(camsrc_handle_t handle, int vflip)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	p->vflip = vflip;

	if (CAMERASRC_STATE(p) > CAMERASRC_STATE_READY) {
		CAMERASRC_SET_CMD(_CAMERASRC_CMD_VFLIP, &(p->vflip));
	}

	camsrc_info("leave - %d", p->vflip);

	return CAMERASRC_SUCCESS;
}


int camerasrc_set_hflip(camsrc_handle_t handle, int hflip)
{
	camerasrc_handle_t *p = NULL;

	camsrc_info("enter");

	if (handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	p->hflip = hflip;
	if (CAMERASRC_STATE(p) > CAMERASRC_STATE_READY) {
		CAMERASRC_SET_CMD(_CAMERASRC_CMD_HFLIP, &(p->hflip));
	}

	camsrc_info("leave - %d", p->hflip);

	return CAMERASRC_SUCCESS;
}


int camerasrc_tbm_init(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;

	if (handle == NULL) {
		camsrc_error("handle is NULL");
		return FALSE;
	}

	camsrc_warning("start");

	p = CAMERASRC_HANDLE(handle);

	if (p->bufmgr) {
		camsrc_warning("bufmgr is already created %p", p->bufmgr);
		return TRUE;
	}

	/* tbm buffer mgr init */
	p->bufmgr = tbm_bufmgr_init(-1);
	if (!p->bufmgr) {
		camsrc_error("tbm_bufmgr_init failed");
		return FALSE;
	}

	camsrc_warning("tbm_bufmgr %p", p->bufmgr);

	return TRUE;
}


int camerasrc_tbm_deinit(camsrc_handle_t handle)
{
	camerasrc_handle_t *p = NULL;

	if (handle == NULL) {
		camsrc_error("handle is NULL");
		return FALSE;
	}

	p = CAMERASRC_HANDLE(handle);

	camsrc_warning("enter - tbm bufmgr %p", p->bufmgr);

	if (p->bufmgr) {
		tbm_bufmgr_deinit(p->bufmgr);
		p->bufmgr = NULL;
		camsrc_warning("release tbm_bufmgr done");
	}

	camsrc_warning("done");

	return TRUE;
}


int camerasrc_tbm_alloc_buffer(camsrc_handle_t handle, int size, tbm_bo *bo, int *dma_buf_fd, unsigned char **vaddr)
{
	camerasrc_handle_t *p = NULL;
	tbm_bo bo_alloc = NULL;
	tbm_bo_handle bo_handle_fd;
	tbm_bo_handle bo_handle_vaddr;

	if (!handle || !bo || !dma_buf_fd || !vaddr) {
		camsrc_error("pointer(%p %p %p %p) is NULL", handle, bo, dma_buf_fd, vaddr);
		return FALSE;
	}

	if (size <= 0) {
		camsrc_error("size[%d] is too small", size);
		return FALSE;
	}

	p = CAMERASRC_HANDLE(handle);

	if (p->bufmgr == NULL) {
		camsrc_error("tbm bufmgr is NULL");
		return FALSE;
	}

	/* alloc tbm buffer object */
	bo_alloc = tbm_bo_alloc(p->bufmgr, size, TBM_BO_DEFAULT);
	if (!bo_alloc) {
		camsrc_error("tbm_bo_alloc failed");
		return FALSE;
	}

	/* get dmabuf fd */
	bo_handle_fd = tbm_bo_get_handle(bo_alloc, TBM_DEVICE_MM);
	if (bo_handle_fd.u32 == 0) {
		tbm_bo_unref(bo_alloc);
		camsrc_error("tbm_bo_get_handle TBM_DEVICE_MM failed");
		return FALSE;
	}

	/* get virtual address */
	bo_handle_vaddr = tbm_bo_get_handle(bo_alloc, TBM_DEVICE_CPU);
	if (bo_handle_vaddr.ptr == NULL) {
		tbm_bo_unref(bo_alloc);
		camsrc_error("tbm_bo_get_handle TBM_DEVICE_CPU failed");
		return FALSE;
	}

	*bo = bo_alloc;
	*dma_buf_fd = bo_handle_fd.u32;
	*vaddr = bo_handle_vaddr.ptr;

	camsrc_warning("bo %p, dmabuf fd %u, vaddr %p, size %d",
	               *bo, *dma_buf_fd, *vaddr, size);

	return TRUE;
}


/* For Query functionalities */
int camerasrc_read_basic_dev_info(camerasrc_dev_id_t dev_id, camerasrc_caps_info_t* caps_info)
{
	int err = CAMERASRC_ERR_UNKNOWN;
	int nread = 0;
	char* store_path = NULL;
	FILE *fp = NULL;

	camsrc_info("enter");

	if(dev_id == CAMERASRC_DEV_ID_PRIMARY)
		store_path = CAMERASRC_PRIMARY_BASIC_INFO_PATH;
	else if (dev_id == CAMERASRC_DEV_ID_SECONDARY)
		store_path = CAMERASRC_SECONDARY_BASIC_INFO_PATH;
	else
	{
		camsrc_error("Unsupported device ID");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}

	fp = fopen(store_path, "rb");
	if(fp)
	{
		fseek(fp, 0, SEEK_SET);
		nread = fread(caps_info, 1, sizeof(camerasrc_caps_info_t), fp);
		camsrc_info("Need to be read : %d / Actual read : %d", sizeof(camerasrc_caps_info_t), nread);
		fclose(fp);
	}
	else
		return CAMERASRC_ERR_ALLOCATION;

	err = CAMERASRC_SUCCESS;
	camsrc_info("leave");
	return err;
}


int camerasrc_read_misc_dev_info(camerasrc_dev_id_t dev_id, camerasrc_ctrl_list_info_t* ctrl_info)
{
	int err = CAMERASRC_ERR_UNKNOWN;
	camsrc_info("enter");

	int nread = 0;
	FILE *fp = NULL;
	char* store_path = NULL;

	if(dev_id == CAMERASRC_DEV_ID_PRIMARY)
		store_path = CAMERASRC_PRIMARY_MISC_INFO_PATH;
	else if (dev_id == CAMERASRC_DEV_ID_SECONDARY)
		store_path = CAMERASRC_SECONDARY_MISC_INFO_PATH;
	else
	{
		camsrc_error("Unsupported device ID");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}

	fp = fopen(store_path, "rb");
	if(fp)
	{
		fseek(fp, 0, SEEK_SET);
		nread = fread(ctrl_info, 1, sizeof(camerasrc_ctrl_list_info_t), fp);
		camsrc_info("Need to be read : %d / Actual read : %d", sizeof(camerasrc_ctrl_list_info_t), nread);
		fclose(fp);
	}
	else
		return CAMERASRC_ERR_ALLOCATION;

	err = CAMERASRC_SUCCESS;
	camsrc_info("leave");
	return err;
}


int camerasrc_read_extra_dev_info(camerasrc_dev_id_t dev_id, camerasrc_extra_info_t* extra_info)
{
	int err = CAMERASRC_ERR_UNKNOWN;
	camsrc_info("enter");

	int nread = 0;
	FILE *fp = NULL;
	char* store_path = NULL;

	if(dev_id == CAMERASRC_DEV_ID_PRIMARY)
		store_path = CAMERASRC_PRIMARY_EXTRA_INFO_PATH;
	else if (dev_id == CAMERASRC_DEV_ID_SECONDARY)
		store_path = CAMERASRC_SECONDARY_EXTRA_INFO_PATH;
	else
	{
		camsrc_error("Unsupported device ID");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}

	fp = fopen(store_path, "rb");
	if(fp)
	{
		fseek(fp, 0, SEEK_SET);
		nread = fread(extra_info, 1, sizeof(camerasrc_extra_info_t), fp);
		camsrc_info("Need to be read : %d / Actual read : %d", sizeof(camerasrc_extra_info_t), nread);
		fclose(fp);
	}
	else
		return CAMERASRC_ERR_ALLOCATION;

	err = CAMERASRC_SUCCESS;
	camsrc_info("leave");
	return err;
}


int camerasrc_write_basic_dev_info(camsrc_handle_t handle, camerasrc_caps_info_t* caps_info)
{
	camerasrc_handle_t* p = NULL;
	char* store_path = NULL;
	FILE *fp = NULL;

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	int nwrite = 0;

	if(p->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY)
	{
		camsrc_info("Primary(Mega) camera capabilities info will be written..");
		store_path = CAMERASRC_PRIMARY_BASIC_INFO_PATH;
	}
	else if (p->cur_dev_id == CAMERASRC_DEV_ID_SECONDARY)
	{
		camsrc_info("Secondary(VGA) camera capabilities info will be written..");
		store_path = CAMERASRC_SECONDARY_BASIC_INFO_PATH;
	}
	else
	{
		camsrc_error("Unsupported device ID");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}
	camsrc_info("PATH = %s", store_path);

	fp = fopen(store_path, "wb");
	if(fp)
	{
		fseek(fp, 0, SEEK_SET);
		nwrite = fwrite(caps_info, 1, sizeof(camerasrc_caps_info_t), fp);
		camsrc_info("Need to be written : %d / Actual written : %d", sizeof(camerasrc_caps_info_t), nwrite);
		fclose(fp);
	}
	else
		return CAMERASRC_ERR_ALLOCATION;

	camsrc_info("leave");
	return CAMERASRC_SUCCESS;
}


int camerasrc_write_misc_dev_info(camsrc_handle_t handle, camerasrc_ctrl_list_info_t* ctrl_info)
{
	camerasrc_handle_t* p = NULL;

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	int nwrite = 0;
	FILE *fp = NULL;

	char* store_path = NULL;

	if(p->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY)
	{
		camsrc_info("Primary(Mega) camera controls info will be written..");
		store_path = CAMERASRC_PRIMARY_MISC_INFO_PATH;
	}
	else if (p->cur_dev_id == CAMERASRC_DEV_ID_SECONDARY)
	{
		camsrc_info("Secondary(VGA) camera controls info will be written..");
		store_path = CAMERASRC_SECONDARY_MISC_INFO_PATH;
	}
	else
	{
		camsrc_error("Unsupported device ID");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}

	fp = fopen(store_path, "wb");
	if(fp)
	{
		fseek(fp, 0, SEEK_SET);
		nwrite = fwrite(ctrl_info, 1, sizeof(camerasrc_ctrl_list_info_t), fp);
		camsrc_info("Need to be written : %d / Actual written : %d", sizeof(camerasrc_ctrl_list_info_t), nwrite);
		fclose(fp);
	}
	else
		return CAMERASRC_ERR_ALLOCATION;

	camsrc_info("leave");
	return CAMERASRC_SUCCESS;
}


int camerasrc_write_extra_dev_info(camsrc_handle_t handle, camerasrc_extra_info_t* extra_info)
{
	camerasrc_handle_t* p = NULL;

	camsrc_info("enter");

	if(handle == NULL) {
		camsrc_error("handle is null");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	p = CAMERASRC_HANDLE(handle);

	int nwrite = 0;
	FILE *fp = NULL;

	char* store_path = NULL;

	if(p->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY)
	{
		camsrc_info("Primary(Mega) extra controls info will be written..");
		store_path = CAMERASRC_PRIMARY_EXTRA_INFO_PATH;
	}
	else if (p->cur_dev_id == CAMERASRC_DEV_ID_SECONDARY)
	{
		camsrc_info("Secondary(VGA) extra controls info will be written..");
		store_path = CAMERASRC_SECONDARY_EXTRA_INFO_PATH;
	}
	else
	{
		camsrc_error("Unsupported device ID");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}

	fp = fopen(store_path, "wb");
	if(fp)
	{
		fseek(fp, 0, SEEK_SET);
		nwrite = fwrite(extra_info, 1, sizeof(camerasrc_extra_info_t), fp);
		camsrc_info("Need to be written : %d / Actual written : %d", sizeof(camerasrc_extra_info_t), nwrite);
		fclose(fp);
	}
	else
		return CAMERASRC_ERR_ALLOCATION;

	camsrc_info("leave");
	return CAMERASRC_SUCCESS;
}


static int _camerasrc_set_shot_meta(camerasrc_handle_t *p, unsigned int grpid, int fcount,
                                    unsigned long long timestamp, struct camera2_shot_ext *shot_ext)
{
	if (p == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_NULL_POINTER;
	}
	memset(shot_ext, 0, sizeof(struct camera2_shot_ext));

	shot_ext->dis_bypass = 1;
	shot_ext->dnr_bypass = 1;
	shot_ext->fd_bypass = 1;

	shot_ext->shot.ctl.request.metadataMode = METADATA_MODE_FULL;
	shot_ext->shot.ctl.lens.aperture = 1.89999998;
	shot_ext->shot.ctl.lens.focalLength = 1.60000002;
	shot_ext->shot.ctl.lens.opticalStabilizationMode = OPTICAL_STABILIZATION_MODE_STILL;
	shot_ext->shot.ctl.sensor.frameDuration = 33333333;
	shot_ext->shot.ctl.flash.flashMode = CAM2_FLASH_MODE_OFF;
	shot_ext->shot.ctl.noise.mode = PROCESSING_MODE_OFF;
	/* Color */
	shot_ext->shot.ctl.color.mode = COLORCORRECTION_MODE_FAST;
	/* FIXME: Not working */
	//shot_ext->shot.ctl.color.transform [0] = 1;
	//shot_ext->shot.ctl.color.transform [8] = 1;
	shot_ext->shot.ctl.color.hue = 3;
	shot_ext->shot.ctl.color.saturation = 3;
	shot_ext->shot.ctl.color.brightness = 3;
	shot_ext->shot.ctl.color.contrast = 3;

	/* Scaler crop region ?*/
	/* aa */
	shot_ext->shot.ctl.aa.captureIntent = AA_CAPTURE_INTENT_CUSTOM;
	shot_ext->shot.ctl.aa.mode = AA_CONTROL_USE_SCENE_MODE;
	shot_ext->shot.ctl.aa.sceneMode = AA_SCENE_MODE_LLS;
	shot_ext->shot.ctl.aa.aeMode = AA_AEMODE_CENTER;
	shot_ext->shot.ctl.aa.aeExpCompensation = 5;
	/* FIXME : not working */
	//shot_ext->shot.ctl.aa.aeTargetFpsRange[0] = 8;
	//shot_ext->shot.ctl.aa.aeTargetFpsRange[1] = 30;
	shot_ext->shot.ctl.aa.aeAntibandingMode = AA_AE_ANTIBANDING_AUTO_50HZ;
	shot_ext->shot.ctl.aa.awbMode = AA_AWBMODE_WB_AUTO;
	/* FIXME : not working */
	//shot_ext->shot.ctl.aa.afRegions[4] = 1000;
	shot_ext->shot.ctl.aa.isoMode = AA_ISOMODE_AUTO;
	shot_ext->shot.ctl.aa.awbValue = 4;

	if (grpid == GRP_SENSOR)
		goto done;

	if (grpid == GRP_3AA) {
		shot_ext->node_group.leader.vid = FIMC_IS_VIDEO_3A1_NUM;
		shot_ext->node_group.leader.request = 1;
		shot_ext->node_group.leader.input.cropRegion[2] = p->format_3aa1.img_size.width;
		shot_ext->node_group.leader.input.cropRegion[3] = p->format_3aa1.img_size.height;
		shot_ext->node_group.leader.output.cropRegion[2] = p->format_3aa1.img_size.width;
		shot_ext->node_group.leader.output.cropRegion[3] = p->format_3aa1.img_size.height;

		shot_ext->node_group.capture[0].vid = FIMC_IS_VIDEO_3A1P_NUM;
		shot_ext->node_group.capture[0].request = 1;
		shot_ext->node_group.capture[0].input.cropRegion[2] = p->format_3aa1.img_size.width;
		shot_ext->node_group.capture[0].input.cropRegion[3] = p->format_3aa1.img_size.height;
		shot_ext->node_group.capture[0].output.cropRegion[2] = p->format_3aa1.img_size.width;
		shot_ext->node_group.capture[0].output.cropRegion[3] = p->format_3aa1.img_size.height;
	} else if (grpid == GRP_ISP) {
		shot_ext->node_group.leader.vid = FIMC_IS_VIDEO_ISP_NUM;
		shot_ext->node_group.leader.request = 1;
		shot_ext->node_group.leader.input.cropRegion[2] = p->format_isp.img_size.width;
		shot_ext->node_group.leader.input.cropRegion[3] = p->format_isp.img_size.height;
		shot_ext->node_group.leader.output.cropRegion[2] = p->format_isp.img_size.width;
		shot_ext->node_group.leader.output.cropRegion[3] = p->format_isp.img_size.height;

		/* SCC */
		shot_ext->node_group.capture[0].vid = FIMC_IS_VIDEO_SCC_NUM;
		shot_ext->node_group.capture[0].request = 1;
		shot_ext->node_group.capture[0].input.cropRegion[2] = p->format_scc.img_size.width;
		shot_ext->node_group.capture[0].input.cropRegion[3] = p->format_scc.img_size.height;
		shot_ext->node_group.capture[0].output.cropRegion[2] = p->format_scp.img_size.width;
		shot_ext->node_group.capture[0].output.cropRegion[3] = p->format_scp.img_size.height;

		/* SCP */
		shot_ext->node_group.capture[1].vid = FIMC_IS_VIDEO_SCP_NUM;
		shot_ext->node_group.capture[1].request = 1;
		shot_ext->node_group.capture[1].input.cropRegion[2] = p->format_scp.img_size.width;
		shot_ext->node_group.capture[1].input.cropRegion[3] = p->format_scp.img_size.height;
		shot_ext->node_group.capture[1].output.cropRegion[2] = p->format_scp.img_size.width;
		shot_ext->node_group.capture[1].output.cropRegion[3] = p->format_scp.img_size.height;

		/*
		 * This is first set of mysterious data :
		 * if not set firmware reposrt lack of user-defined
		 * dynamic meta.
		 * Fisrt netry needs to be non-zero - if it's not
		 * errors for SCC/SCP/DIS are being reported
		 * (no target address) so this needs to be somehow
		 * related to those (I think).
		 * Same rule (first item non-zero) applies to vendorspecific2
		 */
		shot_ext->shot.udm.internal.vendorSpecific1[0] = (uint32_t)~0U;
		shot_ext->shot.udm.internal.vendorSpecific1[1] = 0x000B0C00;
		if (fcount >= 0) {
			shot_ext->shot.udm.internal.vendorSpecific2[0] = (FIMC_IS_ISP_VS2_INIT +
			                                                  ((fcount - 1) % FIMC_IS_ISP_VS2_RANGE) * FIMC_IS_ISP_VS2_STEP);
		}

		shot_ext->shot.udm.bayer.width = 2560;
		shot_ext->shot.udm.bayer.height = 1440;
	}

	shot_ext->shot.dm.request.metadataMode = METADATA_MODE_NONE;
	if (fcount >= 0) {
		shot_ext->shot.dm.request.frameCount = fcount;
	}
	shot_ext->shot.dm.sensor.timeStamp = timestamp;

	camsrc_debug("GRP: %d FCOUNT: %d", grpid, fcount);

done:
	shot_ext->shot.magicNumber = METADATA_MAGIC_NUMBER;

	return CAMERASRC_SUCCESS;
}


/* END For Query functionalities */
