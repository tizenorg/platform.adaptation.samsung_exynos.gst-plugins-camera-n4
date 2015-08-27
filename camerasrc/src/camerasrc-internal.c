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

#include <math.h>
#include "camerasrc-internal.h"

#define _CAMERASRC_GET_CID(ctrl_id, dev_id)                     _camerasrc_ctrl_list[dev_id][ctrl_id][CAMERASRC_CTRL_CID_VALUE]

/* CUSTOM V4L2 CONTROL ID DEFINITIONS (END) */
static int _camerasrc_ctrl_list[CAMERASRC_DEV_ID_EXTENSION][CAMERASRC_CTRL_NUM][CAMERASRC_CTRL_PROPERTY_NUM] =
{       /* { SUPPORT, MAX_VALUE, MIN_VALUE, CID, CURRENT_VALUE } */
    {   /* Primary camera */
        {-1, 4, -4, V4L2_CID_EXPOSURE, 0},              /* Brightness */
        {-1, 3, -3, V4L2_CID_CONTRAST, 0},              /* Contrast */
        {-1, 30, 0, V4L2_CID_ZOOM_ABSOLUTE, 0},         /* Digital zoom */
        {0,  -1, -1, -1, -1},                           /* Optical zoom */
        {-1, V4L2_WHITE_BALANCE_CLOUDY, V4L2_WHITE_BALANCE_AUTO, V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE, V4L2_WHITE_BALANCE_AUTO},          /* White balance */
        {-1, V4L2_COLORFX_AQUA, V4L2_COLORFX_NONE, V4L2_CID_COLORFX, V4L2_COLORFX_NONE},                                /* Colortone */
        {-1, V4L2_SCENE_MODE_TEXT, V4L2_SCENE_MODE_NONE, V4L2_CID_SCENE_MODE, V4L2_SCENE_MODE_NONE},      /* program mode */
        {0,  -1, -1, -1, -1},                           /* Flip. V4L2_CID_VFLIP/HFLIP */
        {-1, 1, 0, V4L2_CID_IMAGE_STABILIZATION, 0},    /* ANTI_HANDSHAKE */
        {-1, 1, 0, V4L2_CID_WIDE_DYNAMIC_RANGE, 0},     /* WIDE_DYNAMIC_RANGE - AUTO CONTRAST */
        {-1, 3, -3, V4L2_CID_SATURATION, 0},            /* SATURATION */
        {-1, 3, -3, V4L2_CID_SHARPNESS, 0},             /* SHARPNESS */
        {-1, -1, -1, V4L2_CID_ISO_SENSITIVITY, -1},     /* ISO */
        {-1, V4L2_EXPOSURE_METERING_MATRIX, V4L2_EXPOSURE_METERING_AVERAGE, V4L2_CID_EXPOSURE_METERING, V4L2_EXPOSURE_METERING_AVERAGE},    /* PHOTOMETRY */
    },
    {   /* Secondary camera */
        {0,  -1, -1, V4L2_CID_EXPOSURE, -1},            /* Brightness */
        {0,  -1, -1, V4L2_CID_CONTRAST, -1},            /* Contrast */
        {0,  -1, -1, V4L2_CID_ZOOM_ABSOLUTE, -1},       /* Digital zoom */
        {0,  -1, -1, -1, -1},                           /* Optical zoom */
        {0,  -1, -1, V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE, -1},/* White balance */
        {0,  -1, -1, V4L2_CID_COLORFX, -1},             /* Colortone */
        {0,  -1, -1, V4L2_CID_SCENE_MODE, -1},          /* program mode */
        {0,  -1, -1, -1, -1},                           /* Flip */
        {0,  -1, -1, V4L2_CID_WIDE_DYNAMIC_RANGE, -1},  /* ANTI_HANDSHAKE */
        {0,  -1, -1, V4L2_CID_WIDE_DYNAMIC_RANGE, -1},  /* WIDE_DYNAMIC_RANGE */
        {0,  -1, -1, V4L2_CID_SATURATION, -1},          /* SATURATION */
        {0,  -1, -1, V4L2_CID_SHARPNESS, -1},           /* SHARPNESS */
        {0,  -1, -1, V4L2_CID_ISO_SENSITIVITY, -1},     /* ISO */
        {0,  -1, -1, V4L2_CID_EXPOSURE_METERING, -1},   /* PHOTOMETRY */
    },
};


/*#define USE_IOCTL_DEBUG*/
#if defined (USE_IOCTL_DEBUG)
static char* get_request_name(int request, char* res_str) {
	switch (request) {
	case VIDIOC_QBUF:
		sprintf(res_str, "[VIDIOC_QBUF]");
		break;
	case VIDIOC_DQBUF:
		sprintf(res_str, "[VIDIOC_DQBUF]");
		break;
	case VIDIOC_S_INPUT:
		sprintf(res_str, "[VIDIOC_S_INPUT]");
		break;
	case VIDIOC_G_INPUT:
		sprintf(res_str, "[VIDIOC_G_INPUT]");
		break;
	case VIDIOC_S_PARM:
		sprintf(res_str, "[VIDIOC_S_PARM]");
		break;
	case VIDIOC_G_PARM:
		sprintf(res_str, "[VIDIOC_G_PARM]");
		break;
	case VIDIOC_S_FMT:
		sprintf(res_str, "[VIDIOC_S_FMT]");
		break;
	case VIDIOC_G_FMT:
		sprintf(res_str, "[VIDIOC_G_FMT]");
		break;
	case VIDIOC_REQBUFS:
		sprintf(res_str, "[VIDIOC_REQBUFS]");
		break;
	case VIDIOC_QUERYBUF:
		sprintf(res_str, "[VIDIOC_QUERYBUF]");
		break;
	case VIDIOC_STREAMON:
		sprintf(res_str, "[VIDIOC_STREAMON]");
		break;
	case VIDIOC_STREAMOFF:
		sprintf(res_str, "[VIDIOC_STREAMOFF]");
		break;
	case VIDIOC_S_CTRL:
		sprintf(res_str, "[VIDIOC_S_CTRL] ");
		break;
	case VIDIOC_G_CTRL:
		sprintf(res_str, "[VIDIOC_G_CTRL]");
		break;
	case VIDIOC_ENUMINPUT:
		sprintf(res_str, "[VIDIOC_ENUMINPUT]");
		break;
	case VIDIOC_S_JPEGCOMP:
		sprintf(res_str, "[VIDIOC_S_JPEGCOMP]");
		break;
	case VIDIOC_G_JPEGCOMP:
		sprintf(res_str, "[VIDIOC_G_JPEGCOMP]");
		break;
	/* Extension */
	case VIDIOC_S_STROBE:
		sprintf(res_str, "[VIDIOC_S_STROBE]");
		break;
	case VIDIOC_G_STROBE:
		sprintf(res_str, "[VIDIOC_G_STROBE]");
		break;
	case VIDIOC_S_RECOGNITION:
		sprintf(res_str, "[VIDIOC_S_RECOGNITION]");
		break;
	case VIDIOC_G_RECOGNITION:
		sprintf(res_str, "[VIDIOC_G_RECOGNITION]");
		break;
	case VIDIOC_G_EXIF:
		sprintf(res_str, "[VIDIOC_G_EXIF]");
		break;
	default:
		sprintf(res_str, "[UNKNOWN IOCTL(%x)]", request);
		break;
	}

	return 0;
}

#define PRINT_IOCTL_INFO(request, arg) {\
	char res_str[255];\
	get_request_name(request, res_str);\
	camsrc_info("[request : %s, argument address : %x]", res_str, arg);\
}
#else

#define PRINT_IOCTL_INFO(request, arg)

#endif


#define LOCK(x) {\
	if(0 != pthread_mutex_lock(&(x->mutex))) {\
		camsrc_error("Mutex lock error");\
		camsrc_assert(0);\
	}\
}

#define UNLOCK(x) {\
	if(0 != pthread_mutex_unlock(&(x->mutex))) {\
		camsrc_error("Mutex unlock error");\
		camsrc_assert(0);\
	}\
}

#define AF_MUT_LOCK(p) {\
	if(0 != pthread_mutex_lock(&(p->af_mutex))) {\
		camsrc_error("AF Mutex locking error");\
		camsrc_assert(0);\
	}\
}

#define AF_MUT_UNLOCK(p) {\
	if(0 != pthread_mutex_unlock(&(p->af_mutex))) {\
		camsrc_error("AF Mutex unlocking error");\
		camsrc_assert(0);\
	}\
}


static int _camerasrc_ioctl(camerasrc_handle_t *handle, int fd, int request, void *arg)
{
	int err;
	int nAgain = 10;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	if (handle == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	if (fd < 0) {
		camsrc_error("invalid fd %d", fd);
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	LOCK(handle);

	PRINT_IOCTL_INFO(request, arg);

again:
	do {
		err = ioctl (fd, request, arg);
	} while (-1 == err && EINTR == errno);

	if (err != 0) {
		handle->errnum = errno;
		err = errno;
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("ioctl[%x] err : %s", request, err_msg);
		if (err == EEXIST) {
			camsrc_info("EEXIST occured, but can go.");
			err = 0;
		} else if (err == ENOENT) {
			camsrc_info("ENOENT occured, but can go.");
			err = 0;
#if defined (ENABLE_Q_ERROR)
#warning "ENABLE_Q_ERROR enabled"
		} else if (request == VIDIOC_DQBUF) {
			goto DQ_ERROR;
		} else if (request == VIDIOC_QBUF) {
			goto ENQ_ERROR;
#endif
		} else if (err == EINVAL) {
			camsrc_error("EINVAL occured, Shutdown");
			UNLOCK(handle);
			return CAMERASRC_ERR_INVALID_PARAMETER;
		} else if (err == EBUSY) {
			camsrc_error("EBUSY occured, Shutdown");
			UNLOCK(handle);
			return CAMERASRC_ERR_PRIVILEGE;
		} else if (err == ENODEV) {
			camsrc_error("ENODEV occured, Shutdown");
			//UNLOCK(handle);
			//return CAMERASRC_ERR_UNAVAILABLE_DEVICE;
		} else if (err == EAGAIN && nAgain--) {
			goto again;
		} else {
			/* Why does this return SUCCESS? */
			camsrc_error("Unhandled exception occured on IOCTL");
		}
	}

	UNLOCK(handle);

	return CAMERASRC_SUCCESS;

#if defined (ENABLE_Q_ERROR)
DQ_ERROR:
	camsrc_error("DQ Frame error occured");
	printf("DQ Frame error occured");
	UNLOCK(handle);
	return CAMERASRC_ERR_INTERNAL;

ENQ_ERROR:
	camsrc_error("Q Frame error occured");
	printf("Q Frame error occured");
	UNLOCK(handle);
	return CAMERASRC_ERR_INTERNAL;
#endif
}


static int _camerasrc_ioctl_with_err(camerasrc_handle_t *handle, int request, void *arg, int *error)
{
	int fd = -1;
	int err;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	if (handle == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	if (handle->fd_sensor < 0) {
		camsrc_error("invalid fd %d", handle->fd_sensor);
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	fd = handle->fd_sensor;

	LOCK(handle);

	*error = 0;

	PRINT_IOCTL_INFO(request, arg);

	do {
		err = ioctl (fd, request, arg);
	} while (-1 == err && EINTR == errno);

	if (err != 0) {
		handle->errnum = errno;
		*error = errno;
		strerror_r(*error, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("ioctl[%x] err : %s", request, err_msg);
		UNLOCK(handle);
		return CAMERASRC_ERR_IO_CONTROL;
	}

	UNLOCK(handle);

	return CAMERASRC_SUCCESS;
}


static int _camerasrc_ioctl_once(camerasrc_handle_t *handle, int request, void *arg)
{
	int fd = -1;
	int err = -1;
	int nAgain = 10;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	if (handle == NULL) {
		camsrc_error("NULL handle");
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	if (handle->fd_sensor < 0) {
		camsrc_error("invalid fd %d", handle->fd_sensor);
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	fd = handle->fd_sensor;

	LOCK(handle);

	PRINT_IOCTL_INFO(request, arg);

again:
	err =  ioctl (fd, request, arg);

	if (err != 0) {
		handle->errnum = errno;
		err = errno;
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_error("ioctl[%x] err : %s", request, err_msg);
		if (err == EEXIST) {
			camsrc_info("EEXIST occured, but can go.");
			err = 0;
		} else if (err == ENOENT) {
			camsrc_info("ENOENT occured, but can go.");
			err = 0;
#if defined (ENABLE_Q_ERROR)
#warning "ENABLE_Q_ERROR enabled"
		} else if (request == VIDIOC_DQBUF) {
			goto DQ_ERROR;
		} else if (request == VIDIOC_QBUF) {
			goto ENQ_ERROR;
#endif
		} else if (err == EINVAL) {
			camsrc_error("EINVAL occured, Shutdown");
			UNLOCK(handle);
			return CAMERASRC_ERR_INVALID_PARAMETER;
		} else if (err == EAGAIN && nAgain--) {
			goto again;
		} else {
			camsrc_error("Unhandled exception occured on IOCTL");
		}
	}

	UNLOCK(handle);

	return CAMERASRC_SUCCESS;

#if defined (ENABLE_Q_ERROR)
DQ_ERROR:
	camsrc_error("DQ Frame error occured");
	printf("DQ Frame error occured");
	UNLOCK(handle);
	return CAMERASRC_ERR_INTERNAL;

ENQ_ERROR:
	camsrc_error("Q Frame error occured");
	printf("Q Frame error occured");
	UNLOCK(handle);
	return CAMERASRC_ERR_INTERNAL;
#endif
}


static int _camerasrc_init_autofocusing_mode(camerasrc_handle_t *handle)
{
	int ctrl_id = V4L2_CID_AUTO_FOCUS_RANGE;
	int mode;

	camsrc_info("enter");

	if (!handle) {
		camsrc_warning("handle is NULL");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	if (handle->cur_dev_id == CAMERASRC_DEV_ID_SECONDARY) {
		camsrc_info("secondary camera does not support AF");
		return CAMERASRC_SUCCESS;
	}

	if (handle->af_status == CAMERASRC_AUTO_FOCUS_STATUS_ONGOING) {
		camsrc_info("Dev BUSY. Init failed.");
		return CAMERASRC_ERR_INVALID_STATE;
	}

	switch (handle->cur_af_mode) {
	case CAMERASRC_AF_MODE_AUTO:
		if (handle->cur_af_range == CAMERASRC_AF_RANGE_MACRO) {
			mode = V4L2_AUTO_FOCUS_RANGE_MACRO;
			camsrc_info("ON AUTOFOCUSING...BY AUTO_MACRO");
		} else {
			mode = V4L2_AUTO_FOCUS_RANGE_NORMAL;
			camsrc_info("ON AUTOFOCUSING...BY AUTO_NORMAL");
		}
		break;
	case CAMERASRC_AF_MODE_CONTINUOUS:
		if (handle->cur_af_range == CAMERASRC_AF_RANGE_MACRO) {
			mode = V4L2_AUTO_FOCUS_RANGE_MACRO;
			camsrc_info("ON AUTOFOCUSING...BY CONTINUOUS_MACRO");
		} else {
			mode = V4L2_AUTO_FOCUS_RANGE_NORMAL;
			camsrc_info("ON AUTOFOCUSING...BY CONTINUOUS_NORMAL");
		}
		break;
	case CAMERASRC_AF_MODE_TOUCH_AUTO:
		if (handle->cur_af_range == CAMERASRC_AF_RANGE_MACRO) {
			mode = V4L2_AUTO_FOCUS_RANGE_MACRO;
			camsrc_info("ON AUTOFOCUSING...BY TOUCH_AUTO_MACRO");
		} else {
			mode = V4L2_AUTO_FOCUS_RANGE_NORMAL;
			camsrc_info("ON AUTOFOCUSING...BY TOUCH_AUTO_NORMAL");
		}
		break;
	case CAMERASRC_AF_MODE_MANUAL:
		camsrc_warning("MANUAL focusing is not supported.");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	case CAMERASRC_AF_MODE_PAN:
		camsrc_warning("PAN MODE focusing is not supported.");
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	default:
		camsrc_warning("Unsupported AF mode[%d]", handle->cur_af_mode );
		return CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
	}

	GST_WARNING("set focus mode %d", mode);

	SET_CTRL_VAL(ctrl_id, mode);

	return CAMERASRC_SUCCESS;
}


static void* _camerasrc_run_autofocusing(camerasrc_handle_t *handle)
{
	int err = 0;

	camsrc_info("enter");

	while (1) {
		AF_MUT_LOCK(handle);
		switch (handle->af_cmd) {
		case CAMERASRC_AUTO_FOCUS_CMD_START:
		{
			if (CAMERASRC_STATE(handle) > CAMERASRC_STATE_READY &&
			    handle->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY) {
				if (handle->cur_af_mode == CAMERASRC_AF_MODE_CONTINUOUS) {
					camsrc_info("AF CMD: CONTINUOUS AF START");
					SET_CTRL_VAL_ERR(V4L2_CID_FOCUS_AUTO, 1, err);
				} else {
					camsrc_info("AF CMD: NORMAL AF START");
					SET_CTRL_VAL_ERR(V4L2_CID_FOCUS_AUTO, 0, err);
				}
				CAMERASRC_SET_STATE(handle, CAMERASRC_STATE_PREVIEW);
			} else {
				camsrc_warning("Invalid state %d or Not PRIMARY camera %d. skip AF:Start command...",
				               CAMERASRC_STATE(handle), handle->cur_dev_id);
			}

			handle->af_status = CAMERASRC_AUTO_FOCUS_STATUS_RELEASED;
			handle->af_cmd = CAMERASRC_AUTO_FOCUS_CMD_NULL;
			break;
		}
		case CAMERASRC_AUTO_FOCUS_CMD_STOP:
		{
			if (handle->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY) {
				camsrc_info("AF CMD: CONTINUOUS AF STOP");

				SET_CTRL_VAL_ERR(V4L2_CID_AUTO_FOCUS_STOP, 1, err);

				camsrc_info("Stopping AF done. err = %d", err);
			} else {
				camsrc_warning("Not PRIMARY camera. skip AF:Stop command...");
			}

			handle->af_status = CAMERASRC_AUTO_FOCUS_STATUS_RELEASED;
			handle->af_cmd = CAMERASRC_AUTO_FOCUS_CMD_NULL;
			AF_MUT_UNLOCK(handle);
			continue;
		}
		case CAMERASRC_AUTO_FOCUS_CMD_KILL:
		{
			camsrc_info("AF CMD:KILL");

			handle->af_status = CAMERASRC_AUTO_FOCUS_STATUS_RELEASED;
			handle->af_cmd = CAMERASRC_AUTO_FOCUS_CMD_NULL;

			AF_MUT_UNLOCK(handle);
			goto OUT_OF_LOOP;
		}
		case CAMERASRC_AUTO_FOCUS_CMD_NULL:
		default:
		{
			char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};
			camsrc_log("AF CMD:NULL....");
			err = pthread_cond_wait(&handle->af_wait_cond, &handle->af_mutex);
			if (err) {
				strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
				camsrc_error("AF CMD pthread_cond_wait - err:(%s)", err_msg);
			}

			AF_MUT_UNLOCK(handle);

			continue;
		}
		}

		AF_MUT_UNLOCK(handle);
		usleep(CAMERASRC_AF_INTERVAL);
	}

OUT_OF_LOOP:
	camsrc_info("AF thread is finished.");
	return NULL;
}

/*
   Because of camerasrc_start_autofocusing() in camsrc-cam.c, _camerasrc_run_autofocusing is needed.
   So I use below 'abled' function instead of 'disabled' one, even though the way uses one more thread with quite complex mechanism.
 */
static int _camerasrc_start_autofocusing(camerasrc_handle_t *handle)
{
	int err;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};
	camsrc_info("enter");

	AF_MUT_LOCK(handle);

	handle->af_cmd = CAMERASRC_AUTO_FOCUS_CMD_START;
	err = pthread_cond_signal(&handle->af_wait_cond);
	if (err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF wait cond err(%s)", err_msg);
	}

	AF_MUT_UNLOCK(handle);

	return CAMERASRC_SUCCESS;
}

static int _camerasrc_stop_autofocusing(camerasrc_handle_t *handle)
{
	int err;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};

	camsrc_info("enter");

	AF_MUT_LOCK(handle);

	handle->af_cmd = CAMERASRC_AUTO_FOCUS_CMD_STOP;
	err = pthread_cond_signal(&handle->af_wait_cond);
	if (err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF wait cond err(%s)", err_msg);
	}

	AF_MUT_UNLOCK(handle);

	return CAMERASRC_SUCCESS;
}

static int _camerasrc_destroy_autofocusing(camerasrc_handle_t *handle)
{
	int err;
	char err_msg[CAMERASRC_ERRMSG_MAX_LEN] = {'\0',};
	camsrc_info("enter");

	AF_MUT_LOCK(handle);

	handle->af_cmd = CAMERASRC_AUTO_FOCUS_CMD_KILL;
	err = pthread_cond_signal(&handle->af_wait_cond);
	if (err) {
		strerror_r(err, err_msg, CAMERASRC_ERRMSG_MAX_LEN);
		camsrc_info("AF wait cond err(%s)", err_msg);
	}

	AF_MUT_UNLOCK(handle);

	return CAMERASRC_SUCCESS;
}


static int _camerasrc_get_autofocusing_result(camerasrc_handle_t *handle)
{
	int err;
	int af_result = V4L2_AUTO_FOCUS_STATUS_BUSY;
	/* This is used for skipping same result.
	   Same AF-result is comming many times when focused or failed. */
	static int af_state = 0;

	camsrc_log("enter");

	GET_CTRL_VAL_ERR(V4L2_CID_FOCUS_AUTO, af_result, err);

	camsrc_log("AF RESULT : 0x%x", af_result);

	switch (af_result) {
	case V4L2_AUTO_FOCUS_STATUS_BUSY:
		if (af_state != CAMERASRC_AUTO_FOCUS_RESULT_FUCUSING) {
			af_state = CAMERASRC_AUTO_FOCUS_RESULT_FUCUSING;
			handle->af_cb(handle, CAMERASRC_AUTO_FOCUS_RESULT_FUCUSING, handle->af_usr_data);
			camsrc_info("AF Progressing...");
		}
		break;
	case V4L2_AUTO_FOCUS_STATUS_REACHED:
		if (af_state != CAMERASRC_AUTO_FOCUS_RESULT_FOCUSED) {
			af_state = CAMERASRC_AUTO_FOCUS_RESULT_FOCUSED;
			handle->af_cb(handle, CAMERASRC_AUTO_FOCUS_RESULT_FOCUSED, handle->af_usr_data);
			camsrc_info("AF Success");
		}
		break;
	case V4L2_AUTO_FOCUS_STATUS_FAILED:
		if (af_state != CAMERASRC_AUTO_FOCUS_RESULT_FAILED) {
			af_state = CAMERASRC_AUTO_FOCUS_RESULT_FAILED;
			handle->af_cb(handle, CAMERASRC_AUTO_FOCUS_RESULT_FAILED, handle->af_usr_data);
			camsrc_info("AF Fail");
		}
		break;
	default:
		camsrc_warning("unknown AF result : 0x%x", af_result);
		break;
	}


	return CAMERASRC_SUCCESS;
}


static int _camerasrc_get_frame_data(camerasrc_handle_t *handle, camerasrc_frame_data_t *data)
{
	/*camsrc_info("enter");*/
	if (handle == NULL || data == NULL) {
		camsrc_error("NULL pointer");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	if (handle->buffer_scp) {
		memcpy(&data->buffer, &handle->buffer_scp[data->index], sizeof(camerasrc_buffer_t));
	} else {
		camsrc_error("buffer_scp is NULL");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	return CAMERASRC_SUCCESS;
}


static void _dump_exif_info(camerasrc_exif_t *exif_struct)
{
	camsrc_info("== Dynamic value ==");
	camsrc_info("unsigned int exposure_time_numerator = %d", exif_struct->exposure_time_numerator);
	camsrc_info("unsigned int exposure_time_denominator = %d", exif_struct->exposure_time_denominator);
	camsrc_info("int shutter_speed_numerator = %d", exif_struct->shutter_speed_numerator);
	camsrc_info("int shutter_speed_denominator = %d", exif_struct->shutter_speed_denominator);
	camsrc_info("int brigtness_numerator = %d", exif_struct->brigtness_numerator);
	camsrc_info("int brightness_denominator = %d", exif_struct->brightness_denominator);
	camsrc_info("unsigned short int iso = %d", exif_struct->iso);
	camsrc_info("unsigned short int flash = %d", exif_struct->flash);
	camsrc_info("int metering_mode = %d", exif_struct->metering_mode);
	camsrc_info("int exif_image_width = %d", exif_struct->exif_image_width);
	camsrc_info("int exif_image_height = %d", exif_struct->exif_image_height);

	camsrc_info("== Fixed value ==");
	camsrc_info("int software_used = %d", exif_struct->software_used);
	camsrc_info("int focal_len_numerator = %d", exif_struct->focal_len_numerator);
	camsrc_info("int focal_len_denominator = %d", exif_struct->focal_len_denominator);
	camsrc_info("int aperture_f_num_numerator = %d", exif_struct->aperture_f_num_numerator);
	camsrc_info("int aperture_f_num_denominator = %d", exif_struct->aperture_f_num_denominator);
	camsrc_info("int aperture_in_APEX = %d", exif_struct->aperture_in_APEX);
	camsrc_info("int max_lens_aperture_in_APEX = %d", exif_struct->max_lens_aperture_in_APEX);
	camsrc_info("int exposure_bias_in_APEX = %d", exif_struct->exposure_bias_in_APEX);
	camsrc_info("int component_configuration = %x", exif_struct->component_configuration);
	camsrc_info("int colorspace = %d", exif_struct->colorspace);

	return;
}


static int _camerasrc_get_exif_info(camerasrc_handle_t *handle, camerasrc_exif_t *exif_struct)
{
	int photometry_mode = V4L2_EXPOSURE_METERING_AVERAGE;

	if (exif_struct == NULL) {
		return CAMERASRC_ERR_INVALID_PARAMETER;
	}

	/**
	 * Dynamic value
	**/
	/* exposure time */
	exif_struct->exposure_time_numerator = handle->current_exif.exposure_time.numerator;
	exif_struct->exposure_time_denominator = handle->current_exif.exposure_time.denominator;

	/* shutter speed */
	exif_struct->shutter_speed_numerator = handle->current_exif.shutter_speed.numerator;
	exif_struct->shutter_speed_denominator = handle->current_exif.shutter_speed.denominator;

	/* brightness */
	exif_struct->brigtness_numerator = handle->current_exif.brightness.numerator;
	exif_struct->brightness_denominator = handle->current_exif.brightness.denominator;

	/* iso */
	exif_struct->iso = handle->current_exif.iso_speed_rating;
	ISO_APPROXIMATE_VALUE(exif_struct->iso, exif_struct->iso);

	/* flash */
	exif_struct->flash = handle->current_exif.flash;

	/* image size */
	exif_struct->exif_image_width = handle->format_scp.img_size.width;
	exif_struct->exif_image_height = handle->format_scp.img_size.height;

	/* Get the value using CID */
	/* Not implemented yet
	   GET_CTRL_VAL(V4L2_CID_FW_VERSION, exif_struct->software_used);
	 */

	/* metering */
	if (handle->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY) {
		camsrc_log("                                V4L2_CID_EXPOSURE_METERING");
		GET_CTRL_VAL(V4L2_CID_EXPOSURE_METERING, photometry_mode);
		PHOTOMETRY_MODE_TO_METERING_MODE(photometry_mode, exif_struct->metering_mode);
	} else {
		exif_struct->metering_mode = V4L2_EXPOSURE_METERING_AVERAGE;
	}

	/**
	 * Fixed value
	**/
	if (handle->cur_dev_id == CAMERASRC_DEV_ID_PRIMARY) {
		/* focal length */
		exif_struct->focal_len_numerator = EXIF_DEFAULT_FOCAL_LENGTH_NUMERATOR_REAR;
		exif_struct->focal_len_denominator = EXIF_DEFAULT_FOCAL_LENGTH_DENOMINATOR_REAR;

		/* f number */
		exif_struct->aperture_f_num_numerator = EXIF_DEFAULT_FNUMBER_NUMERATOR_REAR;
		exif_struct->aperture_f_num_denominator = EXIF_DEFAULT_FNUMBER_DENOMINATOR_REAR;
	} else {
		/* focal length */
		exif_struct->focal_len_numerator = EXIF_DEFAULT_FOCAL_LENGTH_NUMERATOR_FRONT;
		exif_struct->focal_len_denominator = EXIF_DEFAULT_FOCAL_LENGTH_DENOMINATOR_FRONT;

		/* f number */
		exif_struct->aperture_f_num_numerator = EXIF_DEFAULT_FNUMBER_NUMERATOR_FRONT;
		exif_struct->aperture_f_num_denominator = EXIF_DEFAULT_FNUMBER_DENOMINATOR_FRONT;
	}

	/* aperture value */
	exif_struct->aperture_in_APEX \
		= CAMERASRC_EXIF_APERTURE_VALUE_IN_APEX(exif_struct->aperture_f_num_numerator, exif_struct->aperture_f_num_denominator);
	exif_struct->max_lens_aperture_in_APEX = exif_struct->aperture_in_APEX;

	/* exposure bias */
	exif_struct->exposure_bias_in_APEX = exif_struct->aperture_in_APEX \
		+ CAMERASRC_EXIF_SHUTTERSPEED_VALUE_IN_APEX(exif_struct->exposure_time_numerator, exif_struct->exposure_time_denominator);

	/* component configuration */
	exif_struct->component_configuration = EXIF_DEFAULT_COMPONENT_CONFIGRATION;

	/* color space */
	exif_struct->colorspace = EXIF_DEFAULT_COLOR_SPACE;

	_dump_exif_info(exif_struct);

	return CAMERASRC_SUCCESS;
}


static int _camerasrc_set_cmd(camerasrc_handle_t *handle, _camsrc_cmd_t cmd, void *value)
{
	int err = CAMERASRC_ERR_UNKNOWN;

	switch (cmd) {
	case _CAMERASRC_CMD_STROBE_MODE:
	{
		camerasrc_strobe_mode_t *mode = (camerasrc_strobe_mode_t*)value;

		camsrc_info("[_CAMERASRC_CMD_STROBE_MODE] cmd set value %d", *mode );

		//SET_CTRL_VAL(V4L2_CID_HW_FLASH_MODE, *mode);
	}
		break;
	case _CAMERASRC_CMD_SHUTTER_SPEED:
		/* WRITEME */
		camsrc_info("[_CAMERASRC_CMD_SHUTTER_SPEED] cmd set");
		break;
	case _CAMERASRC_CMD_EXPOSURE_VALUE:
		camsrc_info("[_CAMERASRC_CMD_EXPOSURE_VALUE] cmd set");
		/* WRITEME */
		break;
	case _CAMERASRC_CMD_CTRL:
		camsrc_info("[_CAMERASRC_CMD_CTRL] cmd set");
		SET_CTRL_VAL(_CAMERASRC_GET_CID(((_camerasrc_ctrl_t *) value)->cid, handle->cur_dev_id), ((_camerasrc_ctrl_t *) value)->value);
		break;
	case _CAMERASRC_CMD_AF_CONTROL:
		/* WRITEME */
		camsrc_info("[_CAMERASRC_CMD_AF_CONTROL] cmd set(value=%d)", (int)value);

		/* FIXME : Please fix whole AF implementation!!! */
		switch ((int)value) {
		case _CAMERASRC_AF_START:
			_camerasrc_start_autofocusing(handle);
			break;
		case _CAMERASRC_AF_STOP:
			_camerasrc_stop_autofocusing(handle);
			break;
		case _CAMERASRC_AF_DESTROY:
			_camerasrc_destroy_autofocusing(handle);
			break;
		case _CAMERASRC_AF_RESULT:
			camsrc_info("Not support : SET AF result");
			break;
		case _CAMERASRC_AF_RELEASE:
		case _CAMERASRC_AF_INIT:
		default:
			_camerasrc_init_autofocusing_mode(handle);
			break;
		}
		break;
	case _CAMERASRC_CMD_AF_AREA:
	{
		camerasrc_rect_t *rect = (camerasrc_rect_t *)value;

		camsrc_info("[_CAMERASRC_CMD_AF_AREA] cmd set (%d,%d,%dx%d)",
		                       rect->x, rect->y, rect->width, rect->height);
#if 0
		SET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_LEFT, rect->x);
		SET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_TOP, rect->y);
		SET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_WIDTH, 0);/*rect->width); Not supported */
		SET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_HEIGHT, 0);/*rect->height); Not supported */
#else
		camsrc_warning("AF area set is not supported");
#endif
		break;
	}
	case _CAMERASRC_CMD_FRAME_DATA:
		/* WRITEME */
		camsrc_info("[_CAMERASRC_CMD_FRAME_DATA] cmd set");
		break;
	case _CAMERASRC_CMD_EXIF_INFO:
		/* WRITEME */
		camsrc_info("[_CAMERASRC_CMD_EXIF_INFO] cmd set");
		break;
	case _CAMERASRC_CMD_ROTATION:
	{
		int *rotate = (int *)value;
		camsrc_info("[_CAMERASRC_CMD_ROTATION] cmd set : %d", *rotate);
		SET_CTRL_VAL(V4L2_CID_ROTATE, (int)*rotate);
	}
		break;
	case _CAMERASRC_CMD_VFLIP:
	{
		int *vflip = (int *)value;
		camsrc_info("[_CAMERASRC_CMD_VFLIP] cmd set : %d", *vflip);
		SET_CTRL_VAL(V4L2_CID_VFLIP, (int)*vflip);
	}
		break;
	case _CAMERASRC_CMD_HFLIP:
	{
		int *hflip = (int *)value;
		camsrc_info("[_CAMERASRC_CMD_HFLIP] cmd set : %d", *hflip);
		SET_CTRL_VAL(V4L2_CID_HFLIP, (int)*hflip);
	}
		break;
	default:
		camsrc_error("[_CAMERASRC_CMD_UNKNOWN] cmd set");
		err = CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
		goto ERROR;
	}

	return CAMERASRC_SUCCESS;

ERROR:
	camsrc_error("cmd execution error occured");

	return err;
}

static int _camerasrc_get_cmd(camerasrc_handle_t *handle, _camsrc_cmd_t cmd, void *value)
{
	int err = CAMERASRC_ERR_UNKNOWN;

	if (!value) {
		camsrc_error("value is NULL");
		return CAMERASRC_ERR_NULL_POINTER;
	}

	switch (cmd) {
	case _CAMERASRC_CMD_STROBE_MODE:
	{
		camerasrc_strobe_mode_t mode = 0;

		//GET_CTRL_VAL(V4L2_CID_HW_FLASH_MODE, mode);
		*((camerasrc_strobe_mode_t*)value) = mode;

		camsrc_info("[_CAMERASRC_CMD_STROBE_MODE] cmd get - %d", mode);
	}
		break;
	case _CAMERASRC_CMD_SHUTTER_SPEED:
		/* WRITEME */
		camsrc_info("[_CAMERASRC_CMD_SHUTTER_SPEED] cmd get");
		break;
	case _CAMERASRC_CMD_EXPOSURE_VALUE:
		/* WRITEME */
		camsrc_info("[_CAMERASRC_CMD_EXPOSURE_VALUE] cmd get");
		break;
	case _CAMERASRC_CMD_CTRL:
		camsrc_info("[_CAMERASRC_CMD_CTRL] cmd get");
		GET_CTRL_VAL(_CAMERASRC_GET_CID(((_camerasrc_ctrl_t *) value)->cid, handle->cur_dev_id), ((_camerasrc_ctrl_t *) value)->value);
		break;
	case _CAMERASRC_CMD_AF_CONTROL:
		camsrc_log("[_CAMERASRC_CMD_AF_CONTROL] cmd get");
		switch ((int)value) {
		case _CAMERASRC_AF_RESULT:
			_camerasrc_get_autofocusing_result(handle);
			break;
		default:
			camsrc_info("Not support : GET AF CONTROL [%d]", (int)value);
			break;
		}
		break;
	case _CAMERASRC_CMD_AF_AREA:
	{
		camerasrc_rect_t* rect = (camerasrc_rect_t*)value;
#if 0
		GET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_LEFT, rect->x);
		GET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_TOP, rect->y);
		GET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_WIDTH, rect->width);
		GET_CTRL_VAL(V4L2_CID_FOCUS_AUTO_RECTANGLE_HEIGHT, rect->height);
#else
		camsrc_warning("AF area is not supported");
#endif
		camsrc_info("[_CAMERASRC_CMD_AF_AREA] cmd get (%d,%d,%dx%d)",
		                       rect->x, rect->y, rect->width, rect->height);
		break;
	}
	case _CAMERASRC_CMD_FRAME_DATA:
		/* WRITEME */
		/*camsrc_info("[_CAMERASRC_CMD_FRAME_DATA] cmd get");*/
		err = _camerasrc_get_frame_data(handle, (camerasrc_frame_data_t*)value);
		if (err != CAMERASRC_SUCCESS) {
			goto ERROR;
		}
		break;
	case _CAMERASRC_CMD_EXIF_INFO:
		camsrc_info("[_CAMERASRC_CMD_EXIF_INFO] cmd get");
		err = _camerasrc_get_exif_info (handle, (camerasrc_exif_t*)value);
		if (err != CAMERASRC_SUCCESS) {
			goto ERROR;
		}
		break;
	case _CAMERASRC_CMD_ROTATION:
		camsrc_info("[_CAMERASRC_CMD_ROTATION] cmd get");
		GET_CTRL_VAL(V4L2_CID_ROTATE, *(int*)value);
		break;
	case _CAMERASRC_CMD_VFLIP:
		camsrc_info("[_CAMERASRC_CMD_VFLIP] cmd get");
		GET_CTRL_VAL(V4L2_CID_VFLIP, *(int*)value);
		break;
	case _CAMERASRC_CMD_HFLIP:
		camsrc_info("[_CAMERASRC_CMD_HFLIP] cmd get");
		GET_CTRL_VAL(V4L2_CID_HFLIP, *(int*)value);
		break;
	default:
		camsrc_error("[_CAMERASRC_CMD_UNKNOWN] cmd get");
		err = CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
		goto ERROR;
	}

	return CAMERASRC_SUCCESS;

ERROR:
	camsrc_error("cmd execution error occured");

	return err;
}

static const CAMERASRC_DEV_DEPENDENT_MISC_FUNC dev_misc_functions = {
	._ioctl            = _camerasrc_ioctl,
	._ioctl_once       = _camerasrc_ioctl_once,
	._run_autofocusing = _camerasrc_run_autofocusing,
	._set_cmd          = _camerasrc_set_cmd,
	._get_cmd          = _camerasrc_get_cmd,
};

const CAMERASRC_DEV_DEPENDENT_MISC_FUNC	*dev_misc_func = &dev_misc_functions;
