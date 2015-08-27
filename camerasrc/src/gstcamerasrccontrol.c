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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gst/gst.h>
#include "gstcamerasrccontrol.h"
#include "camerasrc-common.h"

#define gst_camerasrc_debug(fmt, args...)       GST_INFO(fmt, ##args)

#define CAMERA_CONTROL_AF_STOP_TOTALTIME        2000000
#define CAMERA_CONTROL_AF_STOP_INTERVAL         20000

/**
 * Enumeration of Wide Dynamic Range
 */
enum {
	CAMERA_CONTROL_WDR_OFF = 1,
	CAMERA_CONTROL_WDR_ON,
	CAMERA_CONTROL_WDR_AUTO,
};

G_DEFINE_TYPE(GstCamerasrcControlChannel,
                gst_camerasrc_control_channel,
                GST_TYPE_CAMERA_CONTROL_CHANNEL);

static void gst_camerasrc_control_channel_class_init(GstCamerasrcControlChannelClass *klass)
{
	gst_camerasrc_debug("class init");
}

static void gst_camerasrc_control_channel_init(GstCamerasrcControlChannel *control_channel)
{
	gst_camerasrc_debug("channel init");
	
	control_channel->id = (guint32) - 1;
}

static G_GNUC_UNUSED gboolean gst_camerasrc_control_contains_channel(GstCameraSrc *camerasrc,
                                                                     GstCamerasrcControlChannel *camerasrc_control_channel)
{
	gst_camerasrc_debug("contains channel");

	const GList *item;

	for (item = camerasrc->camera_controls ; item != NULL ; item = item->next) {
		if (item->data == camerasrc_control_channel) {
			return TRUE;
		}
	}

	return FALSE;
}

const GList *gst_camerasrc_control_list_channels(GstCameraSrc *camerasrc)
{
	gst_camerasrc_debug("list channels");
	
	return camerasrc->camera_controls;
}

gboolean gst_camerasrc_control_set_value(GstCameraSrc *camerasrc, GstCameraControlChannel *control_channel, gint value)
{
	gst_camerasrc_debug("set value : %d", value);
	
	int error = CAMERASRC_ERROR;
	
	GstCamerasrcControlChannel *camerasrc_control_channel = GST_CAMERASRC_CONTROL_CHANNEL(control_channel);

	g_return_val_if_fail(camerasrc, FALSE);
	g_return_val_if_fail(gst_camerasrc_control_contains_channel(camerasrc, camerasrc_control_channel), FALSE);

	error = camerasrc_set_control(camerasrc->v4l2_handle, camerasrc_control_channel->id, value);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set value. Ctrl-id [%d], value [%d], err code [%d]", camerasrc_control_channel->id, value, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_get_value(GstCameraSrc *camerasrc, GstCameraControlChannel *control_channel, gint *value)
{
	gst_camerasrc_debug("get value");
	
	int error = CAMERASRC_ERROR;
	
	GstCamerasrcControlChannel *camerasrc_control_channel = GST_CAMERASRC_CONTROL_CHANNEL(control_channel);

	g_return_val_if_fail(camerasrc, FALSE);
	g_return_val_if_fail(gst_camerasrc_control_contains_channel(camerasrc, camerasrc_control_channel), FALSE);

	error = camerasrc_get_control(camerasrc->v4l2_handle, camerasrc_control_channel->id, value);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get control value. Ctrl-id [%d], err code[%x]", camerasrc_control_channel->id, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_set_exposure(GstCameraSrc *camerasrc, gint type, gint value1, gint value2)
{
	gst_camerasrc_debug("set exposure");
	
	int error = CAMERASRC_ERROR;
	camerasrc_frac_t frac;
	
	g_return_val_if_fail(camerasrc, FALSE);

	/* TODO : F number */
	switch (type) {
	case GST_CAMERA_CONTROL_F_NUMBER:
		error = CAMERASRC_SUCCESS;
		break;
	case GST_CAMERA_CONTROL_SHUTTER_SPEED:
		frac.numerator = value1;
		frac.denominator = value2;
		gst_camerasrc_debug("                camerasrc_set_shutter_speed");
		error = camerasrc_set_shutter_speed(camerasrc->v4l2_handle, frac);
		break;
	case GST_CAMERA_CONTROL_ISO:
		gst_camerasrc_debug("                camerasrc_set_iso_value");
		error = camerasrc_set_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_ISO, value1);
		break;
	case GST_CAMERA_CONTROL_PROGRAM_MODE:
		gst_camerasrc_debug("                camerasrc_set_control");
		error = camerasrc_set_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_PROGRAM_MODE, value1);
		break;
	case GST_CAMERA_CONTROL_EXPOSURE_MODE:
		gst_camerasrc_debug("                camerasrc_set_exposure_mode");
		error = camerasrc_set_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_PHOTOMETRY, value1);
		break;
	case GST_CAMERA_CONTROL_EXPOSURE_VALUE:
		frac.numerator = value1;
		frac.denominator = value2;
		gst_camerasrc_debug("                camerasrc_set_exposure_value");
		error = camerasrc_set_exposure_value(camerasrc->v4l2_handle, frac);
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set exposure. Type[%d],value1[%d],value2[%d],err code[%x]", type, value1, value2, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_get_exposure(GstCameraSrc *camerasrc, gint type, gint *value1, gint *value2)
{
	gst_camerasrc_debug("get exposure");
	
	int error = CAMERASRC_ERROR;
	camerasrc_frac_t frac;
	
	g_return_val_if_fail(camerasrc, FALSE);

	/* TODO : F number */
	switch (type) {
	case GST_CAMERA_CONTROL_F_NUMBER:
		break;
	case GST_CAMERA_CONTROL_SHUTTER_SPEED:
		error = camerasrc_get_shutter_speed(camerasrc->v4l2_handle, &frac);
		if (error == CAMERASRC_SUCCESS) {
			*value1 = frac.numerator;
			*value2 = frac.denominator;
		}
		break;
	case GST_CAMERA_CONTROL_ISO:
		error = camerasrc_get_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_ISO, value1);
		break;
	case GST_CAMERA_CONTROL_PROGRAM_MODE:
		error = camerasrc_get_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_PROGRAM_MODE, value1);
		break;
	case GST_CAMERA_CONTROL_EXPOSURE_MODE:
		error = camerasrc_get_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_PHOTOMETRY, value1);
		break;
	case GST_CAMERA_CONTROL_EXPOSURE_VALUE:
		error = camerasrc_get_exposure_value(camerasrc->v4l2_handle, &frac);
		if (error == CAMERASRC_SUCCESS) {
			*value1 = frac.numerator;
			*value2 = frac.denominator;
		}
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get exposure. Type [%d]", type);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_set_capture_mode(GstCameraSrc *camerasrc, gint type, gint value)
{
	/* TODO : single/multishot select(capture mode), output mode, frame count, JPEG quality */

	gst_camerasrc_debug("set capture mode");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	switch (type) {
	case GST_CAMERA_CONTROL_CAPTURE_MODE:
		break;
	case GST_CAMERA_CONTROL_OUTPUT_MODE:
		break;
	case GST_CAMERA_CONTROL_FRAME_COUNT:
		break;
	case GST_CAMERA_CONTROL_JPEG_QUALITY:
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set capture mode. Type[%d],value[%d],err code[%x]", type, value, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_get_capture_mode(GstCameraSrc *camerasrc, gint type, gint *value)
{
	/* TODO : single/multishot select(capture mode), output mode, frame count, JPEG quality */

	gst_camerasrc_debug("get capture mode");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	switch (type) {
	case GST_CAMERA_CONTROL_CAPTURE_MODE:
		break;
	case GST_CAMERA_CONTROL_OUTPUT_MODE:
		break;
	case GST_CAMERA_CONTROL_FRAME_COUNT:
		break;
	case GST_CAMERA_CONTROL_JPEG_QUALITY:
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set capture mode. Type[%d],err code[%x]", type, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_set_strobe(GstCameraSrc *camerasrc, gint type, gint value)
{
	gst_camerasrc_debug("set strobe");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	switch (type) {
	case GST_CAMERA_CONTROL_STROBE_MODE:
		error = camerasrc_set_strobe_mode(camerasrc->v4l2_handle, value);
		break;
	case GST_CAMERA_CONTROL_STROBE_CONTROL:
	case GST_CAMERA_CONTROL_STROBE_CAPABILITIES:
	case GST_CAMERA_CONTROL_STROBE_STATUS:
	case GST_CAMERA_CONTROL_STROBE_EV:
	default:
		gst_camerasrc_debug("Not supported type[%d], return CAMERASRC_ERR_DEVICE_NOT_SUPPORT.", type);
		error = CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
		break;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set strobe. Type[%d],value[%d],err code[%x]", type, value, error);
		return FALSE;
	}

	gst_camerasrc_debug("Succeed to set strobe. Type[%d],value[%d]", type, value);

	return TRUE;
}

gboolean gst_camerasrc_control_get_strobe(GstCameraSrc *camerasrc, gint type, gint *value)
{
	gst_camerasrc_debug("get strobe");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	switch (type) {
	case GST_CAMERA_CONTROL_STROBE_MODE:
		error = camerasrc_get_strobe_mode(camerasrc->v4l2_handle, (camerasrc_strobe_mode_t *)value);
		break;
	case GST_CAMERA_CONTROL_STROBE_CONTROL:
	case GST_CAMERA_CONTROL_STROBE_CAPABILITIES:
	case GST_CAMERA_CONTROL_STROBE_STATUS:
	case GST_CAMERA_CONTROL_STROBE_EV:
	default:
		gst_camerasrc_debug("Not supported type[%d].", type);
		error = CAMERASRC_ERR_DEVICE_NOT_SUPPORT;
		break;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get strobe. Type[%d],err code[%x]", type, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_set_detect(GstCameraSrc *camerasrc, gint type, gint value)
{
	gst_camerasrc_debug("set detect");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	/* TODO */
	switch (type) {
	case GST_CAMERA_CONTROL_FACE_DETECT_MODE:
		break;
	case GST_CAMERA_CONTROL_FACE_DETECT_NUMBER:
		break;
	case GST_CAMERA_CONTROL_FACE_FOCUS_SELECT:
		break;
	case GST_CAMERA_CONTROL_FACE_SELECT_NUMBER:
		break;
	case GST_CAMERA_CONTROL_FACE_DETECT_STATUS:
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set detect. Type[%d],value[%d],err code[%x]", type, value, error);
		return FALSE;
	}

	return error;
}

gboolean gst_camerasrc_control_get_detect(GstCameraSrc *camerasrc, gint type, gint *value)
{
	gst_camerasrc_debug("get detect");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);
	
	/* TODO : detection number, focus select, select number, detect status */
	switch (type) {
	case GST_CAMERA_CONTROL_FACE_DETECT_MODE:
		break;
	case GST_CAMERA_CONTROL_FACE_DETECT_NUMBER:
		break;
	case GST_CAMERA_CONTROL_FACE_FOCUS_SELECT:
		break;
	case GST_CAMERA_CONTROL_FACE_SELECT_NUMBER:
		break;
	case GST_CAMERA_CONTROL_FACE_DETECT_STATUS:
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get detect. Type[%d],err code[%x]", type, error);
		return FALSE;
	}

	return error;
}

gboolean gst_camerasrc_control_set_zoom(GstCameraSrc *camerasrc, gint type, gint value)
{
	gst_camerasrc_debug("set zoom");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	switch (type) {
	case GST_CAMERA_CONTROL_DIGITAL_ZOOM:
		error = camerasrc_set_control (camerasrc->v4l2_handle, CAMERASRC_CTRL_DIGITAL_ZOOM, value);
		break;
	case GST_CAMERA_CONTROL_OPTICAL_ZOOM:
		error = camerasrc_set_control (camerasrc->v4l2_handle, CAMERASRC_CTRL_OPTICAL_ZOOM, value);
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set zoom. Type[%d],value[%d],err code[%x]", type, value, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_get_zoom(GstCameraSrc *camerasrc, gint type, gint *value)
{
	gst_camerasrc_debug("get zoom");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	switch (type) {
	case GST_CAMERA_CONTROL_DIGITAL_ZOOM:
		error = camerasrc_get_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_DIGITAL_ZOOM, value);
		break;
	case GST_CAMERA_CONTROL_OPTICAL_ZOOM:
		error = camerasrc_get_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_OPTICAL_ZOOM, value);
		break;
	default:
		gst_camerasrc_debug("Not supported type.");
		return FALSE;
	}

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get zoom. Type[%d],err code[%x]", type, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_set_focus(GstCameraSrc *camerasrc, gint focus_mode, gint focus_range)
{
	gst_camerasrc_debug("set focus");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_init_autofocusing_mode(camerasrc->v4l2_handle, focus_mode, focus_range);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set AF mode.");
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_get_focus(GstCameraSrc *camerasrc, gint *focus_mode, gint *focus_range)
{
	gst_camerasrc_debug("get focus");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_get_autofocusing_mode(camerasrc->v4l2_handle, (camerasrc_af_mode_t *)focus_mode, (camerasrc_af_scan_range_t *)focus_range);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get AF mode.");
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_start_auto_focus(GstCameraSrc *camerasrc)
{
	gst_camerasrc_debug("start auto focus");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_start_autofocusing(camerasrc->v4l2_handle);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to start AF. error[%x]", error);
		return FALSE;
	} else {
		gst_camerasrc_debug("Succeeded to start AF.");
		return TRUE;
	}
}

gboolean gst_camerasrc_control_stop_auto_focus(GstCameraSrc *camerasrc)
{
	gst_camerasrc_debug("stop auto focus");
	
	int error = CAMERASRC_ERROR;
	int try_count = 0;
	camerasrc_auto_focus_status_t af_status = CAMERASRC_AUTO_FOCUS_STATUS_RELEASED;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_stop_autofocusing(camerasrc->v4l2_handle);
	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to stop autofocus.");
		return FALSE;
	}

	while (try_count++ < CAMERA_CONTROL_AF_STOP_TOTALTIME / CAMERA_CONTROL_AF_STOP_INTERVAL) {
		error = camerasrc_get_autofocusing_status(camerasrc->v4l2_handle, &af_status);
		if (error != CAMERASRC_SUCCESS) {
			gst_camerasrc_debug("Failed to get af status.(%x)", error);
			return FALSE;
		}

		if (af_status == CAMERASRC_AUTO_FOCUS_STATUS_RELEASED) {
			gst_camerasrc_debug("AF Stop done. try count[%d]", try_count);
			break;
		}

		usleep(CAMERA_CONTROL_AF_STOP_INTERVAL);
	}

	return TRUE;
}

gboolean gst_camerasrc_control_set_focus_level(GstCameraSrc *camerasrc, gint focus_level)
{
	/* TODO : */

	gst_camerasrc_debug("Not support");
	return FALSE;
}

gboolean gst_camerasrc_control_get_focus_level(GstCameraSrc *camerasrc, gint *focus_level)
{
	/* TODO : */

	gst_camerasrc_debug("Not support");
	return FALSE;
}

gboolean gst_camerasrc_control_set_auto_focus_area(GstCameraSrc *camerasrc, GstCameraControlRectType rect)
{
	gst_camerasrc_debug("set auto focus area");
	
	int error = CAMERASRC_ERROR;
	camerasrc_rect_t camerasrc_rect = { 0, 0, 0, 0 };

	g_return_val_if_fail(camerasrc, FALSE);

	if (camerasrc->camera_id == CAMERASRC_DEV_ID_SECONDARY) {
		GST_INFO_OBJECT(camerasrc, "It's secondary camera. Skip setting...");
		return TRUE;
	}

	camerasrc_rect.x = rect.x;
	camerasrc_rect.y = rect.y;
	camerasrc_rect.width = rect.width;
	camerasrc_rect.height = rect.height;

	GST_INFO_OBJECT(camerasrc, "Set AF area %d,%d,%dx%d",
	                           camerasrc_rect.x, camerasrc_rect.y,
	                           camerasrc_rect.width, camerasrc_rect.height);

	error = camerasrc_set_autofocusing_area(camerasrc->v4l2_handle, &camerasrc_rect);

	if (error != CAMERASRC_SUCCESS) {
		GST_ERROR_OBJECT(camerasrc, "Failed to set auto focus area.");
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_get_auto_focus_area(GstCameraSrc *camerasrc, GstCameraControlRectType *rect)
{
	gst_camerasrc_debug("get auto focus area");
	
	int error = CAMERASRC_ERROR;
	camerasrc_rect_t camerasrc_rect = { 0, 0, 0, 0 };

	g_return_val_if_fail(camerasrc, FALSE);
	g_return_val_if_fail(rect, FALSE);

	error = camerasrc_get_autofocusing_area(camerasrc->v4l2_handle, &camerasrc_rect);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get auto focus area.");

		rect->x = rect->y = -1;
		rect->width = rect->height = -1;

		return FALSE;
	}

	rect->x = camerasrc_rect.x;
	rect->y = camerasrc_rect.y;
	rect->width = camerasrc_rect.width;
	rect->height = camerasrc_rect.height;

	return TRUE;
}

gboolean gst_camerasrc_control_set_wdr(GstCameraSrc *camerasrc, gint value)
{
	gst_camerasrc_debug("set wdr");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_set_control(camerasrc->v4l2_handle, CAMERASRC_CTRL_WIDE_DYNAMIC_RANGE, value);
	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set wdr. value[%d],err code[%x]", value, error);
		return FALSE;
	}

	return TRUE;
}

gboolean gst_camerasrc_control_get_wdr(GstCameraSrc *camerasrc, gint *value)
{
	gst_camerasrc_debug("get wdr");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_get_control (camerasrc->v4l2_handle, CAMERASRC_CTRL_WIDE_DYNAMIC_RANGE, value);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get wdr. err code[%x]", error);
		return FALSE;
	}
	
	return TRUE;
}

gboolean gst_camerasrc_control_set_ahs(GstCameraSrc *camerasrc, gint value)
{
	gst_camerasrc_debug("set ahs");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_set_control (camerasrc->v4l2_handle, CAMERASRC_CTRL_ANTI_HANDSHAKE, value);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to set ahs. value[%d],err code[%x]", value, error);
		return FALSE;
	}
	
	return TRUE;
}

gboolean gst_camerasrc_control_get_ahs(GstCameraSrc *camerasrc, gint *value)
{
	gst_camerasrc_debug("get ahs");
	
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_get_control (camerasrc->v4l2_handle, CAMERASRC_CTRL_ANTI_HANDSHAKE, value);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get ahs. err code[%x]", error);
		return FALSE;
	}
	
	return TRUE;
}


gboolean gst_camerasrc_control_get_exif_info(GstCameraSrc *camerasrc, GstCameraControlExifInfo *info)
{
	gst_camerasrc_debug("get exif info");
	
	int error = CAMERASRC_ERROR;
	camerasrc_exif_t exif_struct;

	g_return_val_if_fail(camerasrc, FALSE);

	error = camerasrc_get_exif_info(camerasrc->v4l2_handle, &exif_struct);

	if (error != CAMERASRC_SUCCESS) {
		gst_camerasrc_debug("Failed to get exif info. err code[%x]", error);
		return FALSE;
	}

	/* Dynamic value */
	info->exposure_time_numerator = exif_struct.exposure_time_numerator;
	info->exposure_time_denominator = exif_struct.exposure_time_denominator;
	info->shutter_speed_numerator = exif_struct.shutter_speed_numerator;
	info->shutter_speed_denominator = exif_struct.shutter_speed_denominator;
	info->brigtness_numerator = exif_struct.brigtness_numerator;
	info->brightness_denominator = exif_struct.brightness_denominator;
	info->iso = exif_struct.iso;
	info->flash = exif_struct.flash;
	info->metering_mode = exif_struct.metering_mode;
	info->exif_image_width = exif_struct.exif_image_width;
	info->exif_image_height = exif_struct.exif_image_height;
	info->software_used = exif_struct.software_used;
	info->exposure_bias_in_APEX = exif_struct.exposure_bias_in_APEX;

	/* Fixed value */
	info->component_configuration = exif_struct.component_configuration;
	info->colorspace = exif_struct.colorspace;
	info->max_lens_aperture_in_APEX = exif_struct.max_lens_aperture_in_APEX;

	info->focal_len_numerator = exif_struct.focal_len_numerator;
	info->focal_len_denominator = exif_struct.focal_len_denominator;
	info->aperture_f_num_numerator = exif_struct.aperture_f_num_numerator;
	info->aperture_f_num_denominator = exif_struct.aperture_f_num_denominator;
	info->aperture_in_APEX = exif_struct.aperture_in_APEX;

	return TRUE;
}

gboolean gst_camerasrc_control_get_basic_dev_info (GstCameraSrc *camerasrc, gint dev_id, GstCameraControlCapsInfoType *info)
{
	gst_camerasrc_debug("get basic dev info");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	/**
	 * Just implementation issue, but at this time, we assume
	 * GstCameraControlCapsInfoType is exactly same with camerasrc_caps_info_t
	 * For performance.
	 * Here is plugin code. we can do like this?
	 */
#if 1
	error = camerasrc_read_basic_dev_info(dev_id, (camerasrc_caps_info_t*)info);
	if (error != CAMERASRC_SUCCESS) {
		return FALSE;
	}
#else
	int i, j, k;
	camerasrc_caps_info_t caps_info;

	error = camerasrc_read_basic_dev_info(dev_id, &caps_info);
	if (error != CAMERASRC_SUCCESS) {
		return FALSE;
	}

	if (caps_info.num_fmt_desc != 0) {
		info->num_fmt_desc = caps_info.num_fmt_desc;
		for (i = 0 ; i < caps_info.num_fmt_desc ; i++) {
			if (caps_info.fmt_desc[i].num_resolution != 0) {
				info->fmt_desc[i].fcc = caps_info.fmt_desc[i].fcc;
				info->fmt_desc[i].num_resolution = caps_info.fmt_desc[i].num_resolution;
				for (j = 0 ; j < caps_info.fmt_desc[i].num_resolution ; j++) {
					if (caps_info.fmt_desc[i].resolutions[j].num_avail_tpf != 0) {
						info->fmt_desc[i].resolutions[j].w = caps_info.fmt_desc[i].resolutions[j].w;
						info->fmt_desc[i].resolutions[j].h = caps_info.fmt_desc[i].resolutions[j].h;
						info->fmt_desc[i].resolutions[j].num_avail_tpf = caps_info.fmt_desc[i].resolutions[j].num_avail_tpf;
						for (k = 0 ; k < caps_info.fmt_desc[i].resolutions[j].num_avail_tpf ; k++) {
							info->fmt_desc[i].resolutions[j].tpf[k].num = caps_info.fmt_desc[i].resolutions[j].tpf[k].num;
							info->fmt_desc[i].resolutions[j].tpf[k].den = caps_info.fmt_desc[i].resolutions[j].tpf[k].den;
						}
					} else {
						/* No available timeperframe */
						return FALSE;
					}
				}
			} else {
				/* No available resolution set */
				return FALSE;
			}
		}
	} else {
		/* No available image format(fourcc) */
		return FALSE;
	}
#endif
	return TRUE;
}

gboolean gst_camerasrc_control_get_misc_dev_info(GstCameraSrc *camerasrc, gint dev_id, GstCameraControlCtrlListInfoType  *info)
{
	gst_camerasrc_debug("get misc dev info");

	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	/**
	 * Just implementation issue, but at this time, we assume
	 * GstCameraControlCtrlListInfoType is exactly same with camerasrc_ctrl_list_info_t
	 * For performance.
	 * Here is plugin code. we can do like this?
	 */
#if 1
	error = camerasrc_read_misc_dev_info(dev_id, (camerasrc_ctrl_list_info_t*)info);
	if (error != CAMERASRC_SUCCESS) {
		return FALSE;
	}
#else
	int i, j;
	camerasrc_ctrl_list_info_t ctrl_info;

	error = camerasrc_read_misc_dev_info(dev_id, &ctrl_info);
	if (error != CAMERASRC_SUCCESS) {
		return FALSE;
	}

	if (ctrl_info.num_ctrl_list_info != 0) {
		info->num_ctrl_list_info = ctrl_info.num_ctrl_list_info;
		for (i = 0 ; i < ctrl_info.num_ctrl_list_info ; i++) {
			info->ctrl_info[i].camerasrc_ctrl_id = ctrl_info.ctrl_info[i].camerasrc_ctrl_id;
			info->ctrl_info[i].v4l2_ctrl_id = ctrl_info.ctrl_info[i].v4l2_ctrl_id;
			info->ctrl_info[i].ctrl_type = ctrl_info.ctrl_info[i].ctrl_type;
			info->ctrl_info[i].max = ctrl_info.ctrl_info[i].max;
			info->ctrl_info[i].min = ctrl_info.ctrl_info[i].min;
			info->ctrl_info[i].step = ctrl_info.ctrl_info[i].step;
			info->ctrl_info[i].default_val = ctrl_info.ctrl_info[i].default_val;
			info->ctrl_info[i].num_ctrl_menu = ctrl_info.ctrl_info[i].num_ctrl_menu;

			memcpy(info->ctrl_info[i].ctrl_name,ctrl_info.ctrl_info[i].ctrl_name,MAX_SZ_CTRL_NAME_STRING);

			if (ctrl_info.ctrl_info[i].ctrl_type == CTRL_TYPE_ARRAY && ctrl_info.ctrl_info[i].num_ctrl_menu != 0) {
				for (j = 0 ; j < ctrl_info.ctrl_info[i].num_ctrl_menu ; j++) {
					info->ctrl_info[i].ctrl_menu[j].menu_index = ctrl_info.ctrl_info[i].ctrl_menu[j].menu_index;
					memcpy(info->ctrl_info[i].ctrl_menu[j].menu_name, ctrl_info.ctrl_info[i].ctrl_menu[j].menu_name, MAX_SZ_CTRL_NAME_STRING);
				}
			} else {
				/* Not a menu type or not available menus */
				return FALSE;
			}
		}
	} else {
		/* Not avaliable controls */
		return FALSE;
	}
#endif
	return TRUE;
}

gboolean gst_camerasrc_control_get_extra_dev_info(GstCameraSrc *camerasrc, gint dev_id, GstCameraControlExtraInfoType  *info)
{
	int error = CAMERASRC_ERROR;

	g_return_val_if_fail(camerasrc, FALSE);

	gst_camerasrc_debug("get extra dev info");

	/**
	 * Just implementation issue, but at this time, we assume
	 * GstCameraControlCtrlListInfoType is exactly same with camerasrc_ctrl_list_info_t
	 * For performance.
	 * Here is plugin code. we can do like this?
	 */

	error = camerasrc_read_extra_dev_info(dev_id, (camerasrc_extra_info_t*)info);
	if (error != CAMERASRC_SUCCESS) {
		return FALSE;
	}

	return TRUE;
}

void gst_camerasrc_control_set_capture_command(GstCameraSrc *camerasrc, GstCameraControlCaptureCommand cmd)
{
	gst_camerasrc_debug("set capture command");

	if (camerasrc == NULL) {
		gst_camerasrc_debug("camerasrc is NULL");
		return;
	}

	gst_camerasrc_set_capture_command(camerasrc, cmd);

	return;
}
