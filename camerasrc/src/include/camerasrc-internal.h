/*
 * camerasrc
 *
 * Copyright (c) 2000 - 2015 Samsung Electronics Co., Ltd. All rights reserved.
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

#ifndef __CAMERASRC_INTERNAL_H__
#define __CAMERASRC_INTERNAL_H__

#include "camerasrc-common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Video 4 linux control ID definitions (Extended by kernel team)
 * extended pixel format for V4l2
 */


/**
 * Miscellaneous camera-dependent definitions
 */
#define CAMERASRC_AF_INTERVAL                   20000
#define CAMERASRC_TIMEOUT_CRITICAL_VALUE        3000

/* EXIF */
#define EXIF_DEFAULT_SHUTTER_SPEED_DENOMINATOR       256
#define EXIF_DEFAULT_EXPOSURE_TIME_NUMERATOR         1
#define EXIF_DEFAULT_BRIGHTNESS_DENOMINATOR          256
#define EXIF_DEFAULT_ISO_SPEED_RATING                100
#define EXIF_DEFAULT_FNUMBER_NUMERATOR_REAR          26
#define EXIF_DEFAULT_FNUMBER_DENOMINATOR_REAR        10
#define EXIF_DEFAULT_FNUMBER_NUMERATOR_FRONT         28
#define EXIF_DEFAULT_FNUMBER_DENOMINATOR_FRONT       10
#define EXIF_DEFAULT_FOCAL_LENGTH_NUMERATOR_REAR     37
#define EXIF_DEFAULT_FOCAL_LENGTH_DENOMINATOR_REAR   10
#define EXIF_DEFAULT_FOCAL_LENGTH_NUMERATOR_FRONT    273
#define EXIF_DEFAULT_FOCAL_LENGTH_DENOMINATOR_FRONT  100
#define EXIF_DEFAULT_COLOR_SPACE                     1
#define EXIF_DEFAULT_COMPONENT_CONFIGRATION          (0x00000000) | (0x00000001) | (0x00000002 << 8) | (0x00000003 << 16)    /* Y Cb Cr - */

#ifdef __cplusplus
}
#endif

#endif /*__CAMERASRC_INTERNAL_H__*/
