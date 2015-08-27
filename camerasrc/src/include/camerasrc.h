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

#ifndef __CAMERASRC_H__
#define __CAMERASRC_H__

#include <stdint.h> /* to use uint64_t */
#include <camerasrc-error.h>
#include <gst/video/cameracontrol.h>

/* for Tizen Buffer Manager */
#include <exynos_drm.h>
#include <tbm_bufmgr.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_FACE_NUM                            16
#define MAX_PLANE_NUM                           4
#define CAMERASRC_PREVIEW_BUFFER_NUM            12       /* number of request buffer when preview */
#define CAMERASRC_VIDEO_BUFFER_NUM              7
#define CAMERASRC_STILL_BUFFER_NUM              1

#define SPARE_SIZE                              (2*16 * 1024)
#define BAYER_PLANE_COUNT                       2
#define ISP_PLANE_COUNT                         2
#define SCC_PLANE_COUNT                         2

/**
 * Type definition of av camera src handle.
 */
typedef void *camsrc_handle_t;


/* ENUMERATION DEFINITIONS */
/*! @enum camerasrc_state_t
 *  @brief Enumeration type for state transition
 */
typedef enum {
    CAMERASRC_STATE_NONE = 0,
    CAMERASRC_STATE_CREATED,
    CAMERASRC_STATE_REALIZED,
    CAMERASRC_STATE_READY,
    CAMERASRC_STATE_PREVIEW,
    CAMERASRC_STATE_STILL,
    CAMERASRC_STATE_VIDEO,
    CAMERASRC_STATE_UNREALIZED,
    CAMERASRC_STATE_DESTROYED,
    CAMERASRC_STATE_AF_IN_PROGRESS,
}camerasrc_state_t;

/*! @enum camerasrc_dev_id_t
 *  @brief Enumeration type for camera device ID
 *
 *  Devices will be managed by this IDs. (Independent with device index of V4L2)
 */
typedef enum {
    CAMERASRC_DEV_ID_PRIMARY,           /**< Higher resolution camera*/
    CAMERASRC_DEV_ID_SECONDARY,         /**< Lower resolution camera*/
    CAMERASRC_DEV_ID_EXTENSION,         /**< reserved for extension*/
    CAMERASRC_DEV_ID_UNKNOWN,           /**< reserved for extension*/
    CAMERASRC_DEV_ID_NUM,               /**< Number of IDs*/
}camerasrc_dev_id_t;

typedef enum {
    CAMERASRC_COLOR_VIOLET = 0,
    CAMERASRC_COLOR_PURPLE,
    CAMERASRC_COLOR_MAGENTA_1,
    CAMERASRC_COLOR_MAGENTA_2,
    CAMERASRC_COLOR_RED_1,
    CAMERASRC_COLOR_RED_2,
    CAMERASRC_COLOR_BROWN,
    CAMERASRC_COLOR_YELLOW,
    CAMERASRC_COLOR_GREEN_1,
    CAMERASRC_COLOR_GREEN_2,
    CAMERASRC_COLOR_GREEN_3,
    CAMERASRC_COLOR_GREEN_4,
    CAMERASRC_COLOR_COBALT_BLUE,
    CAMERASRC_COLOR_CYAN,
    CAMERASRC_COLOR_BLUE_1,
    CAMERASRC_COLOR_BLUE_2,
    CAMERASRC_COLOR_GRAY,
    CAMERASRC_COLOR_NUM,
}camerasrc_color_t;

typedef enum {
    CAMERASRC_PARTCOLOR_MODE_NONE = 0,
    CAMERASRC_PARTCOLOR_MODE_SWAP,
    CAMERASRC_PARTCOLOR_MODE_ACCENT,
    CAMERASRC_PARTCOLOR_MODE_NUM,
}camerasrc_partcolor_mode_t;

/*! @enum camerasrc_ctrl_t
 *  @brief Enumeration type for camera controls
 *
 *  Special control entries for camera effects
 *
 *  @remark Strobo can be controlled by this entry and ::camerasrc_set_strobo_status
 */
typedef enum {
    CAMERASRC_CTRL_BRIGHTNESS = 0,          /**< Brightness control entry*/
    CAMERASRC_CTRL_CONTRAST,                /**< Contrast control entry*/
    CAMERASRC_CTRL_DIGITAL_ZOOM,            /**< Digital zoom control entry*/
    CAMERASRC_CTRL_OPTICAL_ZOOM,            /**< Optical zoom control entry*/
    CAMERASRC_CTRL_WHITE_BALANCE,           /**< White balance control entry*/
    CAMERASRC_CTRL_COLOR_TONE,              /**< Color tone control entry*/
    CAMERASRC_CTRL_PROGRAM_MODE,            /**< Program mode control entry*/
    CAMERASRC_CTRL_FLIP,                    /**< Flip control entry*/
    CAMERASRC_CTRL_ANTI_HANDSHAKE,          /**< Anti-handshake control, 0:OFF / 1:ON / 2:AUTO / 3:MOVIE */
    CAMERASRC_CTRL_WIDE_DYNAMIC_RANGE,      /**< wide dynamic control, 0:OFF / 1:ON / 2:AUTO */
    CAMERASRC_CTRL_SATURATION,              /**< Saturation value control */
    CAMERASRC_CTRL_SHARPNESS,               /**< Sharpness value control */
    CAMERASRC_CTRL_ISO,                     /**< Sensor sensitivity*/
    CAMERASRC_CTRL_PHOTOMETRY,              /**< Exposure mode*/
    CAMERASRC_CTRL_NUM,                     /**< Number of Controls*/
}camerasrc_ctrl_t;

/*! @enum camerasrc_af_mode_t
 *  @brief AF operation mode
 */
typedef enum {
    CAMERASRC_AF_MODE_AUTO = 0,      /**< Auto Focus */
    CAMERASRC_AF_MODE_MANUAL,        /**< Manual Focus */
    CAMERASRC_AF_MODE_PAN,           /**< Pan Focus */
    CAMERASRC_AF_MODE_TOUCH_AUTO,    /**< Touch Auto Focus */
    CAMERASRC_AF_MODE_CONTINUOUS,    /**< Continuous Focus */
    CAMERASRC_AF_MODE_NUM,           /**< Number of AF modes */
}camerasrc_af_mode_t;

/*! @enum camerasrc_af_scan_range_t
 *  @brief AF scan range
 *  AF scan range
 */
typedef enum {
    CAMERASRC_AF_RANGE_NORMAL = 0,   /**< Scan autofocus in normal range */
    CAMERASRC_AF_RANGE_MACRO,        /**< Scan autofocus in macro range(close distance) */
    CAMERASRC_AF_RANGE_FULL,         /**< Scan autofocus in full range(all range scan, limited by dev spec) */
    CAMERASRC_AF_RANGE_NUM,          /**< Number of AF range types */
}camerasrc_af_scan_range_t;

/*! @enum camerasrc_resol_name_t
 *  @brief Enumeration type of resolution settings based on traditional resolution name
 *  Means pixel order of contents.
 *  @remark In the Grandprix, only YUV422P & RGGB8 is used
 */
typedef enum {
    CAMERASRC_RESOL_QQCIF = 0,          /**< 88 x 72 */
    CAMERASRC_RESOL_QQVGA,              /**< 160 x 120 */
    CAMERASRC_RESOL_QCIF,               /**< 176 x 144 */
    CAMERASRC_RESOL_QVGA,               /**< 320 x 240 */
    CAMERASRC_RESOL_CIF,                /**< 352 x 288 */
    CAMERASRC_RESOL_VGA,                /**< 640 x 480 */
    CAMERASRC_RESOL_WVGA,               /**< 800 x 480 */
    CAMERASRC_RESOL_SVGA,               /**< 800 x 600 */
    CAMERASRC_RESOL_WSXGA,              /**< 1280 x 960 (1M) */
    CAMERASRC_RESOL_UXGA,               /**< 1600 x 1200 (2M) */
    CAMERASRC_RESOL_QXGA,               /**< 2048 x 1536 (3M) */
    CAMERASRC_RESOL_WQSXGA,             /**< 2560 x 1920 (5M) */
    CAMERASRC_RESOL_720P,               /**< 1280 x 720 (720P) */
    CAMERASRC_RESOL_WQVGA,              /**< 400 x 240 */
    CAMERASRC_RESOL_RQVGA,              /**< 240 x 320 */
    CAMERASRC_RESOL_RWQVGA,             /**< 240 x 400 */
    CAMERASRC_RESOL_QVGA_60FPS,         /**< 320 x 240 60FPS(Slow motion I) */
    CAMERASRC_RESOL_QVGA_120FPS,        /**< 320 x 240 60FPS(Slow motion II) */
    CAMERASRC_RESOL_NUM,
}camerasrc_resol_name_t;

/*! @enum camerasrc_pix_format_t
 *  @brief Means order of pixel of contents
 *  Means pixel order of contents.
 *  @remark In the Grandprix, only YUV422P & RGGB8 is used
 */
typedef enum {
    CAMERASRC_PIX_NONE = -1,      /**< Default value or Not supported */
    CAMERASRC_PIX_YUV422P = 0,    /**< Pixel format like YYYYYYYYUUUUVVVV*/
    CAMERASRC_PIX_YUV420P,        /**< Pixel format like YYYYYYYYUUVV*/
    CAMERASRC_PIX_YUV420,         /**< Pixel format like YYYYYYYYUVUV*/
    CAMERASRC_PIX_SN12,           /**< YUV420 (interleaved, non-linear) */
    CAMERASRC_PIX_YUY2,           /**< YUV 4:2:2 as for UYVY but with different component ordering within the u_int32 macropixel */
    CAMERASRC_PIX_RGGB8,          /**< Raw RGB Pixel format like CCD order, a pixel consists of 8 bits, Actually means JPEG + JPEG image output */
    CAMERASRC_PIX_RGGB10,         /**< Raw RGB Pixel format like CCD order, a pixel consists of 10 bits, Actually means JPEG + YUV image output */
    CAMERASRC_PIX_RGB565,         /**< Raw RGB Pixel format like CCD order, a pixel consists of 10 bits, Actually means JPEG + YUV image output */
    CAMERASRC_PIX_SBGGR12,        /**< Bayer RGB format : 12  BGBG.. GRGR.. : V4L2_PIX_FMT_SBGGR12 */
    CAMERASRC_PIX_UYVY,           /**< YUV 4:2:2 */
    CAMERASRC_PIX_NV12,           /**< YUV 4:2:0, 8-bit Y plane followed by an interleaved U/V plane with 2x2 subsampling */
    CAMERASRC_PIX_SN21,           /**< YUV420 (interleaved V/U plane, non-linear) */
    CAMERASRC_PIX_NV21,           /**< YUV 4:2:0, 8-bit Y plane followed by an interleaved V/U plane with 2x2 subsampling */
    CAMERASRC_PIX_YV12,           /**< Pixel format like YYYYYYYYVVUU */
    CAMERASRC_PIX_NUM,            /**< Number of pixel formats*/
}camerasrc_pix_format_t;

/*! @enum camerasrc_colorspace_t
 *  @brief Means stored order or compressed status of image.
 *  Means stored order or compressed status of image. supplements of camerasrc_pix_format_t
 *
 *  @note RAW means RGB/YUV pixel data, JPEG means compressed JPG file with marker information(header)
 */
typedef enum {
    CAMERASRC_COL_NONE = -1,    /**< Default value or Not supported */
    CAMERASRC_COL_RAW,          /**< Non-compressed RGB/YUV pixel data*/
    CAMERASRC_COL_JPEG,         /**< Compressed jpg data*/
    CAMERASRC_COL_NUM,          /**< Number of colorspace data*/
}camerasrc_colorspace_t;

/*! @enum camerasrc_auto_focus_status_t
 *  @brief AF status
 *  AF status
 */
typedef enum {
    CAMERASRC_AUTO_FOCUS_STATUS_RELEASED,       /**< AF status released */
    CAMERASRC_AUTO_FOCUS_STATUS_ONGOING,        /**< AF in progress */
    CAMERASRC_AUTO_FOCUS_STATUS_NUM,            /**< Number of AF status*/
}camerasrc_auto_focus_status_t;

/*! @enum camerasrc_auto_focus_cmd_t
 *  @brief AF status
 *  AF status
 */
typedef enum {
    CAMERASRC_AUTO_FOCUS_CMD_NULL,              /**< Null command */
    CAMERASRC_AUTO_FOCUS_CMD_START,             /**< Start AF */
    CAMERASRC_AUTO_FOCUS_CMD_STOP,              /**< Stop AF */
    CAMERASRC_AUTO_FOCUS_CMD_KILL,              /**< Kill AF thread */
    CAMERASRC_AUTO_FOCUS_CMD_NUM,               /**< Number of AF command*/
}camerasrc_auto_focus_cmd_t;

/*! @enum camerasrc_auto_focus_result_t
 *  @brief AF status
 *  AF status
 */
typedef enum {
    CAMERASRC_AUTO_FOCUS_RESULT_RESTART = 0,    /**< AF Restart */
    CAMERASRC_AUTO_FOCUS_RESULT_FUCUSING,       /**< AF Focusing */
    CAMERASRC_AUTO_FOCUS_RESULT_FOCUSED,        /**< AF Focused */
    CAMERASRC_AUTO_FOCUS_RESULT_FAILED,         /**< AF failed */
    CAMERASRC_AUTO_FOCUS_RESULT_NUM,            /**< Number of AF result */
}camerasrc_auto_focus_result_t;

/*! @enum camerasrc_ae_lock_t
 *  @brief
 */
typedef enum {
    CAMERASRC_AE_LOCK = 0,
    CAMERASRC_AE_UNLOCK,
    CAMERASRC_AE_NUM,
}camerasrc_ae_lock_t;

/*! @enum camerasrc_io_method_t
 *  @brief
 */
typedef enum {
    CAMERASRC_IO_METHOD_READ= 0,
    CAMERASRC_IO_METHOD_MMAP,
    CAMERASRC_IO_METHOD_USRPTR,
    CAMERASRC_IO_METHOD_DMABUF,
    CAMERASRC_IO_METHOD_NUM,
}camerasrc_io_method_t;

/*! @enum camerasrc_sensor_mode_t
 *  @brief
 */
typedef enum {
    CAMERASRC_SENSOR_MODE_CAMERA = 0,
    CAMERASRC_SENSOR_MODE_MOVIE,
} camerasrc_sensor_mode_t;

/*! @enum camerasrc_buffer_queued_status
 *  @brief
 */
typedef enum {
	CAMERASRC_BUFFER_QUEUED   = 0,
	CAMERASRC_BUFFER_DEQUEUED = 1,
}camerasrc_buffer_queued_status;

/* STRUCTURE DEFINITIONS */

typedef struct _camerasrc_rect_t {
    int x;
    int y;
    int width;
    int height;
} camerasrc_rect_t;

/*! @struct camsrc_frac_t
 *  @brief Time per frame or frame per second will be expressed by this structure
 *  Time per frame or frame per second will be expressed by this structure
 */
typedef struct _camerasrc_frac_t {
    int numerator;              /**< Upper number of fraction*/
    int denominator;            /**< Lower number of fraction*/
} camerasrc_frac_t;

/*! @struct camsrc_exif_info_t
 *  @brief EXIF information in captured image when preview format is Interleaved data
 *  EXIF information in captured image when preview format is Interleaved data
 */
typedef struct _camerasrc_exif_info_t {
    int flash;                          /**< Flash */
    int iso_speed_rating;               /**< ISO Speed Raring */
    camerasrc_frac_t shutter_speed;     /**< Shutter Speed */
    camerasrc_frac_t exposure_time;     /**< Exposure Time */
    camerasrc_frac_t brightness;        /**< Brightness */
} camerasrc_exif_info_t;

/*! @struct camerasrc_buffer_t
 *  @brief data buffer
 *  Image data buffer
 */
typedef struct _camerasrc_buffer_t {
	/* Supports for Planes & DMA-buf */
	struct {
		unsigned int length;    /**< Size of stored data */
		unsigned char *start;   /**< Start address of data */
		int fd;                 /**< dmabuf-fd */
		tbm_bo bo;              /**< tbm buffer object */
	} planes[MAX_PLANE_NUM];                    /**< planes: SCMN_IMGB_MAX_PLANE */
	int num_planes;
	int width;                                      /**< width of image */
	int height;                                     /**< height of image */
} camerasrc_buffer_t;



/*! @struct camerasrc_usr_buf_t
 *  @brief data buffer set to present usrptr buffer to camsrctem
 *  Image data buffer set
 */
typedef struct {
    camerasrc_buffer_t* present_buffer;
    unsigned int num_buffer;
} camerasrc_usr_buf_t;

/*! @struct camerasrc_dimension_t
 *  @brief For non-regular size resolution
 *  width and height can be set independently
 */
typedef struct _camerasrc_dimension_t {
    int width;
    int height;
} camerasrc_dimension_t;

/*! @union  camerasrc_size_t
 *  @brief Size can be expressed by resolution name(predefined) and dimension(x, y)
 */

/*! @struct camerasrc_format_t
 *  @brief Format description structure
 *  in/output format description structure just like v4l2_format
 */
typedef struct _camerasrc_format_t {
    camerasrc_dimension_t img_size;     /**< Image size */
    camerasrc_dimension_t capture_size; /**< Capture size */
    camerasrc_dimension_t thumb_size;   /**< Thumbnail size */
    camerasrc_pix_format_t pix_format;  /**< Pixel format */
    int num_planes;                     /**< bytes per a line*/
    int bytesperline;                   /**< bytes per a line*/
    int sizeimage;                      /**< size of whole image*/
    camerasrc_colorspace_t colorspace;  /**< stored status of image*/
    unsigned int quality;               /**< jpeg compress ratio*/
    int rotation;                       /**< Rotation angle of camera input */
    guint num_buffers;                  /**< Number of buffers */
} camerasrc_format_t;

typedef struct _camerasrc_ctrl_query_t {
    int support;                        /**<1: support, 0: Not support, -1: extra support(Non v4l2)*/
    int max;                            /**<Integer max value(includes enums)*/
    int min;                            /**<Integer min value(includes enums)*/
}camerasrc_ctrl_query_t;

typedef struct _camerasrc_exif_t {
    /* Dynamic value */
    unsigned int exposure_time_numerator;       /**< Exposure time, given in seconds */
    unsigned int exposure_time_denominator;
    int shutter_speed_numerator;                /**< Shutter speed, given in APEX(Additive System Photographic Exposure) */
    int shutter_speed_denominator;
    int brigtness_numerator;                    /**< Value of brightness, before firing flash, given in APEX value */
    int brightness_denominator;
    unsigned short int iso;                     /**< Sensitivity value of sensor */
    unsigned short int flash;                   /**< Whether flash is fired(1) or not(0) */
    int metering_mode;                          /**< metering mode in EXIF 2.2 */
    int exif_image_width;                       /**< Size of image */
    int exif_image_height;
    int exposure_bias_in_APEX;                  /**< Exposure bias in APEX standard */
    int software_used;                          /**< Firmware S/W version */
    int focal_len_numerator;                    /**< Lens focal length (f = 4.5mm) */
    int focal_len_denominator;
    int aperture_f_num_numerator;               /**< Aperture value (f_num = 2.8) */
    int aperture_f_num_denominator;
    int aperture_in_APEX;                       /**< Aperture value in APEX standard */
    int max_lens_aperture_in_APEX;              /**< Max aperture value in APEX standard */

    /* Fixed value */
    int component_configuration;                /**< color components arrangement (YCbCr = 1230) */
    int colorspace;                             /**< colorspace information (sRGB=1) */
}camerasrc_exif_t;

typedef struct _camerasrc_frame_data_t {
	int index;
	camerasrc_buffer_t buffer;
}camerasrc_frame_data_t;

typedef struct _camerasrc_capture_data_info {
	int buffer_index;
	gboolean make_thumbnail;
	gboolean flash_activated;
} camerasrc_capture_data_info;

/* JPEG/YUV interleaved data */
#define INTERLEAVED_JPEG_MAX_SIZE               (1024*1024*6)  /* 6 Mbyte */

/* For Query functionalities
   For Querying capabilities */
/*! Use static size of structures for querying because of performance
 */

#define MAX_NUM_FMT_DESC        32
#define MAX_NUM_RESOLUTION      32
#define MAX_NUM_AVAILABLE_TPF   16
#define MAX_NUM_AVAILABLE_FPS   16
#define MAX_NUM_CTRL_LIST_INFO  64
#define MAX_NUM_CTRL_MENU       64
#define MAX_SZ_CTRL_NAME_STRING 32
#define MAX_SZ_DEV_NAME_STRING  32

enum{
    CAMERASRC_FCC_USE_NONE              = 0x00000001,
    CAMERASRC_FCC_USE_REC_PREVIEW       = 0x00000010,
    CAMERASRC_FCC_USE_CAP_PREVIEW       = 0x00000100,
    CAMERASRC_FCC_USE_RECORDING         = 0x00001000,
    CAMERASRC_FCC_USE_NORMAL_CAPTURE    = 0x00010000,
    CAMERASRC_FCC_USE_CONT_CAPTURE      = 0x00100000,
    CAMERASRC_FCC_USE_NUM               = 6,
};

/*! @struct camerasrc_tpf_frac_t
 *  @brief For timeperframe as fraction type
 *  Elapse time consumed by one frame, reverse of FPS
 */
typedef struct {
    int num;
    int den;
}camerasrc_tpf_frac_t;

/*! @struct camerasrc_resolution_t
 *  @brief For querying supported resolutions
 */
typedef struct {
    int w;
    int h;

    /* Available time per frame(tpf) as each pixelformat */
    int num_avail_tpf;
    camerasrc_tpf_frac_t tpf[MAX_NUM_AVAILABLE_TPF];
} camerasrc_resolution_t;

/*! @struct camerasrc_fmt_desc_t
 *  @brief For querying supported format type
 */
typedef struct {
    /* fourcc name of each pixelformat */
    unsigned int fcc;
    int fcc_use;

    /* Available resolutions as each pixelformat */
    int num_resolution;
    camerasrc_resolution_t resolutions[MAX_NUM_RESOLUTION];
} camerasrc_fmt_desc_t;

/*! @struct camerasrc_caps_info_t
 *  @brief For querying image input capabilities
 */
typedef struct {
    char dev_name[MAX_SZ_DEV_NAME_STRING];
    camerasrc_dev_id_t input_id;
    int num_fmt_desc;
    camerasrc_fmt_desc_t fmt_desc[MAX_NUM_FMT_DESC];

    int num_preview_resolution;
    int preview_resolution_width[MAX_NUM_RESOLUTION];
    int preview_resolution_height[MAX_NUM_RESOLUTION];

    int num_capture_resolution;
    int capture_resolution_width[MAX_NUM_RESOLUTION];
    int capture_resolution_height[MAX_NUM_RESOLUTION];

    int num_preview_fmt;
    unsigned int preview_fmt[MAX_NUM_FMT_DESC];

    int num_capture_fmt;
    unsigned int capture_fmt[MAX_NUM_FMT_DESC];

    int num_fps;
    camerasrc_frac_t fps[MAX_NUM_AVAILABLE_FPS];
} camerasrc_caps_info_t;

/* For Querying controls */
enum {
    CTRL_TYPE_RANGE = 0,                                        /**< Integer, range type */
    CTRL_TYPE_BOOL,                                             /**< Boolean type, 1 equals positive and 0 is negative */
    CTRL_TYPE_ARRAY,                                            /**< Array type, also called menu type. each integer(enumeration) value can be set */
    CTRL_TYPE_UNKNOWN,                                          /**< Unknown type, for error control */
    CTRL_TYPE_NUM,
};

/*! @struct camerasrc_ctrl_menu_t
 *  @brief For querying menu of specified controls
 */
typedef struct {
    int menu_index;                                             /**< What number is used for accessing this menu */
    char menu_name[MAX_SZ_CTRL_NAME_STRING];                    /**< name of each menu */
}camerasrc_ctrl_menu_t;

/*! @struct camerasrc_ctrl_info_t
 *  @brief For querying controls detail
 */
typedef struct {
    camerasrc_ctrl_t camsrc_ctrl_id;                             /**< camsrc camera control ID for controlling this */
    int v4l2_ctrl_id;                                           /**< v4l2 ctrl id, user not need to use this. see @struct camerasrc_ctrl_t */
    int ctrl_type;                                              /**< Type of this control */
    char ctrl_name[MAX_SZ_CTRL_NAME_STRING];                    /**< Name of this control */
    int min;                                                    /**< minimum value */
    int max;                                                    /**< maximum value */
    int step;                                                   /**< unit of the values */
    int default_val;                                            /**< Default value of the array or range */
    int num_ctrl_menu;                                          /**< In the case of array type control, number of supported menu information */
    camerasrc_ctrl_menu_t ctrl_menu[MAX_NUM_CTRL_MENU];         /**< @struct camerasrc_ctrl_menu_t for detailed each menu information*/
} camerasrc_ctrl_info_t;

/*! @struct camerasrc_ctrl_list_info_t
 *  @brief For querying controls
 */
typedef struct {
    int num_ctrl_list_info;                                     /**< Number of supported controls */
    camerasrc_ctrl_info_t ctrl_info[MAX_NUM_CTRL_LIST_INFO];    /**< @struct camerasrc_ctrl_info_t for each control information */
} camerasrc_ctrl_list_info_t;


/* capabilities field */
#define CAMERASRC_STROBE_CAP_NONE               0x0000  /* No strobe supported */
#define CAMERASRC_STROBE_CAP_OFF                0x0001  /* Always flash off mode */
#define CAMERASRC_STROBE_CAP_ON                 0x0002  /* Always use flash light mode */
#define CAMERASRC_STROBE_CAP_AUTO               0x0004  /* Flashlight works automatic */
#define CAMERASRC_STROBE_CAP_REDEYE             0x0008  /* Red-eye reduction */
#define CAMERASRC_STROBE_CAP_SLOWSYNC           0x0010  /* Slow sync */
#define CAMERASRC_STROBE_CAP_FRONT_CURTAIN      0x0020  /* Front curtain */
#define CAMERASRC_STROBE_CAP_REAR_CURTAIN       0x0040  /* Rear curtain */
#define CAMERASRC_STROBE_CAP_PERMANENT          0x0080  /* keep turned on until turning off */
#define CAMERASRC_STROBE_CAP_EXTERNAL           0x0100  /* use external strobe */

typedef struct _camerasrc_extra_info_t{
    unsigned int strobe_caps;                                   /**< Use above caps field */
    unsigned int detection_caps;                                /**< Just boolean */
    unsigned int reserved[4];
} camerasrc_extra_info_t;
/* END For Query functionalities */

/*! @def CAMERASRC_SET_SIZE_BY_DIMENSION
 *  @brief Set image size
 */
#define CAMERASRC_SET_SIZE_BY_DIMENSION(format, img_width, img_height) { \
    format.img_size.width = img_width; \
    format.img_size.height = img_height; \
}

/*! @def CAMERASRC_SET_CAPTURE_SIZE_BY_DIMENSION
 *  @brief Set captured image size
 */
#define CAMERASRC_SET_CAPTURE_SIZE_BY_DIMENSION(format, img_width, img_height) { \
    format.capture_size.width = img_width; \
    format.capture_size.height = img_height; \
}


/* CALLBACK DEFINITIONS */
/*! @typedef camerasrc_callback_t
 *  @brief Called back when auto-focusing returns
 *  This callback will be called when the lens properly auto-focused
 */
typedef int (*camerasrc_callback_t) (camsrc_handle_t handle, int state, void* usr_data);


/* FUNCTION DEFINITIONS */

/**** M A I N    O P E R A T I O N ****/

/**
 * allocate the handle, set initial state & settings
 *
 * @param[in]		phandle ::camsrc_handle_t camerasrc context handle to be created
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t code
 * @see			camerasrc_destroy
 * @note		State transition : [CAMERASRC_STATE_NONE] => [CAMERASRC_STATE_CREATED]
 *			Phase description : Non-running phase
 */
int camerasrc_create(camsrc_handle_t *phandle);

/**
 * proceed fd close, other finalization routines
 *
 * @param[in] handle ::camsrc_handle_t camerasrc context handle
 *
 * @return Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t code
 *
 * @see <camerasrc_create>
 *
 * @note State transition : [CAMERASRC_STATE_UNREALIZED] => [CAMERASRC_STATE_DESTROYED]
 *       Phase description : Non-running phase
 */
int camerasrc_destroy(camsrc_handle_t handle);

/**
 * Get the state of camerasrc context handle
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[out]		state  ::camerasrc_state_t camerasrc context current state
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t code
 *
 */
int camerasrc_get_state(camsrc_handle_t handle, camerasrc_state_t* state);

/**
 * Allocate the device context handle, open device node and do the miscellaneous settings
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @param[in]		camera_id ::Camera ID currently set
 * @param[in]		mode ::Camera or Video
 * @return		Success on ::camerasrc_handle_t or returns NULL code, and displays debug message
 * @see			camerasrc_unrealize
 * @note		State transition : [CAMERASRC_STATE_CREATED] => [CAMERASRC_STATE_REALIZED]
 *			Phase description : Non-running phase
 *			device name can be dependent on kernel module
 */
int camerasrc_realize(camsrc_handle_t handle, camerasrc_dev_id_t camera_id, camerasrc_sensor_mode_t mode);

/**
 * Deallocate the device structure of buffers, close device
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @return		Success on ::camerasrc_handle_t or returns NULL code, and displays debug message
 * @see			camerasrc_realize
 * @note		State transition : [CAMERASRC_STATE_READY] => [CAMERASRC_STATE_UNREALIZED]
 *			Phase description : Transit to Non-running phase
 */
int camerasrc_unrealize(camsrc_handle_t handle);

/**
 * Prepare Handle to be ready to capture
 * Can change settings like below at this state
 * - camera device ID setting
 * - color format setting
 * - image size setting
 * - image storing method
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @return		Success on ::camerasrc_handle_t or returns NULL code, and displays debug message
 * @see			camerasrc_stop
 * @note		State transition : [CAMERASRC_STATE_REALIZED] => [CAMERASRC_STATE_READY]
 *			Phase description : Running phase
 */
int camerasrc_start(camsrc_handle_t handle);

/**
 * Get total number of buffer which managed in camerasrc currently.
 * If this called, it will return default number of buffer in MMAP mode.
 * but use this API after calling ::camerasrc_present_usr_buffer , It will
 * return User specfied buffer number.
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[in]		num_buffer Number of buffer that's managed in camerasrc currently
 * @return		Success on ::camsrc_handle_t or returns NULL code, and displays debug message
 * @see			camerasrc_io_method_t
 *
 */
int camerasrc_get_num_buffer(camsrc_handle_t handle, unsigned int* num_buffer);

/**
 * Inner ring buffer start refreshing. refresh process occurs asynchronously, and
 * ::camerasrc_wait_frame_available function can anounce when it is available.
 *
 * Camera is grabbing low quality, high speed frame
 * - Can attempt the [AF] state only at this state
 * - preview frames are always automatically fed
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		State transition : [CAMERASRC_STATE_READY] => [CAMERASRC_STATE_PREVIEW]
 *			Phase description : Running phase
 */
int camerasrc_start_preview_stream(camsrc_handle_t handle);

int camerasrc_create_buffer(camsrc_handle_t handle);
int camerasrc_destroy_buffer(camsrc_handle_t handle);

/**
 * Stop frame refreshing. Ring buffers don't be refreshed any more
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @return		Success on ::camerasrc_handle_t or returns NULL code, and displays debug message
 * @see			camerasrc_stop
 * @note		State transition : [CAMERASRC_STATE_STILL/PREVIEW/VIDEO] => [CAMERASRC_STATE_READY]
 *			Phase description : Running phase
 */
int camerasrc_stop_stream(camsrc_handle_t handle);

/**
 * non-busy waiting function for image frame available
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[in]		timeout ::int timeout
 * @return		Success on ::camsrc_handle_t or returns NULL code, and displays debug message
 *
 */
int camerasrc_wait_frame_available(camsrc_handle_t handle, int timeout);

/**
 * Queue(in user space, almost same with free buffer) buffer to dev's ring buffer
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @param[in]		buf_index ::int index of buffer
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 *
 */
int camerasrc_queue_buffer(camsrc_handle_t handle, int buf_index);

/**
 * Dequeue(Pop) buffer from v4l2 driver to usr space.
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[out]		buf_index main buffer index number to be dequeued.
 * @param[out]		buffer main image buffer
 * @param[out]		thm_buffer thumbnail image buffer
 * @return		Success on ::camsrc_handle_t or returns NULL code, and displays debug message
 *
 */
int camerasrc_dequeue_buffer(camsrc_handle_t handle, int *buf_index, camerasrc_buffer_t *buffer, camerasrc_buffer_t *thm_buffer);

/**
 * Read frame from camera device.
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[out]		buffer ::camerasrc_buffer_t main image buffer to be get.
 * @param[out]		thm_buffer ::camerasrc_buffer_t thumbnail image buffer to be get.
 * @param[out]		buffer_index ::int v4l2 buffer index.
 * @note		if thm_buffer is NULL, thumbnail image will be discarded
 *
 */
int camerasrc_read_frame(camsrc_handle_t handle, camerasrc_buffer_t *main_img_buffer, camerasrc_buffer_t *thm_img_buffer, int *buffer_index);

/**
 * Get screennail buffer
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[out]		scrnl_buf ::camerasrc_buffer_t screennail buffer to be gotten
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 */
int camerasrc_get_screennail_buffer(camsrc_handle_t handle, camerasrc_buffer_t *scrnl_buf);

/**
 * Set autofocus callback. ::camerasrc_callback_t type defined function can be set
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @param[in]		cb ::camerasrc_callback_t callback after focusing over
 * @param[in]		use_data ::void * user data pointer that will be passed to callback
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		Callback function can be set on READY or REALIZED state
 *
 */
int camerasrc_set_focused_callback(camsrc_handle_t handle, camerasrc_callback_t cb, void *usr_data);

/**
 * Set autofocusing area. autofocusing will be performed refer this rect of the preview
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[in]		rect ::camerasrc_rect_t rectangle area for auto focusing
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 */
int camerasrc_set_autofocusing_area(camsrc_handle_t handle, camerasrc_rect_t* rect);

/**
 * Get autofocusing area.
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[out]		rect ::camerasrc_rect_t rectangle area for auto focusing
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 */
int camerasrc_get_autofocusing_area(camsrc_handle_t handle, camerasrc_rect_t* rect);

/**
 * Start auto focusing with ::camerasrc_af_mode. After ::interval time, call the callback
 * function with ::camerasrc_af_status value.
 * Auto-focus is in progress. Cant return preview state
 * before time-out, success, call camerasrc_autofocus_stop
 * - If focused, focus status will be locked at preview state
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @param[in]		af_mode Auto focusing operation mode see ::camerasrc_af_mode
 * @param[in]		interval interval time in millisecond
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		State transition : [CAMERASRC_STATE_PREVIEW] => [CAMERASRC_STATE_AF_IN_PROGRESS]
 *			Phase description : Running phase
 */
int camerasrc_start_autofocusing(camsrc_handle_t handle);

/**
 * Stop auto focusing
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		State transition : [CAMERASRC_STATE_AF_IN_PROGRESS] => [CAMERASRC_STATE_PREVIEW]
 *			Phase description : Running phase
 */
int camerasrc_stop_autofocusing(camsrc_handle_t handle);

/**
 * Release auto focusing
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		State transition : [CAMERASRC_STATE_AF_IN_PROGRESS] => [CAMERASRC_STATE_PREVIEW]
 *			Phase description : Running phase
 */
int camerasrc_release_autofocusing(camsrc_handle_t handle);

/**
 * Initialize auto focusing mode to specified focal length
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[in]		af_mode ::camerasrc_af_mode_t Auto focusing mode
 * @param[in]		af_range ::camerasrc_af_scan_range_t Auto focusing range
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 */
int camerasrc_init_autofocusing_mode(camsrc_handle_t handle, camerasrc_af_mode_t af_mode, camerasrc_af_scan_range_t af_range);

/**
 * Get current auto focusing mode
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[out]		af_mode ::camerasrc_af_mode_t Auto focusing mode
 * @param[out]		af_range ::camerasrc_af_scan_range_t Auto focusing range
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 */
int camerasrc_get_autofocusing_mode(camsrc_handle_t handle, camerasrc_af_mode_t* af_mode, camerasrc_af_scan_range_t* af_range);

/**
 * Get current auto focusing status
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[out]		af_status ::camerasrc_auto_focus_status_t Auto focusing status
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 *
 */
int camerasrc_get_autofocusing_status(camsrc_handle_t handle, camerasrc_auto_focus_status_t* af_status);


/**** E F F E C T    C O N T R O L    O P E R A T I O N ****/

/**
 * Control miscellaneous settings through ::camerasrc_ctrl_t IDs
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[in]		ctrl_id control ID to be checked
 * @param[in]		value value to be set
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		This function is only effective at CAMERASRC_STATE_READY
 */
int camerasrc_set_control(camsrc_handle_t handle, camerasrc_ctrl_t ctrl_id, int value);

/**
 * Get the value of miscellaneous settings through ::camerasrc_ctrl_t IDs
 *
 * @param[in]		handle ::camsrc_handle_t camerasrc context handle
 * @param[in]		ctrl_id control ID to be checked
 * @param[out]		value value to be stored
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		This function is only effective at CAMERASRC_STATE_READY
 */
int camerasrc_get_control(camsrc_handle_t handle, camerasrc_ctrl_t ctrl_id, int* value);

/**** O U T P U T    C O N T R O L    O P E R A T I O N ****/

/**
 * Control frame refresh rate setting
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @param[in]		frac ::camsrc_frac_t time per frame
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		This function is only effective at CAMERASRC_STATE_READY
 */
int camerasrc_set_timeperframe(camsrc_handle_t handle, camerasrc_frac_t* frac);

/**
 * Set output format of camera device just like ioctl VIDIOC_S_FMT
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @param[in]		fmt ::camerasrc_format_t output format description to be set
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		This function is only effective at CAMERASRC_STATE_READY.
 *			device dependent function.
 */
int camerasrc_set_format(camsrc_handle_t handle, camerasrc_format_t* fmt);

/**
 * Get output format of camera device just like ioctl VIDIOC_G_FMT
 *
 * @param[in]		handle ::camerasrc_handle_t camerasrc context handle
 * @param[in]		fmt ::camerasrc_format_t output format description to be set
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 * @note		This function is only effective at CAMERASRC_PHASE_RUNNING
 */
int camerasrc_get_format(camsrc_handle_t handle, camerasrc_format_t* fmt);

/**
 * Get data of frame
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		data ::camerasrc_frame_data_t data
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_get_frame_data(camsrc_handle_t handle, camerasrc_frame_data_t *data);

/****  S H U T T E R   S P E E D   &   A P E R T U R E   M O D U L A T I O N  ****/
/**
 * Get exif string to be combined with jpg image from camerasrc
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		exif_string exif information string
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_get_exif_info(camsrc_handle_t handle, camerasrc_exif_t* exif_struct);

/**
 * Set vflip to camera driver

 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		vflip ::int vflip
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 */
int camerasrc_set_vflip(camsrc_handle_t handle, int vflip);

/**
 * Set hflip to camera driver

 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		hflip ::int hflip
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error_t error code
 */
int camerasrc_set_hflip(camsrc_handle_t handle, int hflip);


/* W I L L   B E   D E P R E C A T E D */

/*! @enum camerasrc_colortone_t
 *  @brief Enumeration type for camera colortone
 *
 *  Colortone entries for camera effects. This can be used with CAMERASRC_CTRL_COLOR_TONE
 *  This values are defined for utility. It's dependent on the device/camera module.
 */
typedef enum {
    CAMERASRC_COLORTONE_NONE = 0,
    CAMERASRC_COLORTONE_NEGATIVE,
    CAMERASRC_COLORTONE_SOLARIZATION_1,
    CAMERASRC_COLORTONE_SOLARIZATION_2,
    CAMERASRC_COLORTONE_SOLARIZATION_3,
    CAMERASRC_COLORTONE_SOLARIZATION_4,
    CAMERASRC_COLORTONE_EMBOSS,
    CAMERASRC_COLORTONE_OUTLINE,
    CAMERASRC_COLORTONE_AQUA,
    CAMERASRC_COLORTONE_SEPHIA,
    CAMERASRC_COLORTONE_GRAY,
    CAMERASRC_COLORTONE_B_N_W,
    CAMERASRC_COLORTONE_RED,
    CAMERASRC_COLORTONE_GREEN,
    CAMERASRC_COLORTONE_BLUE,
    CAMERASRC_COLORTONE_ANTIQUE,
    CAMERASRC_COLORTONE_SKETCH1,
    CAMERASRC_COLORTONE_SKETCH2,
    CAMERASRC_COLORTONE_SKETCH3,
    CAMERASRC_COLORTONE_SKETCH4,
    CAMERASRC_COLORTONE_NUM,
}camerasrc_colortone_t;

/*! @enum camerasrc_program_mode_t
 *  @brief Enumeration type for preset program mode
 *
 *  WRITEME
 */
typedef enum {
    CAMERASRC_PROGRAM_MODE_NORMAL = 0,
    CAMERASRC_PROGRAM_MODE_PORTRAIT,
    CAMERASRC_PROGRAM_MODE_LANDSCAPE,
    CAMERASRC_PROGRAM_MODE_SPORTS,
    CAMERASRC_PROGRAM_MODE_PARTY_N_INDOOR,
    CAMERASRC_PROGRAM_MODE_BEACH_N_INDOOR,
    CAMERASRC_PROGRAM_MODE_SUNSET,
    CAMERASRC_PROGRAM_MODE_DUSK_N_DAWN,
    CAMERASRC_PROGRAM_MODE_FALL_COLOR,
    CAMERASRC_PROGRAM_MODE_NIGHT_SCENE,
    CAMERASRC_PROGRAM_MODE_FIREWORK,
    CAMERASRC_PROGRAM_MODE_TEXT,
    CAMERASRC_PROGRAM_MODE_SHOW_WINDOW,
    CAMERASRC_PROGRAM_MODE_CANDLE_LIGHT,
    CAMERASRC_PROGRAM_MODE_BACK_LIGHT,
    CAMERASRC_PROGRAM_MODE_NUM,
}camerasrc_program_mode_t;

/*! @enum camerasrc_whitebalance_t
 *  @brief Enumeration type for preset whitebalance
 *
 *  WRITEME
 */
typedef enum {
    CAMERASRC_WHITEBALANCE_AUTO = 0,
    CAMERASRC_WHITEBALANCE_INCANDESCENT,
    CAMERASRC_WHITEBALANCE_FLUORESCENT,
    CAMERASRC_WHITEBALANCE_DAYLIGHT,
    CAMERASRC_WHITEBALANCE_CLOUDY,
    CAMERASRC_WHITEBALANCE_SHADE,
    CAMERASRC_WHITEBALANCE_HORIZON,
    CAMERASRC_WHITEBALANCE_FLASH,
    CAMERASRC_WHITEBALANCE_CUSTOM,
    CAMERASRC_WHITEBALANCE_NUM,
}camerasrc_whitebalance_t;

/**
 * Enumerations for flip.
 */
typedef enum {
    CAMERASRC_FILP_NONE = 0,            /**< Not flipped */
    CAMERASRC_FILP_VERTICAL,            /**< Flip vertically */
    CAMERASRC_FILP_HORIZONTAL,          /**< Flip horizontally */
    CAMERASRC_FILP_NUM,                 /**< Number of flip status */
}camerasrc_flip_t;

/*! @enum camerasrc_strobo_status_t
 *  @brief strobo status
 *  strobo status
 */
typedef enum {
    CAMERASRC_STROBO_STATUS_BANNED = 0, /**< strobo off*/
    CAMERASRC_STROBO_STATUS_FORCE_ON,   /**< strobo on.*/
    CAMERASRC_STROBO_STATUS_AUTO,       /**< control strobo automatically*/
    CAMERASRC_STROBO_STATUS_MOVIE_ON,   /**< control strobo automatically*/
    CAMERASRC_STROBO_STATUS_NUM,        /**< Number of AF status*/
}camerasrc_strobo_status_t;
/*! @enum camerasrc_strobe_mode_t
 *  @brief strobe mode
 *  strobe mode
 */
typedef enum {
    CAMERASRC_STROBE_MODE_OFF = 1,      /**< off */
    CAMERASRC_STROBE_MODE_AUTO,         /**< auto */
    CAMERASRC_STROBE_MODE_ON,           /**< on */
    CAMERASRC_STROBE_MODE_PERMANENT,    /**< permanent */
    CAMERASRC_STROBE_MODE_NUM,          /**< Number of strobe mode */
}camerasrc_strobe_mode_t;

/*! @enum camerasrc_ae_mode_t
 *  @brief Auto exposure mode
 *  Auto exposure operation mode
 */
typedef enum {
    CAMERASRC_AE_MODE_OFF = 0,
    CAMERASRC_AE_MODE_ALL,
    CAMERASRC_AE_MODE_CENTER_WEIGHTED_AVR_1,
    CAMERASRC_AE_MODE_CENTER_WEIGHTED_AVR_2,
    CAMERASRC_AE_MODE_CENTER_WEIGHTED_AVR_3,
    CAMERASRC_AE_MODE_SPOT_1,
    CAMERASRC_AE_MODE_SPOT_2,
    CAMERASRC_AE_MODE_CUSTOM_1,
    CAMERASRC_AE_MODE_CUSTOM_2,
} camerasrc_ae_mode_t;

/*! @enum camerasrc_iso_t
 *  @brief Reserved iso number in definition
 *  Traditionally predefined ISO values
 */
typedef enum {
    CAMERASRC_ISO_AUTO = 0,
    CAMERASRC_ISO_50,
    CAMERASRC_ISO_100,
    CAMERASRC_ISO_200,
    CAMERASRC_ISO_400,
    CAMERASRC_ISO_800,
    CAMERASRC_ISO_1600,
    CAMERASRC_ISO_3200,
} camerasrc_iso_t;


/**
 * Set the mode of strobe
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		mode ::camerasrc_strobe_mode_t mode of strobe
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_set_strobe_mode(camsrc_handle_t handle, camerasrc_strobe_mode_t mode);

/**
 * Get the mode of strobe
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		mode ::camerasrc_strobe_mode_t mode of strobe
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_get_strobe_mode(camsrc_handle_t handle, camerasrc_strobe_mode_t* mode);

/**
 * Set the mode of auto-exposure processing
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		ae_mode ::camerasrc_ae_mode_t AE mode to be set
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_set_exposure_mode(camsrc_handle_t handle, camerasrc_ae_mode_t ae_mode);

/**
 * Get the mode of auto-exposure processing
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		ae_mode ::camerasrc_ae_mode_t AE mode to be got
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_get_exposure_mode(camsrc_handle_t handle, camerasrc_ae_mode_t* ae_mode);

/**
 * Set the shutter speed
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		frac ::camerasrc_frac_t shutter speed to be set
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_set_shutter_speed(camsrc_handle_t handle, camerasrc_frac_t frac);

/**
 * Get the shutter speed
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		frac ::camerasrc_frac_t shutter speed to be got
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_get_shutter_speed(camsrc_handle_t handle, camerasrc_frac_t* frac);

/**
 * Set the exposure value
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		frac ::camerasrc_frac_t exposure value to be set
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_set_exposure_value(camsrc_handle_t handle, camerasrc_frac_t frac);

/**
 * Get the exposure value
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		frac ::camerasrc_frac_t exposure value to be got
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_get_exposure_value(camsrc_handle_t handle, camerasrc_frac_t* frac);

/**
 * Extract EXIF info of current captured image
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		capture_data ::camerasrc_capture_data_info * pointer of capture data information
 * @return		Success on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_extract_exif_info_from_capture_data(camsrc_handle_t handle, camerasrc_capture_data_info *capture_data);


/*-------------------
 *  For TBM control *
 -------------------*/
/**
 * Initialize Tizen Buffer Manager
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @return		Success on TRUE or returns FALSE
 */
int camerasrc_tbm_init(camsrc_handle_t handle);

/**
 * Deinitialize Tizen Buffer Manager
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @return		Success on TRUE or returns FALSE
 */
int camerasrc_tbm_deinit(camsrc_handle_t handle);

/**
 * Memory allocation function for TBM
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		size ::int size
 * @param[out]		bo ::TBM buffer object of allocated memory
 * @param[out]		dma_buf_fd ::dmabuf fd of allocated memory
 * @param[out]		vaddr ::virtual address of allocated memory
 * @return		Success on TRUE or returns FALSE
 */
int camerasrc_tbm_alloc_buffer(camsrc_handle_t handle, int size, tbm_bo *bo, int *dma_buf_fd, unsigned char **vaddr);

/* For Query functionalities */
/**
 * Query basic device info of device
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		camerasrc_caps_info_t device information structure
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_read_basic_dev_info(camerasrc_dev_id_t dev_id, camerasrc_caps_info_t* caps_info);

/**
 * Query miscellaneous device info(effect, WB, preset values, etc.) of device
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		camerasrc_ctrl_list_info_t device capabilities structure
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_read_misc_dev_info(camerasrc_dev_id_t dev_id, camerasrc_ctrl_list_info_t* ctrl_info);

/**
 * Query extra device info(face detection, strobe, etc.) of device
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		camerasrc_extra_info_t device capabilities structure
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_read_extra_dev_info(camerasrc_dev_id_t dev_id, camerasrc_extra_info_t* extra_info);

/**
 * Record miscellaneous device info(effect, WB, preset values, etc.) of device
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		camerasrc_ctrl_list_info_t device capabilities structure
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_write_basic_dev_info(camsrc_handle_t handle, camerasrc_caps_info_t* caps_info);

/**
 * Record miscellaneous device info(effect, WB, preset values, etc.) of device
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[in]		camerasrc_ctrl_list_info_t device capabilities structure
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_write_misc_dev_info(camsrc_handle_t handle, camerasrc_ctrl_list_info_t* ctrl_info);

/**
 * Record extra device info(face detection, strobe, etc.) of device
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		camerasrc_extra_info_t device capabilities structure
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_write_extra_dev_info(camsrc_handle_t handle, camerasrc_extra_info_t* extra_info);

/**
 * Query to device driver about miscellaneous device info(effect, WB, preset values, etc.) of device
 *
 * @param[in]		handle ::camsrc_handle_t handle
 * @param[out]		camerasrc_ctrl_list_info_t device capabilities structure
 * @return		Success(Support) on CAMERASRC_ERR_NONE or returns with ::camerasrc_error error code
 */
int camerasrc_query_misc_dev_info(camsrc_handle_t handle, camerasrc_ctrl_list_info_t* ctrl_list_info);

/* END For Query functionalities */

#ifdef __cplusplus
}
#endif

#endif /*__CAMERASRC_H__*/
