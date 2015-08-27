# plugindir is set in configure

plugin_LTLIBRARIES = libgstcamerasrc.la

# sources used to compile this plug-in
libgstcamerasrc_la_SOURCES = gstcamerasrccontrol.c \
                             gstcamerasrccolorbalance.c \
                             camerasrc.c \
                             camerasrc-internal.c \
                             gstcamerasrc.c

libgstcamerasrc_la_CFLAGS = $(GST_CFLAGS) \
                            $(GST_BASE_CFLAGS) \
                            $(GST_VIDEO_FLAGS) \
                            $(MM_COMMON_CFLAGS) \
                            -I$(srcdir)/include \
                            $(GST_INTERFACES_CFLAGS) \
                            $(TBM_CFLAGS) \
                            $(DRM_CFLAGS) \
                            $(DRM_EXYNOS_CFLAGS)

libgstcamerasrc_la_LIBADD = $(GST_LIBS) \
                            $(GST_BASE_LIBS) \
                            $(GST_VIDEO_LIBS) \
                            $(MM_COMMON_LIBS) \
                            -ldl -lpthread -lrt -lm \
                            $(GST_INTERFACES_LIBS) \
                            $(TBM_LIBS) \
                            $(DRM_LIBS) \
                            $(DRM_EXYNOS_LIBS)

libgstcamerasrc_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS)

libgstcamerasrc_la_CPPFLAGS = -I$(srcdir)/include

