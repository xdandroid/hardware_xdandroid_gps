ifneq ($(BOARD_VENDOR_XDANDROID_GPS_HARDWARE),)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE := gps.$(BOARD_VENDOR_XDANDROID_GPS_HARDWARE)

LOCAL_CFLAGS           += -DHAVE_GPS_HARDWARE

LOCAL_C_INCLUDES += \
	$(TARGET_OUT_HEADERS)/librpc

LOCAL_SRC_FILES += \
	gps.c \
	gps_msm7k.c \
	gps-rpc.c \
	time.cpp

LOCAL_SHARED_LIBRARIES += \
	libutils \
	libcutils \
	librpc

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

include $(BUILD_SHARED_LIBRARY)

endif
