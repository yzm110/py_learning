LOCAL_PATH := $(call my-dir)
my_makefile := $(lastword $(MAKEFILE_LIST))

include $(CLEAR_VARS)

LOCAL_MODULE := libhdm_utility
LOCAL_MODULE_CLASS := CMAKE
LOCAL_INCREMENTAL_FLAGS := true
LOCAL_CPPFLAGS := -std=c++11 -ffast-math -use_fast_math -pthread -fexceptions -fPIC -Werror=return-type

LOCAL_BUILT_SHARED_LIBRARIES := libhdm_utility.so
LOCAL_PREFIX := /usr/app

LOCAL_NOTICE_FILE := LICENSE

LOCAL_BINARY_RELEASE_MK := $(my_makefile)

LOCAL_EXPORT_C_INCLUDE_DIRS := $(LOCAL_PATH)/include \
        $(LOCAL_PATH)/include/hdm_utility 

#LOCAL_EXPORT_CMAKE_CONFIG_DIRS := $$<PREFIX>

LOCAL_3RD_PARTY := ${LOCAL_PATH}/../../3rd_party
ALGO_3RD_PARTY := $(ALGO_3RD_PARTY)
ALGO_COMMON_PATH := $(LOCAL_PATH)/../../../common
# LOCAL_COMMON_PATH := $(LOCAL_PATH)/../../common

LOCAL_C_INCLUDES := \
        $(LOCAL_PATH)/include \
        $(ALGO_COMMON_PATH)/common/include \
        $(ALGO_3RD_PARTY)/include \
        $(LOCAL_3RD_PARTY)/navinfo/queryquery/include \
        $(ALGO_3RD_PARTY)/geographic_transform/include

# LOCAL_SRC_FILES := \
#         src/hdm_coords_converter.cpp \
#         src/hdm_date_time.cpp \
#         src/hdm_dbg_log.cpp \
#         src/hdm_ehr_output.cpp \
#         src/hdm_geometry.cpp \
#         src/hdm_ini_config.cpp \
#         src/hdm_mutex.cpp \
#         src/hdm_system.cpp \
#         src/hdm_thread.cpp

ifeq ($(TARGET_DEVICE),  x86_64)
	# LOCAL_SHARED_LIBRARIES := libcommon  # x86 call static lib for gsl
	LOCAL_CONFIG_FLAGS += -DBUILD_SHARED_LIBS=ON -DZOP_PLATFORM=x86 -DUNIT_TEST=OFF
else ifeq ($(TARGET_DEVICE), xavier-qnx)
	# LOCAL_SHARED_LIBRARIES := libcommon libgsl libgslcblas
	LOCAL_CONFIG_FLAGS += -DBUILD_SHARED_LIBS=ON -DZOP_PLATFORM=qnx -DUNIT_TEST=OFF
endif
LOCAL_CONFIG_FLAGS += -DZONE_ROS2=OFF
LOCAL_SHARED_LIBRARIES := libgeographic_transform libGeographic libsovp_asf_log libsovp-xlog

# LOCAL_STATIC_LIBRARIES := libsparse_static libext4_utils_static libcutils

include $(BUILD_EXTERNAL_PROJECT)
